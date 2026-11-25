"""Independent checks and figures for the lab reports.

The script re-derives with SciPy what the firmware computes in C, so that the
numbers quoted in the reports do not depend on the code under test alone:

  * the filter coefficients hard-coded in shot_detect.c / stability.c are
    compared with a fresh Butterworth design;
  * the accel and gyro envelopes of the control example are recomputed from
    the recorded input signal and compared with the values the detector
    printed in its own journal;
  * the moment of the first release is recomputed by hand from formula (onset
    interpolation) and compared with the reported one.

Run after tools/run_host_tests.ps1 (it needs results/control_example):

    python tools/analysis.py

Outputs: results/analysis/*.txt and results/figures/*.png.
"""

from __future__ import annotations

import csv
import math
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.ticker import FuncFormatter  # noqa: E402
from scipy import signal  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
RESULTS = ROOT / "results"
EXAMPLE = RESULTS / "control_example"
OUT_TXT = RESULTS / "analysis"
OUT_FIG = RESULTS / "figures"

# ---- constants copied from the firmware sources -------------------------

FS = 200  # detector sample rate, Hz
FS_FRAME = 50  # decimated stream, Hz
BOOT_MS = 60_000  # control_example.c: session starts 60 s after power-on

# shot_detect.c
SD_HP = dict(b=[0.80059240, -1.60118480, 0.80059240], a=[1.0, -1.56101808, 0.64135154])
SD_RMS_WIN = 5
SD_TRIG_K, SD_TRIG_FLOOR = 6.0, 1.5
SD_GYRO_TRIG_K, SD_GYRO_TRIG_FLOOR = 8.0, 0.5
SD_CONF_K, SD_STRONG_K = 3.0, 25.0
SD_VALID_DELAY, SD_VALID_END = 6, 40  # samples after the onset crossing
SD_ENVELOPE_DELAY_US = 12_000

# stability.c
STAB_BP = np.array(
    [
        [0.0200833656, 0.0401667311, 0.0200833656, 1.0, -1.5977831770, 0.6929224132],
        [1.0, -2.0, 1.0, 1.0, -1.9207935713, 0.9255748202],
    ]
)

# main.cpp
MA_WINDOW = 4

# ---- figure style -------------------------------------------------------

BLUE, ORANGE = "#2a78d6", "#eb6834"  # categorical slots 1-2, validated on white
INK, INK2, MUTED = "#0b0b0b", "#52514e", "#898781"
GRID, AXIS = "#e1e0d9", "#c3c2b7"

plt.rcParams.update(
    {
        "font.family": ["Segoe UI", "DejaVu Sans"],
        "font.size": 9,
        "axes.edgecolor": AXIS,
        "axes.labelcolor": INK2,
        "axes.titlesize": 9.5,
        "axes.titleweight": "bold",
        "axes.titlecolor": INK,
        "axes.grid": True,
        "axes.axisbelow": True,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "grid.color": GRID,
        "grid.linewidth": 0.6,
        "xtick.color": MUTED,
        "ytick.color": MUTED,
        "xtick.labelcolor": INK2,
        "ytick.labelcolor": INK2,
        "legend.frameon": False,
        "legend.fontsize": 8.5,
        "savefig.dpi": 220,
        "savefig.facecolor": "white",
        "figure.facecolor": "white",
    }
)

WIDTH_IN = 6.5  # text width of the report page (16.5 cm)


def comma(value: float, _pos=None) -> str:
    """Tick label with a decimal comma, as in the report text."""
    return f"{value:g}".replace(".", ",").replace("-", "−")


def write_lines(name: str, lines: list[str]) -> None:
    OUT_TXT.mkdir(parents=True, exist_ok=True)
    (OUT_TXT / name).write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"-- {name}")
    for line in lines:
        print("   " + line)


# ---- 1. filters ---------------------------------------------------------


def gain(sos: np.ndarray, f: float, fs: float) -> float:
    _, h = signal.sosfreqz(sos, worN=[f], fs=fs)
    return float(abs(h[0]))


def ma_gain(f: np.ndarray, m: int, fs: float) -> np.ndarray:
    """Magnitude response of an m-sample moving average."""
    x = np.pi * np.asarray(f, dtype=float) / fs
    with np.errstate(divide="ignore", invalid="ignore"):
        h = np.abs(np.sin(m * x) / (m * np.sin(x)))
    return np.where(np.isclose(x, 0.0), 1.0, h)


def check_filters() -> None:
    lines: list[str] = []

    hp_fw = np.array([SD_HP["b"] + SD_HP["a"]])
    hp_ref = signal.butter(2, 10, "highpass", fs=FS, output="sos")
    bp_ref = signal.butter(2, [0.5, 3.0], "band", fs=FS_FRAME, output="sos")

    lines.append("Highpass 10 Hz, order 2, fs = 200 Hz (shot_detect.c)")
    lines.append(f"  max |firmware - scipy| coefficient difference: {np.max(np.abs(hp_fw - hp_ref)):.2e}")
    for f in (1, 2, 5, 10, 20, 38, 55):
        lines.append(f"  |H({f:>2} Hz)| = {gain(hp_fw, f, FS):.4f}  ({20 * math.log10(gain(hp_fw, f, FS)):6.1f} dB)")
    poles = np.roots(SD_HP["a"])
    lines.append(f"  pole radius: {abs(poles[0]):.4f}")

    lines.append("")
    lines.append("Bandpass 0.5-3 Hz, order 2 (two biquads), fs = 50 Hz (stability.c)")
    lines.append(f"  max |firmware - scipy| coefficient difference: {np.max(np.abs(STAB_BP - bp_ref)):.2e}")
    for f in (0.1, 0.25, 0.5, 1.0, 1.5, 3.0, 6.0, 10.0, 15.0):
        lines.append(f"  |H({f:>5} Hz)| = {gain(STAB_BP, f, FS_FRAME):.4f}  ({20 * math.log10(gain(STAB_BP, f, FS_FRAME)):6.1f} dB)")
    radii = [abs(np.roots([1.0, s[4], s[5]])[0]) for s in STAB_BP]
    lines.append("  pole radii: " + ", ".join(f"{r:.4f}" for r in radii))
    bp_200 = signal.butter(2, [0.5, 3.0], "band", fs=FS, output="sos")
    radii_200 = [abs(np.roots([1.0, s[4], s[5]])[0]) for s in bp_200]
    lines.append("  the same filter designed for 200 Hz, pole radii: " + ", ".join(f"{r:.4f}" for r in radii_200))

    lines.append("")
    lines.append(f"Moving average, {MA_WINDOW} samples, fs = 200 Hz (main.cpp)")
    f = np.linspace(0.01, 100.0, 200_000)
    h = ma_gain(f, MA_WINDOW, FS)
    f3 = f[np.argmax(h < math.sqrt(0.5))]
    lines.append(f"  -3 dB frequency: {f3:.1f} Hz")
    lines.append(f"  |H(12 Hz)| = {float(ma_gain(np.array([12.0]), MA_WINDOW, FS)[0]):.4f} (top of the tremor band)")
    lines.append(f"  group delay: {(MA_WINDOW - 1) / 2 / FS * 1000:.1f} ms")
    lines.append(f"  white-noise reduction: {10 * math.log10(MA_WINDOW):.2f} dB")
    lines.append("  zeros at: " + ", ".join(f"{k * FS / MA_WINDOW:.0f} Hz" for k in range(1, MA_WINDOW // 2 + 1)))

    lines.append("")
    lines.append("Time constants of the exponential averages, tau = -1 / (fs * ln(1 - alpha))")
    for name, fs, alpha in (
        ("hold gate (gyro mean square)", FS, 0.0166),
        ("envelope baselines", FS, 0.005),
        ("stability: rate term", FS_FRAME, 0.20),
        ("stability: sway and drift terms", FS_FRAME, 0.10),
        ("stability: reference orientation", FS_FRAME, 0.04),
        ("stability: display smoothing", FS_FRAME, 0.25),
    ):
        tau = -1.0 / (fs * math.log(1.0 - alpha))
        lines.append(f"  {name:<34} alpha = {alpha:<6}  tau = {tau * 1000:6.0f} ms")

    write_lines("filters.txt", lines)


def figure_filters() -> None:
    fig, axes = plt.subplots(1, 3, figsize=(WIDTH_IN, 2.5), constrained_layout=True)

    def decorate(ax, title, xlabel):
        ax.set_title(title, loc="left")
        ax.set_xlabel(xlabel)
        ax.set_ylim(-60, 5)
        ax.set_yticks([-60, -40, -20, 0])
        ax.yaxis.set_major_formatter(FuncFormatter(comma))

    f = np.linspace(0.05, 100, 4000)
    ax = axes[0]
    ax.plot(f, 20 * np.log10(np.maximum(ma_gain(f, MA_WINDOW, FS), 1e-4)), color=BLUE, lw=1.6)
    decorate(ax, "Ковзне середнє, 4 відліки", "Частота, Гц")
    ax.set_ylabel("Підсилення, дБ")
    ax.set_xlim(0, 100)
    ax.annotate("нуль на 50 Гц", xy=(50, -58), xytext=(57, -30), color=INK2, fontsize=8,
                arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.6))

    ax = axes[1]
    _, h = signal.sosfreqz(np.array([SD_HP["b"] + SD_HP["a"]]), worN=f, fs=FS)
    ax.plot(f, 20 * np.log10(np.maximum(np.abs(h), 1e-4)), color=BLUE, lw=1.6)
    decorate(ax, "ФВЧ Баттерворта, 10 Гц", "Частота, Гц")
    ax.set_xscale("log")
    ax.set_xlim(0.5, 100)
    ax.xaxis.set_major_formatter(FuncFormatter(comma))
    ax.axvline(10, color=MUTED, lw=0.8, ls=(0, (4, 3)))
    ax.text(11.5, -52, "10 Гц", color=INK2, fontsize=8)

    ax = axes[2]
    f2 = np.logspace(math.log10(0.05), math.log10(25), 4000)
    _, h = signal.sosfreqz(STAB_BP, worN=f2, fs=FS_FRAME)
    ax.plot(f2, 20 * np.log10(np.maximum(np.abs(h), 1e-4)), color=BLUE, lw=1.6)
    decorate(ax, "Смуговий фільтр, 0,5–3 Гц", "Частота, Гц")
    ax.set_xscale("log")
    ax.set_xlim(0.05, 25)
    ax.xaxis.set_major_formatter(FuncFormatter(comma))
    for edge in (0.5, 3.0):
        ax.axvline(edge, color=MUTED, lw=0.8, ls=(0, (4, 3)))
    ax.text(0.56, -52, "0,5", color=INK2, fontsize=8)
    ax.text(3.4, -52, "3 Гц", color=INK2, fontsize=8)

    for ax in axes[1:]:
        ax.tick_params(labelleft=False)
    save(fig, "filter_responses.png")


# ---- 2. control example -------------------------------------------------


def load_signal():
    rows = list(csv.DictReader((EXAMPLE / "control_example_signal.csv").open(encoding="utf-8")))
    t = np.array([float(r["t_s"]) for r in rows])
    gyro = np.array([[float(r[f"gyro_{a}"]) for a in "xyz"] for r in rows])
    accel = np.array([[float(r[f"accel_{a}"]) for a in "xyz"] for r in rows])
    return t, gyro, accel


def load_csv(name: str) -> list[dict]:
    return list(csv.DictReader((EXAMPLE / name).open(encoding="utf-8")))


def running_rms(x_sq: np.ndarray, win: int) -> np.ndarray:
    """sqrt of the mean of the last `win` values; shorter mean while filling."""
    c = np.concatenate(([0.0], np.cumsum(x_sq)))
    n = np.arange(1, len(x_sq) + 1)
    lo = np.maximum(n - win, 0)
    return np.sqrt(np.maximum((c[n] - c[lo]) / (n - lo), 0.0))


def envelopes(gyro: np.ndarray, accel: np.ndarray):
    """Accel and gyro envelopes recomputed with a fresh SciPy filter design."""
    b, a = signal.butter(2, 10, "highpass", fs=FS)
    a_mag = np.linalg.norm(accel, axis=1)
    hp = signal.lfilter(b, a, a_mag)
    e = running_rms(hp * hp, SD_RMS_WIN)
    g = running_rms(np.sum(gyro * gyro, axis=1), SD_RMS_WIN)
    return a_mag, e, g


def envelope_single_precision(a_mag: np.ndarray) -> np.ndarray:
    """The accel envelope computed exactly as shot_detect.c does it: the
    firmware coefficients, 32-bit floats and a running sum over the window."""
    f = np.float32
    b0, b1, b2 = (f(v) for v in SD_HP["b"])
    a1, a2 = f(SD_HP["a"][1]), f(SD_HP["a"][2])
    w0 = w1 = total = f(0.0)
    ring = np.zeros(SD_RMS_WIN, dtype=np.float32)
    out = np.empty(len(a_mag))
    for i, mag in enumerate(a_mag.astype(np.float32)):
        d0 = f(mag - a1 * w0 - a2 * w1)
        hp = f(b0 * d0 + b1 * w0 + b2 * w1)
        w1, w0 = w0, d0
        sq = f(hp * hp)
        slot = i % SD_RMS_WIN
        total = f(total + (sq - ring[slot]))
        ring[slot] = sq
        out[i] = math.sqrt(max(float(total) / min(i + 1, SD_RMS_WIN), 0.0))
    return out


def parse_journal():
    """1 Hz diagnostic lines and trigger lines of the detector journal."""
    diag, triggers = [], []
    n = 0
    pat = re.compile(r"\[(\w+)\] E=([\d.]+) base=([\d.]+) \| G=([\d.]+) gbase=([\d.]+)")
    trig = re.compile(r"trigger: onset=(\d+) ms \(E=([\d.]+)/([\d.]+) G=([\d.]+)/([\d.]+)\)")
    for line in (EXAMPLE / "control_example_log.txt").read_text(encoding="utf-8").splitlines():
        if "(shot_detect)" not in line:
            continue
        m = pat.search(line)
        if m:
            n += 1
            diag.append(
                dict(index=n * FS - 1, state=m[1], e=float(m[2]), base=float(m[3]), g=float(m[4]), gbase=float(m[5]))
            )
            continue
        m = trig.search(line)
        if m:
            triggers.append(dict(onset_ms=int(m[1]), e=float(m[2]), th=float(m[3]), g=float(m[4]), gth=float(m[5])))
    return diag, triggers


def check_envelopes(t, a_mag, e, g, diag, triggers) -> None:
    e32 = envelope_single_precision(a_mag)
    lines = ["Envelopes recomputed with SciPy vs. the detector journal (1 Hz diagnostic lines)"]
    lines.append("  t, s    state        E journal  E scipy   E float32  G journal  G scipy")
    worst_e = worst_e32 = worst_g = 0.0
    worst_before = 0.0  # before the first release, i.e. before any strong transient
    for d in diag:
        i = d["index"]
        worst_e = max(worst_e, abs(e[i] - d["e"]))
        worst_e32 = max(worst_e32, abs(e32[i] - d["e"]))
        worst_g = max(worst_g, abs(g[i] - d["g"]))
        if t[i] < 5.0:
            worst_before = max(worst_before, abs(e[i] - d["e"]))
        lines.append(
            f"  {t[i]:6.3f}  {d['state']:<11}  {d['e']:8.3f}  {e[i]:8.4f}  {e32[i]:8.4f}   {d['g']:8.3f}  {g[i]:8.4f}"
        )
    lines.append(f"  largest difference, double precision: E {worst_e:.4f} m/s^2, G {worst_g:.4f} rad/s")
    lines.append(f"    of that before the first release: E {worst_before:.4f} m/s^2")
    lines.append(f"  largest difference, firmware arithmetic (float32 running sum): E {worst_e32:.4f} m/s^2")
    lines.append("  (the journal prints three decimals, so 0.0005 is its rounding error)")

    # Onset of release 1 by hand: the envelope crosses the threshold between
    # samples k-1 and k; interpolate and subtract the group-delay allowance.
    lines.append("")
    lines.append("Onset of release 1 recomputed from the interpolation formula")
    th = triggers[0]["th"]
    k = int(np.argmax((e[1:] > th) & (e[:-1] <= th) & (t[1:] > 4.0))) + 1
    frac = (th - e[k - 1]) / (e[k] - e[k - 1])
    t_prev_us = BOOT_MS * 1000 + (k - 1) * 5000
    onset_us = t_prev_us + frac * 5000
    onset_ms = int((onset_us - SD_ENVELOPE_DELAY_US) // 1000)
    lines.append(f"  crossing sample k = {k} (t = {t[k]:.3f} s), threshold T = {th:.2f} m/s^2")
    lines.append(f"  E[k-1] = {e[k - 1]:.4f}, E[k] = {e[k]:.4f} m/s^2")
    lines.append(f"  f = (T - E[k-1]) / (E[k] - E[k-1]) = {frac:.4f}")
    lines.append(f"  t[k-1] = {t_prev_us} us; onset = t[k-1] + f * 5000 = {onset_us:.0f} us")
    lines.append(f"  reported = (onset - 12000) / 1000 = {onset_ms} ms; detector journal: {triggers[0]['onset_ms']} ms")
    lines.append(f"  envelope at the trigger (sample k+2): scipy {e[k + 2]:.2f}, journal {triggers[0]['e']:.2f} m/s^2")
    write_lines("envelope_check.txt", lines)


def thresholds(t, diag):
    """Trigger thresholds as step functions of the journal baselines."""
    th = np.full_like(t, np.nan)
    gth = np.full_like(t, np.nan)
    for d, nxt in zip(diag, diag[1:] + [dict(index=len(t))]):
        sl = slice(d["index"], nxt["index"])
        th[sl] = max(SD_TRIG_FLOOR, SD_TRIG_K * d["base"])
        gth[sl] = max(SD_GYRO_TRIG_FLOOR, SD_GYRO_TRIG_K * d["gbase"])
    return th, gth


STIMULI_UA = {"release 1": "випуск 1", "knock": "удар", "release 2": "випуск 2", "let-down": "опускання"}
PHASES = [  # (from, to, label) - the scenario of control_example.c
    (0.0, 2.0, "натяг"),
    (2.0, 5.0, "утримання"),
    (7.0, 9.0, "натяг"),
    (9.0, 14.0, "утримання"),
    (16.0, 18.0, "натяг"),
    (18.0, 21.0, "нестійке\nутримання"),
]
MARKERS = {"PROVISIONAL": ("^", "попередня подія"), "CONFIRMED": ("o", "підтверджено"), "REJECTED": ("X", "відхилено")}


def save(fig, name: str) -> None:
    OUT_FIG.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT_FIG / name)
    plt.close(fig)
    print(f"-- figures/{name}")


def figure_control_example(t, a_mag, e, g, diag) -> None:
    events = load_csv("control_example_events.csv")
    summary = load_csv("control_example_summary.csv")
    stab = load_csv("control_example_stability.csv")
    th, gth = thresholds(t, diag)

    fig, axes = plt.subplots(
        4, 1, figsize=(WIDTH_IN, 7.6), sharex=True, constrained_layout=True,
        gridspec_kw=dict(height_ratios=[1.0, 1.15, 1.15, 1.0]),
    )
    ax_a, ax_e, ax_g, ax_s = axes

    # (a) raw acceleration magnitude
    ax_a.plot(t, a_mag, color=BLUE, lw=0.9)
    ax_a.set_title("а) модуль лінійного прискорення, м/с²", loc="left")
    ax_a.set_ylim(0, 95)
    for lo, hi, label in PHASES:
        ax_a.axvspan(lo, hi, ymin=0.0, ymax=1.0, color="#f0efec" if "утрим" in label else "white", lw=0)
        ax_a.text((lo + hi) / 2, 90, label, ha="center", va="top", fontsize=7.5, color=INK2, linespacing=0.9)
    for row in summary:
        ts = float(row["t_s"])
        ax_a.annotate(
            STIMULI_UA[row["stimulus"]], xy=(ts, 60 if "release" in row["stimulus"] else 12),
            xytext=(ts + 0.25, 62 if "release" in row["stimulus"] else 30), fontsize=8, color=INK,
            arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.6),
        )

    # (b), (c) envelopes against their thresholds, log scale
    for ax, env, thr, title, unit, floor in (
        (ax_e, e, th, "б) обвідна каналу прискорення E, м/с²", "E", 0.01),
        (ax_g, g, gth, "в) обвідна каналу гіроскопа G, рад/с", "G", 0.005),
    ):
        ax.plot(t, np.maximum(env, floor), color=BLUE, lw=0.9, label=f"обвідна {unit}")
        ax.plot(t, thr, color=ORANGE, lw=1.4, ls=(0, (5, 2)), label="поріг спрацювання")
        ax.set_yscale("log")
        ax.yaxis.set_major_formatter(FuncFormatter(comma))
        ax.set_title(title, loc="left")
    ax_e.set_ylim(0.01, 200)
    ax_g.set_ylim(0.005, 30)

    # events on the channel that triggered: accel for the first three stimuli
    for ev in events:
        marker, _ = MARKERS[ev["type"]]
        ts = float(ev["t_tick_s"])
        on_gyro = ts > 20.0
        ax = ax_g if on_gyro else ax_e
        # the verdict follows 0.2 s after the provisional event: stack the pair
        upper = ev["type"] != "PROVISIONAL"
        y = (16 if upper else 6) if on_gyro else (110 if upper else 45)
        ax.plot(ts, y, marker=marker, ms=6.5, color=INK, mec="white", mew=0.8, ls="none", zorder=5)
    handles, labels = ax_e.get_legend_handles_labels()
    for marker, label in MARKERS.values():
        handles.append(plt.Line2D([], [], marker=marker, ms=6.5, color=INK, mec="white", mew=0.8, ls="none"))
        labels.append(label)
    fig.legend(handles, labels, loc="outside upper center", ncol=5, columnspacing=1.4, handletextpad=0.5)

    # (d) stability score
    ts = np.array([float(r["t_s"]) for r in stab])
    score = np.array([float(r["score"]) for r in stab])
    ax_s.plot(ts, score, color=BLUE, lw=1.2)
    ax_s.set_title("г) оцінка стабільності утримання, бали", loc="left")
    ax_s.set_ylim(0, 105)
    ax_s.set_yticks([0, 25, 50, 75, 100])
    for row in summary:
        t0 = float(row["t_s"])
        ax_s.text(t0 - 0.5, float(row["hold_score"]) + 5, row["hold_score"].replace(".", ","), ha="center",
                  va="bottom", fontsize=8, color=INK)
    ax_s.set_xlabel("Час від початку фрагмента, с")
    ax_s.set_xlim(0, 24)
    ax_s.set_xticks(range(0, 25, 2))
    save(fig, "control_example.png")


def figure_release_zoom(t, e, diag) -> None:
    """The 0.4 s around release 1: threshold crossing and ring-down window."""
    events = load_csv("control_example_events.csv")
    t_rel = 5.0
    k = int(np.argmax((e[1:] > SD_TRIG_FLOOR) & (e[:-1] <= SD_TRIG_FLOOR) & (t[1:] > 4.0))) + 1
    base = next(d["base"] for d in diag if d["index"] > k)  # baseline during validation
    sl = slice(k - 16, k + 60)
    ms = (t[sl] - t_rel) * 1000.0

    fig, ax = plt.subplots(figsize=(WIDTH_IN, 3.0), constrained_layout=True)
    ax.axvspan((t[k] - t_rel) * 1000 + SD_VALID_DELAY * 5, (t[k] - t_rel) * 1000 + SD_VALID_END * 5,
               color="#f0efec", lw=0)
    ax.text((t[k] - t_rel) * 1000 + 115, 0.0135, "вікно перевірки, 30–200 мс після перетину порога",
            ha="center", fontsize=8, color=INK2)
    ax.plot(ms, np.maximum(e[sl], 0.01), color=BLUE, lw=1.6, marker="o", ms=2.6)
    # T - trigger threshold, B - baseline; 3B marks ring-down activity and
    # 25B a strong peak (explained in the figure caption of the report)
    for level, label in (
        (SD_TRIG_FLOOR, f"T = {SD_TRIG_FLOOR:.1f}".replace(".", ",")),
        (SD_STRONG_K * base, f"25·B = {SD_STRONG_K * base:.2f}".replace(".", ",")),
        (SD_CONF_K * base, f"3·B = {SD_CONF_K * base:.2f}".replace(".", ",")),
    ):
        ax.axhline(level, color=ORANGE, lw=1.1, ls=(0, (5, 2)))
        ax.text(-77, level * 1.1, label, ha="left", va="bottom", fontsize=8, color=INK2)
    for ev, label in zip(events[:2], ("попередня подія", "рішення: підтверджено")):
        x = (float(ev["t_tick_s"]) - t_rel) * 1000
        ax.axvline(x, color=INK, lw=0.9)
        ax.text(x + 3, 68, f"{label},\n+{x:.0f} мс", fontsize=8, color=INK, va="top", linespacing=1.15)
    ax.set_yscale("log")
    ax.yaxis.set_major_formatter(FuncFormatter(comma))
    ax.xaxis.set_major_formatter(FuncFormatter(comma))
    ax.set_ylim(0.01, 80)
    ax.set_xlim(-80, 300)
    ax.set_xlabel("Час від моменту випуску, мс")
    ax.set_title("Обвідна каналу прискорення E, м/с²", loc="left")
    save(fig, "release_zoom.png")


def main() -> None:
    check_filters()
    figure_filters()
    t, gyro, accel = load_signal()
    a_mag, e, g = envelopes(gyro, accel)
    diag, triggers = parse_journal()
    check_envelopes(t, a_mag, e, g, diag, triggers)
    figure_control_example(t, a_mag, e, g, diag)
    figure_release_zoom(t, e, diag)


if __name__ == "__main__":
    main()
