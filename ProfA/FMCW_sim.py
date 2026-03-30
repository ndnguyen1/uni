"""
Synthetic FMCW Radar Simulation
================================
Simulates a 77 GHz FMCW radar with a 4 GHz sweep bandwidth.
Generates beat signals for multiple targets, applies windowing,
and extracts range via FFT.

Parameters are set to match typical TI IWR1443 / AWR1843 front-end specs.
"""

import numpy as np
import matplotlib.pyplot as plt

# --- Radar Parameters --------------------------------------------------------

fc = 77e9          # Centre frequency (Hz)
B  = 4e9           # Sweep bandwidth (Hz)
T  = 40e-6         # Chirp duration (s)
fs = 10e6          # ADC sample rate (Hz)
c  = 3e8           # Speed of light (m/s)

N  = int(fs * T)   # Samples per chirp

# Derived
S         = B / T                     # Chirp slope (Hz/s)
f_res     = 1 / T                     # Frequency resolution (Hz)
range_res = c / (2 * B)               # Range resolution (m)
range_max = (fs / 2) * c / (2 * S)   # Unambiguous range (m)

print(f"Range resolution      : {range_res*100:.2f} cm")
print(f"Max unambiguous range : {range_max:.2f} m")
print(f"Samples per chirp     : {N}")

# --- Targets -----------------------------------------------------------------

# Each target: (range_m, rcs_linear)
# RCS controls the amplitude of the reflected signal
targets = [
    (1.0, 1.0),   # Strong target at 1.0 m
    (2.5, 0.4),   # Weaker target at 2.5 m
]

# --- Signal Generation -------------------------------------------------------

t = np.arange(N) / fs   # Time axis for one chirp

beat = np.zeros(N, dtype=complex)

for R, rcs in targets:
    tau   = 2 * R / c              # Round-trip time of flight
    f_b   = S * tau                # Beat frequency for this target
    phi   = 2 * np.pi * fc * tau   # Phase shift from propagation

    # Complex beat signal (I+jQ).
    # In practice this comes from mixing TX with RX through an IQ demodulator.
    beat += rcs * np.exp(1j * (2 * np.pi * f_b * t - phi))

# --- Noise -------------------------------------------------------------------

SNR_dB    = 20
SNR_lin   = 10 ** (SNR_dB / 10)
sig_power = np.mean(np.abs(beat) ** 2)
noise_std = np.sqrt(sig_power / (2 * SNR_lin))   # per I and Q component

noise       = noise_std * (np.random.randn(N) + 1j * np.random.randn(N))
beat_noisy  = beat + noise

# --- FFT Processing ----------------------------------------------------------

window = np.hanning(N)

fft_rect = np.fft.fft(beat_noisy) / N                        # No windowing
fft_hann = np.fft.fft(beat_noisy * window) / np.sum(window)  # Hann window

# Frequency and range axes (one-sided)
freqs      = np.fft.fftfreq(N, d=1/fs)[:N//2]
range_axis = freqs * c / (2 * S)

mag_rect = 20 * np.log10(np.abs(fft_rect[:N//2]) + 1e-12)
mag_hann = 20 * np.log10(np.abs(fft_hann[:N//2]) + 1e-12)

# --- Theoretical beat frequencies --------------------------------------------

print("\nTheoretical beat frequencies:")
for R, _ in targets:
    f_b = S * (2 * R / c)
    print(f"  R = {R:.1f} m  ->  f_beat = {f_b/1e3:.2f} kHz  (bin {f_b/f_res:.1f})")

# --- Plot --------------------------------------------------------------------

fig, axes = plt.subplots(2, 1, figsize=(10, 8))
fig.suptitle("Synthetic FMCW Range FFT  |  77 GHz, 4 GHz BW, SNR = 20 dB",
             fontsize=13)

configs = [
    (mag_rect, "Rectangular window (no windowing)", "steelblue"),
    (mag_hann, "Hann window",                       "darkorange"),
]

for ax, (mag, label, color) in zip(axes, configs):
    ax.plot(range_axis, mag, color=color, linewidth=0.9)
    for R, _ in targets:
        ax.axvline(R, color="red", linestyle="--", linewidth=0.8, alpha=0.6,
                   label=f"Target at {R} m")
    ax.set_xlim(0, range_max * 0.4)
    ax.set_ylim(bottom=-60)
    ax.set_xlabel("Range (m)")
    ax.set_ylabel("Magnitude (dBFS)")
    ax.set_title(label)
    ax.grid(True, alpha=0.3)
    for R, _ in targets:
        ax.annotate(f"{R} m", xy=(R, mag.max() - 5),
                    ha="center", fontsize=8, color="red")

plt.tight_layout()
plt.savefig("fmcw_range_fft.png", dpi=150, bbox_inches="tight")
plt.show()
print("\nPlot saved: fmcw_range_fft.png")