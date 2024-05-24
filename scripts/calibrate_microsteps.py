#!/usr/bin/env python3
# Based on klippain-shaketune belt tool
# Origin by Frix_x#0161 #
# Thank you

import os
import sys
import importlib
import argparse
import traceback
import matplotlib
import matplotlib.colors
import matplotlib.font_manager
import matplotlib.pyplot as plt
import matplotlib.ticker
import numpy as np

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                             '..', 'klippy'))
shaper_calibrate = importlib.import_module('.shaper_calibrate', 'extras')

from collections import namedtuple

matplotlib.use('Agg')

def parse_log(logname):
    with open(logname) as f:
        for header in f:
            if not header.startswith('#'):
                break
        if not header.startswith('freq,psd_x,psd_y,psd_z,psd_xyz'):
            # Raw accelerometer data
            return np.loadtxt(logname, comments='#', delimiter=',')
    # Power spectral density data or shaper calibration data
    raise ValueError(
        'File %s does not contain raw accelerometer data and therefore '
        'is not supported by Shake&Tune. Please use the official Klipper '
        'script to process it instead.' % (logname,)
    )

# This find all the peaks in a curve by looking at when the derivative term goes from positive to negative
# Then only the peaks found above a threshold are kept to avoid capturing peaks in the low amplitude noise of a signal
def detect_peaks(data, indices, detection_threshold, relative_height_threshold=None, window_size=5, vicinity=3):
    # Smooth the curve using a moving average to avoid catching peaks everywhere in noisy signals
    kernel = np.ones(window_size) / window_size
    smoothed_data = np.convolve(data, kernel, mode='valid')
    mean_pad = [np.mean(data[:window_size])] * (window_size // 2)
    smoothed_data = np.concatenate((mean_pad, smoothed_data))

    # Find peaks on the smoothed curve
    smoothed_peaks = (
        np.where((smoothed_data[:-2] < smoothed_data[1:-1]) & (smoothed_data[1:-1] > smoothed_data[2:]))[0] + 1
    )
    smoothed_peaks = smoothed_peaks[smoothed_data[smoothed_peaks] > detection_threshold]

    # Additional validation for peaks based on relative height
    valid_peaks = smoothed_peaks
    if relative_height_threshold is not None:
        valid_peaks = []
        for peak in smoothed_peaks:
            peak_height = smoothed_data[peak] - np.min(
                smoothed_data[max(0, peak - vicinity) : min(len(smoothed_data), peak + vicinity + 1)]
            )
            if peak_height > relative_height_threshold * smoothed_data[peak]:
                valid_peaks.append(peak)

    # Refine peak positions on the original curve
    refined_peaks = []
    for peak in valid_peaks:
        local_max = peak + np.argmax(data[max(0, peak - vicinity) : min(len(data), peak + vicinity + 1)]) - vicinity
        refined_peaks.append(local_max)

    num_peaks = len(refined_peaks)

    return num_peaks, np.array(refined_peaks), indices[refined_peaks]


# Calculate or estimate a "similarity" factor between two PSD curves and scale it to a percentage. This is
# used here to quantify how close the two belts path behavior and responses are close together.
def compute_curve_similarity_factor(x1, y1, x2, y2, sim_sigmoid_k=0.6):
    # Interpolate PSDs to match the same frequency bins and do a cross-correlation
    y2_interp = np.interp(x1, x2, y2)
    cross_corr = np.correlate(y1, y2_interp, mode='full')

    # Find the peak of the cross-correlation and compute a similarity normalized by the energy of the signals
    peak_value = np.max(cross_corr)
    similarity = peak_value / (np.sqrt(np.sum(y1**2) * np.sum(y2_interp**2)))

    # Apply sigmoid scaling to get better numbers and get a final percentage value
    scaled_similarity = sigmoid_scale(-np.log(1 - similarity), sim_sigmoid_k)

    return scaled_similarity

# Simple helper to compute a sigmoid scalling (from 0 to 100%)
def sigmoid_scale(x, k=1):
    return 1 / (1 + np.exp(-k * x)) * 100

ALPHABET = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ'  # For paired peaks names

PEAKS_DETECTION_THRESHOLD = 0.20
CURVE_SIMILARITY_SIGMOID_K = 0.6
DC_GRAIN_OF_SALT_FACTOR = 0.75
DC_THRESHOLD_METRIC = 1.5e9
DC_MAX_UNPAIRED_PEAKS_ALLOWED = 4

# Define the SignalData namedtuple
SignalData = namedtuple('CalibrationData', ['freqs', 'psd', 'peaks', 'paired_peaks', 'unpaired_peaks'])

######################################################################
# Computation of the PSD graph
######################################################################


# This function create pairs of peaks that are close in frequency on two curves (that are known
# to be resonances points and must be similar on microsteps)
def pair_peaks(signals):
    # Compute a dynamic detection threshold to filter and pair peaks efficiently
    # even if the signal is very noisy (this get clipped to a maximum of 10Hz diff)
    # signals[0].peaks, signals[0].freqs, signals[0].psd, signals[1].peaks, signals[1].freqs, signals[1].psd
    pairs = []
    for num in range(0, len(signals) - 1):
        pairs.append([signals[num], signals[num + 1]])

    paired_peaks = []

    for pair in pairs:
        signal1 = pair[0]
        signal2 = pair[1]
        distances = []
        for p1 in signal1.peaks:
            for p2 in signal2.peaks:
                distances.append(abs(signal1.freqs[p1] - signal2.freqs[p2]))
        distances = np.array(distances)

        median_distance = np.median(distances)
        iqr = np.percentile(distances, 75) - np.percentile(distances, 25)

        threshold = median_distance + 1.5 * iqr
        threshold = min(threshold, 10)

        # Pair the peaks using the dynamic thresold
        unpaired_peaks1 = list(signal1.peaks)
        unpaired_peaks2 = list(signal2.peaks)

        while unpaired_peaks1 and unpaired_peaks2:
            min_distance = threshold + 1
            pair = None

            for p1 in unpaired_peaks1:
                for p2 in unpaired_peaks2:
                    distance = abs(signal1.freqs[p1] - signal2.freqs[p2])
                    if distance < min_distance:
                        min_distance = distance
                        pair = (p1, p2)

            if pair is None:  # No more pairs below the threshold
                break

            p1, p2 = pair
            paired_peaks.append(((p1, signal1.freqs[p1], signal1.psd[p1]), (p2, signal2.freqs[p2], signal2.psd[p2])))
            unpaired_peaks1.remove(p1)
            unpaired_peaks2.remove(p2)

    return paired_peaks

######################################################################
# Graphing
######################################################################


def plot_compare_frequency(ax, lognames, signals, similarity_factor, max_freq):
    for i in range(0, len(signals)):
        mscnt = (lognames[i].split('/')[-1]).split('_')[-1]
        label = 'Microstep ' + mscnt
        ax.plot(signals[i].freqs, signals[i].psd, label=label)

    # Trace the "relax region" (also used as a threshold to filter and detect the peaks)
    psd_lowest_max = min(signal.psd.max() for signal in signals)
    peaks_warning_threshold = PEAKS_DETECTION_THRESHOLD * psd_lowest_max
    ax.axhline(y=peaks_warning_threshold, color='black', linestyle='--', linewidth=0.5)
    ax.fill_between(signals[0].freqs, 0, peaks_warning_threshold, color='green', alpha=0.15, label='Relax Region')

    # Trace and annotate the peaks on the graph
    paired_peak_count = 0
    unpaired_peak_count = 0
    offsets_table_data = []

    # for _, (peak1, peak2) in enumerate(signal1.paired_peaks):
    #     label = ALPHABET[paired_peak_count]
    #     amplitude_offset = abs(
    #         ((signal2.psd[peak2[0]] - signal1.psd[peak1[0]]) / max(signal1.psd[peak1[0]], signal2.psd[peak2[0]])) * 100
    #     )
    #     frequency_offset = abs(signal2.freqs[peak2[0]] - signal1.freqs[peak1[0]])
    #     offsets_table_data.append([f'Peaks {label}', f'{frequency_offset:.1f} Hz', f'{amplitude_offset:.1f} %'])

    #     ax.plot(signal1.freqs[peak1[0]], signal1.psd[peak1[0]], 'x', color='black')
    #     ax.plot(signal2.freqs[peak2[0]], signal2.psd[peak2[0]], 'x', color='black')
    #     ax.plot(
    #         [signal1.freqs[peak1[0]], signal2.freqs[peak2[0]]],
    #         [signal1.psd[peak1[0]], signal2.psd[peak2[0]]],
    #         ':',
    #         color='gray',
    #     )

    #     ax.annotate(
    #         label + '1',
    #         (signal1.freqs[peak1[0]], signal1.psd[peak1[0]]),
    #         textcoords='offset points',
    #         xytext=(8, 5),
    #         ha='left',
    #         fontsize=13,
    #         color='black',
    #     )
    #     ax.annotate(
    #         label + '2',
    #         (signal2.freqs[peak2[0]], signal2.psd[peak2[0]]),
    #         textcoords='offset points',
    #         xytext=(8, 5),
    #         ha='left',
    #         fontsize=13,
    #         color='black',
    #     )
    #     paired_peak_count += 1

    # for peak in signal1.unpaired_peaks:
    #     ax.plot(signal1.freqs[peak], signal1.psd[peak], 'x', color='black')
    #     ax.annotate(
    #         str(unpaired_peak_count + 1),
    #         (signal1.freqs[peak], signal1.psd[peak]),
    #         textcoords='offset points',
    #         xytext=(8, 5),
    #         ha='left',
    #         fontsize=13,
    #         color='red',
    #         weight='bold',
    #     )
    #     unpaired_peak_count += 1

    # for peak in signal2.unpaired_peaks:
    #     ax.plot(signal2.freqs[peak], signal2.psd[peak], 'x', color='black')
    #     ax.annotate(
    #         str(unpaired_peak_count + 1),
    #         (signal2.freqs[peak], signal2.psd[peak]),
    #         textcoords='offset points',
    #         xytext=(8, 5),
    #         ha='left',
    #         fontsize=13,
    #         color='red',
    #         weight='bold',
    #     )
    #     unpaired_peak_count += 1

    # Add estimated similarity to the graph
    ax2 = ax.twinx()  # To split the legends in two box
    ax2.yaxis.set_visible(False)
    ax2.plot([], [], ' ', label=f'Estimated similarity: {similarity_factor:.1f}%')
    ax2.plot([], [], ' ', label=f'Number of unpaired peaks: {unpaired_peak_count}')

    # Setting axis parameters, grid and graph title
    ax.set_xlabel('Frequency (Hz)')
    ax.set_xlim([0, max_freq])
    ax.set_ylabel('Power spectral density')
    psd_highest_max = max(signal.psd.max() for signal in signals)
    ax.set_ylim([0, psd_highest_max * 1.05])

    ax.xaxis.set_minor_locator(matplotlib.ticker.AutoMinorLocator())
    ax.yaxis.set_minor_locator(matplotlib.ticker.AutoMinorLocator())
    ax.ticklabel_format(axis='y', style='scientific', scilimits=(0, 0))
    ax.grid(which='major', color='grey')
    ax.grid(which='minor', color='lightgrey')
    fontP = matplotlib.font_manager.FontProperties()
    fontP.set_size('small')
    ax.set_title(
        'Microstep frequency profiles (estimated similarity: {:.1f}%)'.format(similarity_factor),
        fontsize=14,
        weight='bold',
    )

    # Print the table of offsets ontop of the graph below the original legend (upper right)
    if len(offsets_table_data) > 0:
        columns = [
            '',
            'Frequency delta',
            'Amplitude delta',
        ]
        offset_table = ax.table(
            cellText=offsets_table_data,
            colLabels=columns,
            bbox=[0.66, 0.75, 0.33, 0.15],
            loc='upper right',
            cellLoc='center',
        )
        offset_table.auto_set_font_size(False)
        offset_table.set_fontsize(8)
        offset_table.auto_set_column_width([0, 1, 2])
        offset_table.set_zorder(100)
        cells = [key for key in offset_table.get_celld().keys()]
        for cell in cells:
            offset_table[cell].set_facecolor('white')
            offset_table[cell].set_alpha(0.6)

    ax.legend(loc='upper left', prop=fontP)
    ax2.legend(loc='upper right', prop=fontP)

    return

######################################################################
# Custom tools
######################################################################


# Original Klipper function to get the PSD data of a raw accelerometer signal
def compute_signal_data(data, max_freq):
    helper = shaper_calibrate.ShaperCalibrate(printer=None)
    calibration_data = helper.process_accelerometer_data(data)

    freqs = calibration_data.freq_bins[calibration_data.freq_bins <= max_freq]
    psd = calibration_data.get_psd('all')[calibration_data.freq_bins <= max_freq]

    _, peaks, _ = detect_peaks(psd, freqs, PEAKS_DETECTION_THRESHOLD * psd.max())

    return SignalData(freqs=freqs, psd=psd, peaks=peaks, paired_peaks=None, unpaired_peaks=None)


######################################################################
# Startup and main routines
######################################################################


def microstep_calibration(lognames, max_freq=200.0, st_version=None):
    # Parse data
    datas = [parse_log(fn) for fn in lognames]
    signals = []
    for data in datas:
        signal = compute_signal_data(data, max_freq)
        signals.append(signal)

    # Pair the peaks across the two datasets
    paired_peaks = pair_peaks(signals)
    for i in range(0, len(signals)):
        signals[i] = signals[i]._replace(paired_peaks=paired_peaks)

    # Compute the similarity (using cross-correlation of the PSD signals)
    similarity_factor = compute_curve_similarity_factor(
        signals[0].freqs, signals[0].psd, signals[0].freqs, signals[1].psd, CURVE_SIMILARITY_SIGMOID_K
    )
    print(f'Microsteps estimated similarity: {similarity_factor:.1f}%')

    # Create graph layout
    fig, (ax1) = plt.subplots(
        1,
        gridspec_kw={
            'height_ratios': [4],
            'bottom': 0.050,
            'top': 0.890,
            'left': 0.085,
            'right': 0.966,
            'hspace': 0.169,
            'wspace': 0.200,
        },
    )
    fig.set_size_inches(12, 10)

    # Plot the graphs
    plot_compare_frequency(ax1, lognames, signals, similarity_factor, max_freq)

    return fig

def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Shake&Tune graphs generation script')
    parser.add_argument(
        '-o',
        '--output',
        dest='output',
        type=str,
        help='output file path',
    )
    parser.add_argument('filenames', nargs='+', help='csv file paths')
    return parser.parse_args()

def main():
    options = parse_arguments()

    # Instantiate the graph creator
    fig = microstep_calibration(
        lognames=options.filenames
    )
    fig.savefig(options.output, dpi=300)

if __name__ == '__main__':
    main()
