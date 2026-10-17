#!/usr/bin/env python3
# Plot or compare STEPPER_RIPPLE_VIBRATION_MEASURE tables
#
# Copyright (C) 2026  Maksim Bolgov <maksim8024@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import argparse, math, os, re
import matplotlib

# Line colors of the forward and the backward moves of each run
RUN_COLORS = [('tab:orange', 'tab:purple'), ('tab:green', 'tab:cyan')]


######################################################################
# Table parsing
######################################################################

# Values of a column by speed and the motor rpm per mm/s or None
def parse_table(filename, column):
    values = {}
    ratios = []
    header = None
    with open(filename) as f:
        for line in f:
            parts = line.split()
            if not parts:
                continue
            if parts[0] == 'speed' and 'median' in parts:
                header = parts
                if column not in header:
                    raise SystemExit("No column '%s' in %s"
                                     % (column, filename))
                continue
            if header is None or len(parts) != len(header):
                continue
            row = dict(zip(header, parts))
            try:
                speed = float(row['speed'])
                values[speed] = float(row[column])
            except ValueError:
                continue
            if speed and row.get('rpm', '-') != '-':
                ratios.append(float(row['rpm']) / speed)
    if not values:
        raise SystemExit("No table rows found in %s" % (filename,))
    ratio = sum(ratios) / len(ratios) if ratios else None
    return values, ratio

def table_units(filename):
    with open(filename) as f:
        for line in f:
            parts = line.split()
            if parts and parts[0] == 'speed':
                break
            if parts and parts[0] == 'units:' and len(parts) > 1:
                return ' '.join(parts[1:])
    return 'mm/s^2'

def has_column(filename, column):
    with open(filename) as f:
        for line in f:
            parts = line.split()
            if parts and parts[0] == 'speed' and 'median' in parts:
                return column in parts
    return False

# Values of the forward moves and of the backward moves or None
def read_run(filename, column):
    values, ratio = parse_table(filename, column)
    backward = None
    if has_column(filename, column + '_rev'):
        backward = parse_table(filename, column + '_rev')[0]
    return values, backward, ratio


######################################################################
# Plots
######################################################################

def median(vals):
    vals = sorted(vals)
    if not vals:
        return 0.
    n = len(vals)
    return (vals[n // 2] + vals[(n - 1) // 2]) / 2.

def calc_gain(va, vb):
    return [(a - b) / a * 100. if a else 0. for a, b in zip(va, vb)]

def gain_summary(va, vb, gain, names, column, prefix=''):
    rms_a = math.sqrt(sum(v * v for v in va) / len(va))
    rms_b = math.sqrt(sum(v * v for v in vb) / len(vb))
    return ['%smean gain %.1f%%, median %.1f%%, lower vibration at %d of'
            ' %d speeds' % (prefix, sum(gain) / len(gain), median(gain),
                            sum(1 for g in gain if g > 0.), len(gain)),
            '%sRMS of %s: %s %.4g, %s %.4g (%.1f%%)' % (
                prefix, column, names[0], rms_a, names[1], rms_b,
                (rms_a - rms_b) / rms_a * 100. if rms_a else 0.)]

def setup_matplotlib():
    global matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot, matplotlib.font_manager, matplotlib.ticker

def font_small():
    font = matplotlib.font_manager.FontProperties()
    font.set_size('x-small')
    return font

def style_axis(ax):
    ax.xaxis.set_minor_locator(matplotlib.ticker.MultipleLocator(5))
    ax.yaxis.set_minor_locator(matplotlib.ticker.AutoMinorLocator())
    ax.ticklabel_format(axis='y', style='scientific', scilimits=(0,0))
    ax.grid(which='major', color='grey')
    ax.grid(which='minor', color='lightgrey')

def value_label(column, units):
    if column == 'median':
        return 'magnitude, %s' % (units,)
    return '%s magnitude, %s' % (column, units)

def setup_axes(title, column, units, rpm_per_speed):
    fig, ax = matplotlib.pyplot.subplots(figsize=(8, 6),
                                         constrained_layout=True)
    ax.set_title(title)
    ax.set_xlabel('speed, mm/s')
    ax.set_ylabel(value_label(column, units))
    if rpm_per_speed:
        top = ax.secondary_xaxis('top', functions=(
            lambda s: s * rpm_per_speed, lambda r: r / rpm_per_speed))
        top.set_xlabel('motor speed, rpm')
    return fig, ax

def build_compare(runs, names, rpm_per_speed, title, column, units):
    (first, first_rev), (second, second_rev) = runs
    backward = first_rev is not None and second_rev is not None
    speeds = set(first) & set(second)
    if backward:
        speeds &= set(first_rev) & set(second_rev)
    speeds = sorted(speeds)
    if not speeds:
        raise SystemExit("The inputs have no common speeds")
    va = [first[s] for s in speeds]
    vb = [second[s] for s in speeds]
    summary = gain_summary(va, vb, calc_gain(va, vb), names, column,
                           'forward: ' if backward else '')
    fig, ax = setup_axes(title, column, units, rpm_per_speed)
    suffix = ' forward' if backward else ''
    va_rev = vb_rev = None
    if backward:
        va_rev = [first_rev[s] for s in speeds]
        vb_rev = [second_rev[s] for s in speeds]
        summary += gain_summary(va_rev, vb_rev, calc_gain(va_rev, vb_rev),
                                names, column + '_rev', 'backward: ')
    # The legend lists both moves of the first run, then of the second
    for i, vals, vals_rev in ((0, va, va_rev), (1, vb, vb_rev)):
        ax.plot(speeds, vals, label=names[i] + suffix,
                color=RUN_COLORS[i][0])
        if vals_rev is not None:
            ax.plot(speeds, vals_rev, label=names[i] + ' backward',
                    color=RUN_COLORS[i][1])
    style_axis(ax)
    ax.legend(loc='best', prop=font_small())
    return fig, summary

def build_single(values, backward, name, rpm_per_speed, title, column,
                 units):
    speeds = set(values)
    if backward is not None:
        speeds &= set(backward)
    speeds = sorted(speeds)
    if not speeds:
        raise SystemExit("No speeds to plot")
    def stats(vals, label):
        rms = math.sqrt(sum(v * v for v in vals) / len(vals))
        top = max(range(len(vals)), key=lambda i: vals[i])
        return '%s: mean %.4g, median %.4g, RMS %.4g, max %.4g at %g mm/s' % (
            label, sum(vals) / len(vals), median(vals), rms, vals[top],
            speeds[top])
    suffix = ' forward' if backward is not None else ''
    vals = [values[s] for s in speeds]
    summary = [stats(vals, column + suffix)]
    fig, ax = setup_axes(title, column, units, rpm_per_speed)
    ax.plot(speeds, vals, label=name + suffix, color=RUN_COLORS[0][0])
    if backward is not None:
        vals_rev = [backward[s] for s in speeds]
        ax.plot(speeds, vals_rev, label=name + ' backward',
                color=RUN_COLORS[0][1])
        summary.append(stats(vals_rev, column + ' backward'))
    style_axis(ax)
    ax.legend(loc='best', prop=font_small())
    return fig, summary


######################################################################
# Startup
######################################################################

def short_name(filename):
    name = os.path.splitext(os.path.basename(filename))[0]
    m = re.search(r'[0-9]{8}_[0-9]{6}.*', name)
    return m.group(0) if m else name

def main():
    parser = argparse.ArgumentParser(
        description="Plot a STEPPER_RIPPLE_VIBRATION_MEASURE table"
        " stepper_ripple_vibration_*.txt, or compare two of them.")
    parser.add_argument('tables', nargs='+', help="the table, or the"
                        " reference table and the compared table")
    parser.add_argument('-o', '--output', help="plot filename, .png or"
                        " another format supported by matplotlib")
    parser.add_argument('-c', '--column', default='median',
                        help="table column to plot: median, the default, or"
                        " h1..h4 - the backward moves are shown too")
    args = parser.parse_args()
    files = args.tables
    if len(files) > 2:
        parser.error("at most two tables can be compared")
    runs = [read_run(f, args.column) for f in files]
    rpm_per_speed = next(
        (ratio for values, backward, ratio in runs if ratio), None)
    names = [short_name(f) for f in files]
    units = table_units(files[0])
    setup_matplotlib()
    if len(files) == 1:
        values, backward, ratio = runs[0]
        output = args.output or '%s_%s.png' % (
            os.path.splitext(files[0])[0], args.column)
        fig, summary = build_single(
            values, backward, names[0], rpm_per_speed,
            os.path.basename(files[0]), args.column, units)
    else:
        output = args.output or '%s_vs_%s_%s.png' % (
            os.path.splitext(files[1])[0], short_name(files[0]),
            args.column)
        title = '%s\n%s' % (os.path.basename(files[0]),
                            os.path.basename(files[1]))
        fig, summary = build_compare(
            [(values, backward) for values, backward, ratio in runs], names,
            rpm_per_speed, title, args.column, units)
    fig.savefig(output, dpi=150)
    if rpm_per_speed:
        print("rpm per mm/s: %.4f" % (rpm_per_speed,))
    print("Plot: %s" % (output,))
    print('\n'.join(summary))

if __name__ == '__main__':
    main()
