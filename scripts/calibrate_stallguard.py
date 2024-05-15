#!/usr/bin/env python3
import os
import sys
import csv
import argparse
import statistics
import matplotlib.pyplot as plt

def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(
        description='Generate stallguard graph on CSV data')
    parser.add_argument("csv_path",
                        help="filename of output csv file")
    parser.add_argument("-o", '--output', dest="output",
                        help="filename of output graph")
    parser.add_argument('--over-time', dest="over_time",
                        action='store_true',
                        help='Create overtime graph instead of cummulative')
    parser.add_argument(
        "-s",
        "--min-speed",
        dest="min_speed",
        default=0.0,
        type=float,
        help="minimum speed to plot",
    )
    parser.add_argument(
        "-e",
        "--max-speed",
        dest="max_speed",
        default=4000.0,
        type=float,
        help="maximum speed to plot",
    )
    args = parser.parse_args()
    if len(sys.argv)==1 or args.output is None or args.csv_path is None:
        parser.print_help()
        exit(1)

    file = args.csv_path

    data = {}
    raw_data = []
    source_file_name = os.path.basename(file)

    with open(file, 'r') as csvfile:
        csv_reader = csv.reader(csvfile)
        header = next(csv_reader)  # Skip the header row
        if header != ['#time', 'velocity', 'sg_result', 'cs_actual']:
            print("Header is not match with expected")
            exit(1)

        for row in csv_reader:
            eventtime = float(row[0])
            velocity = float(row[1])
            sg_result = int(row[2])
            cs_actual = int(row[3])
            if velocity < args.min_speed or velocity > args.max_speed:
                continue
            raw_data.append([eventtime, velocity, sg_result, cs_actual])
            if velocity not in data:
                data[velocity] = {
                    'sg_min': sg_result,
                    'sg_list': [sg_result],
                    'sg_max': sg_result,
                    'cs_min': cs_actual,
                    'cs_list': [cs_actual],
                    'cs_max': cs_actual
                }
                continue

            data[velocity]["sg_min"] = min(data[velocity]["sg_min"], sg_result)
            data[velocity]["sg_max"] = max(data[velocity]["sg_max"], sg_result)
            data[velocity]["sg_list"].append(sg_result)

            data[velocity]["cs_min"] = min(data[velocity]["cs_min"], cs_actual)
            data[velocity]["cs_max"] = max(data[velocity]["cs_max"], cs_actual)
            data[velocity]["cs_list"].append(cs_actual)

    for velocity in data:
        mean = statistics.mean(data[velocity]['sg_list'])
        data[velocity]['sg_mean'] = mean

        mean = statistics.mean(data[velocity]['cs_list'])
        data[velocity]['cs_mean'] = mean

    velocities = list(data.keys())
    velocities.sort()

    plt.figure(figsize=[16, 8])
    if not args.over_time:
        sg_min_values = [data[velocity]['sg_min'] for velocity in velocities]
        sg_mean_values = [data[velocity]['sg_mean'] for velocity in velocities]
        sg_max_values = [data[velocity]['sg_max'] for velocity in velocities]
        plt.plot(velocities, sg_min_values, label='Min SG')
        plt.plot(velocities, sg_mean_values, label='Mean SG')
        plt.plot(velocities, sg_max_values, label='Max SG')

        cs_min_values = [data[velocity]['cs_min'] for velocity in velocities]
        cs_mean_values = [data[velocity]['cs_mean'] for velocity in velocities]
        cs_max_values = [data[velocity]['cs_max'] for velocity in velocities]
        plt.plot(velocities, cs_min_values, label='Min CS')
        plt.plot(velocities, cs_mean_values, label='Mean CS')
        plt.plot(velocities, cs_max_values, label='Max CS')

        # Add labels and title
        plt.xlabel('Velocity')
        plt.ylabel('result')
        plt.title("sg_result/cs_actual by Velocity\n" + source_file_name)
        plt.legend()

        plt.savefig(args.output)
    else:
        eventtimes = [event[0] for event in raw_data]
        velocities = [event[1] for event in raw_data]
        sg_results = [event[2] for event in raw_data]
        cs_actual = [event[3] for event in raw_data]

        plt.plot(eventtimes, velocities, label="Velocity mm/s")
        plt.plot(eventtimes, sg_results, label="SG")
        plt.plot(eventtimes, cs_actual, label="CS")
        plt.xlabel('Time')
        plt.ylabel('Values')
        plt.title("Velocity/sg_result/cs_actual by time\n" + source_file_name)
        plt.legend()

        plt.savefig(args.output,)

    # Show the plot
    plt.show()

if __name__ == '__main__':
    main()
