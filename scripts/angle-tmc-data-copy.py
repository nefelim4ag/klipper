#!/usr/bin/python3

import sys
import json
import matplotlib.pyplot as plt
import numpy as np

data_file = sys.argv[1]
conversion_factor = 360 / (1 << 16)
x_values = []

angles = {}
mscnt_min = 0
with open(data_file, 'r') as file:
    for line in file:
        doc = json.loads(line.strip())
        mscnt = list(doc.keys())[0]
        angles[int(mscnt)] = doc[mscnt]
        mscnt_min = min(int(mscnt), mscnt_min)
        x_values.append(int(mscnt))

# fs_angle = 360 / 200
# act_angle = (angles[0]["avg"] - angles[255]["avg"]) * 360 / (1 << 16)
# print("Full Step 0 Angle expected: %.3f, actual: %.3f" % (fs_angle, act_angle))
# # if act_angle < (fs_angle - fs_angle / 100):
# #     for mscnt in angles:
# #         act_angle = (angles[0]["avg"] - angles[mscnt]["avg"]) * 360 / (1 << 16)
# #         if act_angle > (fs_angle - fs_angle / 100):
# #             print("Fullstep 0 at mscnt: %i" % mscnt)
# #             break

# act_angle = (angles[256]["angle_avg"] - angles[511]["angle_avg"])
# print("Full Step 1 Angle expected: %.3f, actual: %.3f" % (fs_angle, act_angle))
# act_angle = (angles[512]["angle_avg"] - angles[767]["angle_avg"])
# print("Full Step 2 Angle expected: %.3f, actual: %.3f" % (fs_angle, act_angle))
# act_angle = (angles[768]["angle_avg"] - angles[1023]["angle_avg"])
# print("Full Step 3 Angle expected: %.3f, actual: %.3f" % (fs_angle, act_angle))


# convert everything to angle diffs
x_values_new = []
means_raw = []
for i in range(1, len(x_values)):
    mscnt_prev = x_values[i-1]
    mscnt = x_values[i]
    x_values_new.append((mscnt_prev + mscnt)/2)
    means_raw.append(1000 * -(angles[mscnt_prev]["angle_avg"] - angles[mscnt]["angle_avg"]) * (40 / 360))

stddev = np.std(means_raw)
print(stddev)

# means_64 = [None, None]
# prev_i = 0
# for i in range(2, 1022):
#     travel = 1000 * (angles[prev_i]["angle_avg"] - angles[i]["angle_avg"]) * (40 / 360)
#     means_64.append(travel)
#     prev_i += 1
# means_64.append(None)
# means_64.append(None)

# means_32 = [None, None, None, None]
# prev_i = 0
# for i in range(4, 1020):
#     travel = 1000 * (angles[prev_i]["angle_avg"] - angles[i]["angle_avg"]) * (40 / 360)
#     means_32.append(travel)
#     prev_i += 1
# means_32.append(None)
# means_32.append(None)
# means_32.append(None)
# means_32.append(None)

# means_16 = [None, None, None, None, None, None, None, None]
# prev_i = 0
# for i in range(8, 1016):
#     travel = 1000 * (angles[prev_i]["angle_avg"] - angles[i]["angle_avg"]) * (40 / 360)
#     means_16.append(travel)
#     prev_i += 1
# means_16.append(None)
# means_16.append(None)
# means_16.append(None)
# means_16.append(None)
# means_16.append(None)
# means_16.append(None)
# means_16.append(None)
# means_16.append(None)

x_values = x_values_new

# max_stddev = max(stddevs)
# print(f'Maximum Standard Deviation across all (in degrees): {max_stddev}')
# print(f"Diff Means stddev: {p.std(means)}")

plt.figure(figsize=(10, 6))
plt.plot(x_values, means_raw, label=f'Travel raw, std: {stddev:.2f}', color='blue', marker='o')
# plt.plot(x_values, means_64, label='Mean travel 64 steps')
# plt.plot(x_values, means_32, label='Mean travel 32 steps')
# plt.plot(x_values, means_16, label='Mean travel 16 steps')
# plt.plot(x_values, maxs, label='Max travel', color='red', marker='^')

plt.xlabel('MSCNT')
plt.ylabel('Travel (um)')
plt.title(f'Microstep distance: {data_file}')
plt.legend()

# Show the plot
plt.show()
