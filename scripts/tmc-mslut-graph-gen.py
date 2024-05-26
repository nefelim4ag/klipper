#!/usr/bin/env python3
import math
import random
import matplotlib.pyplot as plt

def mslut_encoder(quarter_seg, START_SIN90):
    if len(quarter_seg) != 256:
        print("Wrong quarter segment size")
        return

    deltas = []
    prev_v = 0
    for val in quarter_seg:
        delta = val - prev_v
        if delta > 2 or delta < -1:
            print(f"prev: {prev_v}, val: {val} delta can't be encoded")
            return
        prev_v = val
        deltas.append(delta)

    # Search for segments
    segments = {
        0: {
            "start": 0,
            "min": 2,
            "max": -1,
        },
        1: {
            "start": 255,
            "min": 2,
            "max": -1,
        },
        2: {
            "start": 255,
            "min": 3,
            "max": -1,
        },
        3: {
            "start": 255,
            "min": 3,
            "max": -1,
        },
    }
    cur_seg = 0
    # 0 element is always SIN_START?
    for i in range(1, len(deltas)):
        smin = segments[cur_seg]["min"]
        smax = segments[cur_seg]["max"]
        delta = deltas[i]
        nsmin = min(smin, delta)
        nsmax = max(smax, delta)
        if nsmax - nsmin > 1:
            print(f"chop new segment at {i}")
            cur_seg += 1
            segments[cur_seg]["min"] = delta
            segments[cur_seg]["max"] = delta
            segments[cur_seg]["start"] = i
            continue
        else:
            segments[cur_seg]["min"] = nsmin
            segments[cur_seg]["max"] = nsmax

    W = {}
    W[0] = (segments[0]["min"] + 1)
    W[1] = (segments[1]["min"] + 1) % 4
    W[2] = (segments[2]["min"] + 1) % 4
    W[3] = (segments[3]["min"] + 1) % 4

    X = {}
    X[1] = segments[1]["start"]
    X[2] = segments[2]["start"]
    X[3] = segments[3]["start"]

    bit_diffs = []
    # print(deltas)
    # Width control bit coding W0…W3:
    # %00: MSLUT entry 0, 1 select: -1, +0
    # %01: MSLUT entry 0, 1 select: +0, +1
    # %10: MSLUT entry 0, 1 select: +1, +2
    # %11: MSLUT entry 0, 1 select: +2, +3
    WReverseTable = {
        0: {
           -1: 0,
            0: 1,
        },
        1: {
            0: 0,
            1: 1,
        },
        2: {
            1: 0,
            2: 1,
        },
        3: {
            2: 0,
            3: 1,
        },
    }
    _width = 0
    for pair in [[1, X[1]], [X[1], X[2]], [X[2], X[3]]]:
        width = W[_width]
        for i in range(pair[0], pair[1]):
            delta = deltas[i]
            # print(f"{i}, {width}, {delta} -> {WReverseTable[width][delta]}")
            bit = WReverseTable[width][delta]
            if bit != 0 and bit != 1:
                print("Something very wrong happens here")
                exit(0)
            bit_diffs.append(bit)
        _width += 1

    # print(bit_diffs, len(bit_diffs))
    # Where is 2 last bit missing, possibly becuase ...
    # first is sin_start and last is sin90?
    bit_diffs.append(0)
    bit_diffs.append(0)

    MSLUTS = [
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0
    ]
    for m in range(0, len(MSLUTS)):
        for i in range(0, 32):
            bit = bit_diffs[m * 32 + i]
            MSLUTS[m] = MSLUTS[m] | (bit << i)

    return {
        "START_SIN": quarter_seg[0], # Fisrt value is always START_SIN
        "START_SIN90": START_SIN90,
        "X": X,
        "W": W,
        "MSLUTS": MSLUTS
    }

def mslut_decoder(MSLUTS, W, X, START_SIN, START_SIN90):
    bit_diffs = []
    for mslut in MSLUTS:
        for shift in range(0, 32):
            bit_val = (mslut >> shift) & 1
            bit_diffs.append(bit_val)
    # print(bit_diffs, len(bit_diffs))

    decoded_val = []

    for i in range(0, 256):
        if i < X[1] - 1:
            # Segment 0
            offset = W[0] - 1
        elif i < X[2] - 1:
            # Segment 1
            offset = W[1] - 1
        elif i < X[3] - 1:
            # Segment 2
            offset = W[2] - 1
        else:
            # Segment 3
            offset = W[3] - 1
        bit = bit_diffs[i]
        bit_dec = bit + offset
        decoded_val.append(bit_dec)

    # Sum diffs one by one
    sin_value = [START_SIN] # first value is always START_SIN
    for val in decoded_val:
        last = sin_value[-1]
        sin_value.append(last + val)

    if sin_value[-1] != START_SIN90:
        print("Are you sure? End of 1/4 of sin is not in sync with 2 of 4")
    if sin_value[0] != START_SIN:
        print(f"Are you sure? first value is {sin_value[0]} you miss zero crossing")

    # Maybe i'm stupid, as last value is equal it can be ignored..? 257 -> 256
    sin_value.pop()

    return sin_value


def main():
    # Klipper TMC5160 default
    MSLUTS = [
        2863314260, # 0xAAAAB554
        1251300522, # 0x4A9554AA
        608774441,  # 0x24492929
        269500962,  # 0x10104222
        4227858431, # 0xFBFFFFFF
        3048961917,  # 0xB5BB777D
        1227445590,  # 0x49295556
        4211234,  # 0x00404222
    ]

    # Width control bit coding W0…W3:
    # %00: MSLUT entry 0, 1 select: -1, +0
    # %01: MSLUT entry 0, 1 select: +0, +1
    # %10: MSLUT entry 0, 1 select: +1, +2
    # %11: MSLUT entry 0, 1 select: +2, +3
    W = {
        0: 2, #W0
        1: 1, #W1
        2: 1, #W2
        3: 1  #W3
    }
    # Unused, just for inmind decoding
    # _W_lookup_table = {0: {-1, 0}, 1: {0, +1}, 2: {1, 2}, 3: {2, 3}}
    # _W_Fast_offset_table = {0: -1, 1: 0, 2: 1, 3: 2}
    # offset = W - 1

    # The sine wave look-up table can be divided into up to
    # four segments using an individual step width control
    # entry Wx. The segment borders are selected by X1, X2
    # and X3.
    # Segment 0 goes from 0 to X1-1.
    # Segment 1 goes from X1 to X2-1.
    # Segment 2 goes from X2 to X3-1.
    # Segment 3 goes from X3 to 255.
    # For defined response the values shall satisfy:
    # 0<X1<X2<X3
    X = {
        1: 128, # X1
        2: 255, # X2
        3: 255  # X3
    }
    START_SIN = 0
    START_SIN90 = 247
    # START_SIN90 gives the absolute current for
    # microstep table entry at positions 256

    sin_value = mslut_decoder(MSLUTS, W, X, START_SIN, START_SIN90)
    encoded = mslut_encoder(sin_value, START_SIN90)
    if MSLUTS != encoded["MSLUTS"]:
        print("MSLUT encoder broken")
        print(MSLUTS, encoded["MSLUTS"])
    if START_SIN != encoded["START_SIN"]:
        print("START_SIN encoder is broken")
    if START_SIN90 != encoded["START_SIN90"]:
        print("... ? Encoder broken?")
    if X != encoded["X"]:
        print("X encoder broken")
    if W != encoded["W"]:
        if W[0] != encoded["W"][0]:
            print("W encoder broken", W, encoded["W"])
        if X[1] < 255:
            if W[1] != encoded["W"][1]:
                print("W encoder broken", W, encoded["W"])
        if X[2] < 255:
            if W[2] != encoded["W"][2]:
                print("W encoder broken", W, encoded["W"])
        if X[3] < 255:
            if W[3] != encoded["W"][3]:
                print("W encoder broken", W, encoded["W"])

    positions = []
    sin_value_90 = sin_value.copy()
    sin_value_90.reverse()

    # Second half virtual
    sin_value_180 = [-i for i in sin_value]
    sin_value_270 = [-i for i in sin_value_90]

    for MSCNT in range(0, 1024):
        positions.append(MSCNT)

    plt.figure(figsize=[16, 8])
    plt.plot(
        positions,
        sin_value + sin_value_90 + sin_value_180 + sin_value_270,
        label="SIN CUR_A",
    )

    plt.plot(
        positions,
        sin_value_90 + sin_value_180 + sin_value_270 + sin_value,
        label="SIN CUR_B",
    )

    # Custom table PoC
    sin_value = []
    counter = 0
    rollover = False
    for i in range(0, 256):
        sin_value.append(counter)
        if counter < 248 and not rollover:
            if i < 134:
                counter += random.randint(1, 2)
            else:
                counter += 1
        else:
            rollover = True
            counter += -random.randint(0, 1)
    sin_value_90 = sin_value.copy()
    sin_value_90.reverse()

    # Second half virtual
    sin_value_180 = [-i for i in sin_value]
    sin_value_270 = [-i for i in sin_value_90]

    plt.plot(
        positions,
        sin_value + sin_value_90 + sin_value_180 + sin_value_270,
        label="SIN CUR_CUSTOM",
    )

    # # Prusa implementation
    # fac1000 = 170 # 30..200
    # prusa = []
    # fac = 0
    # if fac1000:
    #     fac = float((fac1000 + 1000) / 1000) # correction factor
    # amp = 248 # mslutstart
    # for i in range(0, 1024):
    #     if fac == 0: # default TMC wave
    #         vA = int((amp+1) * math.sin((2 * math.pi*i + math.pi)/1024) + 0.5) - 1
    #     else:
    #         try:
    #             vA = int(amp * pow(math.sin(2 * math.pi*i/1024), fac) + 0.5)
    #         except TypeError:
    #             # complex number, python to powerful
    #             vA = int((amp * pow(math.sin(2 * math.pi*i/1024), fac) + 0.5).real)

    #     prusa.append(vA)
    # plt.plot(positions, prusa, label="SIN Prusa")

    # Add labels and title
    plt.xlabel("MSCNT (position)")
    plt.ylabel("Microstep current by MSLUT: MSCURACT")
    plt.title("TMC MSLUT table view")
    plt.legend()

    plt.savefig("mslut-graph.png")

    new = mslut_encoder(sin_value, sin_value[-1])
    for k in new:
        print(k, new[k])

if __name__ == "__main__":
    main()
