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
    sin_value = [
         0,  1,  2,  4,  5,  7,  8, 10, # 0..8
        11, 13, 14, 16, 17, 19, 21, 22, # 9..16
        24, 25, 27, 28, 30, 31, 33, 34, # 17..24
        36, 37, 39, 40, 42, 43, 45, 46, # 25..32
        48, 49, 51, 52, 54, 55, 57, 58, # 33..40
        60, 61, 62, 64, 65, 67, 68, 70, # 41..48
        71, 73, 74, 76, 77, 79, 80, 81, # 49..56
        83, 84, 86, 87, 89, 90, 91, 93, # 57..64

        94,   96,  97,  98,  100, 101, 103, 104, # 65..72
        105, 107, 108, 109,  111, 112, 114, 115, # 73..80
        116, 118, 119, 120,  122, 123, 124, 126, # 81..88
        127, 128, 129, 131,  132, 133, 135, 136, # 89..96
        137, 138, 140, 141,  142, 143, 145, 146, # 97..104
        147, 148, 150, 151,  152, 153, 154, 156, # 105..112
        157, 158, 159, 160,  161, 163, 164, 165, # 113..120
        166, 167, 168, 169,  170, 172, 173, 174, # 121..128
        174, 175, 176, 177,  178, 179, 180, 181, # 129..136
        182, 183, 184, 185,  186, 187, 188, 189, # 137..144
        190, 191, 192, 193,  194, 195, 196, 197, # 145..152
        198, 199, 200, 200,  201, 202, 203, 204, # 153..160
        205, 206, 206, 207,  208, 209, 210, 211, # 161..168
        211, 212, 213, 214,  214, 215, 216, 217, # 169..176
        217, 218, 219, 219,  220, 221, 222, 222, # 177..184
        223, 224, 224, 225,  225, 226, 227, 227, # 185..192
        228, 228, 229, 230,  230, 231, 231, 232, # 193..200
        232, 233, 233, 234,  234, 235, 235, 236, # 201..208
        236, 237, 237, 237,  238, 238, 239, 239, # 209..216
        239, 240, 240, 240,  241, 241, 241, 242, # 217..224
        242, 242, 243, 243,  243, 243, 244, 244, # 225..232
        244, 244, 245, 245,  245, 245, 245, 246, # 233..240
        246, 246, 246, 246,  246, 246, 246, 247, # 241..248
        247, 247, 247, 247,  247, 247, 247, 247  # 249..256
    ]
    # counter = 0
    # rollover = False
    # for i in range(0, 256):
    #     if i < 64:
    #         sin_value.append(int(i**1.5/5.5))
    #     else:
    #         sin_value.append(int(math.sin(i/168) * 248))
    #     if i > 2 and (sin_value[-1] - sin_value[-2]) > 2:
    #         print(i, sin_value[-1], sin_value[-2])
    #         return
    sin_value_90 = sin_value.copy()
    sin_value_90.reverse()

    # Second half virtual
    sin_value_180 = [-i for i in sin_value]
    sin_value_270 = [-i for i in sin_value_90]

    plt.plot(
        positions,
        sin_value + sin_value_90 + sin_value_180 + sin_value_270,
        label="SIN CUR_CUSTOM A",
    )

    plt.plot(
        positions,
        sin_value_90 + sin_value_180 + sin_value_270 + sin_value,
        label="SIN CUR_CUSTOM B",
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
    plt.xticks([i for i in range(0, 1024, 64)])
    plt.yticks([i for i in range(0, 256, 16)])
    plt.ylabel("Microstep current by MSLUT: MSCURACT")
    plt.title("TMC MSLUT table view")
    plt.legend()

    plt.savefig("mslut-graph.png")

    new = mslut_encoder(sin_value, sin_value[-1])
    print(f"driver_MSLUT0: {new["MSLUTS"][0]}")
    print(f"driver_MSLUT1: {new["MSLUTS"][1]}")
    print(f"driver_MSLUT2: {new["MSLUTS"][2]}")
    print(f"driver_MSLUT3: {new["MSLUTS"][3]}")
    print(f"driver_MSLUT4: {new["MSLUTS"][4]}")
    print(f"driver_MSLUT5: {new["MSLUTS"][5]}")
    print(f"driver_MSLUT6: {new["MSLUTS"][6]}")
    print(f"driver_MSLUT7: {new["MSLUTS"][7]}")
    print(f"driver_W0: {new["W"][0]}")
    print(f"driver_W1: {new["W"][1]}")
    print(f"driver_W2: {new["W"][2]}")
    print(f"driver_W3: {new["W"][3]}")
    print(f"driver_X1: {new["X"][1]}")
    print(f"driver_X2: {new["X"][2]}")
    print(f"driver_X3: {new["X"][3]}")
    print(f"driver_START_SIN: {new["START_SIN"]}")
    print(f"driver_START_SIN90: {new["START_SIN90"]}")

if __name__ == "__main__":
    main()
