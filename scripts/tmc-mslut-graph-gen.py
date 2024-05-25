#!/usr/bin/env python3
import matplotlib.pyplot as plt


def main():
    # Klipper TMC5160 default
    driver_MSLUT0 = 2863314260  # 0xAAAAB554
    driver_MSLUT1 = 1251300522  # 0x4A9554AA
    driver_MSLUT2 = 608774441  # 0x24492929
    driver_MSLUT3 = 269500962  # 0x10104222
    driver_MSLUT4 = 4227858431  # 0xFBFFFFFF
    driver_MSLUT5 = 3048961917  # 0xB5BB777D
    driver_MSLUT6 = 1227445590  # 0x49295556
    driver_MSLUT7 = 4211234  # 0x00404222

    # Width control bit coding W0…W3:
    # %00: MSLUT entry 0, 1 select: -1, +0
    # %01: MSLUT entry 0, 1 select: +0, +1
    # %10: MSLUT entry 0, 1 select: +1, +2
    # %11: MSLUT entry 0, 1 select: +2, +3
    driver_W0 = 2
    driver_W1 = 1
    driver_W2 = 1
    driver_W3 = 1
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
    driver_X1 = 128
    driver_X2 = 255
    driver_X3 = 255
    driver_START_SIN = 0
    driver_START_SIN90 = 247
    # START_SIN90 gives the absolute current for
    # microstep table entry at positions 256
    _MSLUTS = [
        driver_MSLUT0,
        driver_MSLUT1,
        driver_MSLUT2,
        driver_MSLUT3,
        driver_MSLUT4,
        driver_MSLUT5,
        driver_MSLUT6,
        driver_MSLUT7,
    ]
    bit_diffs = []
    for mslut in _MSLUTS:
        for shift in range(0, 32):
            bit_val = (mslut >> shift) & 1
            bit_diffs.append(bit_val)

    decoded_val = []

    for i in range(0, 256):
        if i < driver_X1 - 1:
            # Segment 0
            offset = driver_W0 - 1
        elif i < driver_X2 - 1:
            # Segment 1
            offset = driver_W1 - 1
        elif i < driver_X3 - 1:
            # Segment 2
            offset = driver_W2 - 1
        else:
            # Segment 3
            offset = driver_W3 - 1
        bit = bit_diffs[i]
        bit_dec = bit + offset
        decoded_val.append(bit_dec)

    sin_value = [driver_START_SIN]
    for val in decoded_val:
        last = sin_value[-1]
        sin_value.append(last + val)

    if sin_value[-1] != driver_START_SIN90:
        print("Are you sure? End of 1/4 of sin is not in sync with 2 of 4")

    # Maybe i'm stupid, as last value is equal it can be ignored..? 257 -> 256
    sin_value.pop()
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

    # Add labels and title
    plt.xlabel("MSCNT (position)")
    plt.ylabel("Microstep current by MSLUT: MSCURACT")
    plt.title("TMC MSLUT table view")
    plt.legend()

    plt.savefig("mslut-graph.png")


if __name__ == "__main__":
    main()
