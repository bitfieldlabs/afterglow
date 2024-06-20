#!/usr/bin/python
import sys
import re
import time
import numpy as np
from os import environ
from colorama import Fore, Back, Style
from os import environ

# lamp matrix
NUM_COL = 8
NUM_ROW = 10
lm = np.zeros((NUM_COL, NUM_ROW))

# time
sample_freq = 8000
sample_int = (1000000 / sample_freq)  # sample interval [us]
ttag = 0

# stats
num_updates = 0
lm_toggle_times = np.zeros((NUM_COL, NUM_ROW))
lm_ontimes = np.empty((NUM_COL, NUM_ROW), dtype=object)
lm_offtimes = np.empty((NUM_COL, NUM_ROW), dtype=object)
lm_ontimes_tags = np.empty((NUM_COL, NUM_ROW), dtype=object)

#initialize
for c in range(NUM_COL):
    for r in range(NUM_ROW):
        lm_ontimes[c, r] = []
        lm_offtimes[c, r] = []
        lm_ontimes_tags[c, r] = []


################################################################################

def count_set_bits(word):
    count = 0
    while word:
        count += word & 1
        word >>= 1
    return count

def update_lamp(c, r, state, t):
    # check for lamps turning on
    if (state and not lm[c][r]):
        if (lm_toggle_times[c][r] > 0):
            lm_offtimes[c][r].append((t - lm_toggle_times[c][r]) / 1000)
        lm_toggle_times[c][r] = t
    # check for lamps turning off
    elif (not state and lm[c][r]):
        # store the ontime
        if (lm_toggle_times[c][r] > 0):
            lm_ontimes[c][r].append((t - lm_toggle_times[c][r]) / 1000)
            lm_ontimes_tags[c][r].append(t)
        lm_toggle_times[c][r] = t
    lm[c][r] = state

def update_lamp_matrix(colData, rowData, mode, t):
    global num_updates
    if (mode == 1):
        # WPC mode
        c = -1
        if (colData == 0x01):
            c = 0
        elif (colData == 0x02):
            c = 1
        elif (colData == 0x04):
            c = 2
        elif (colData == 0x08):
            c = 3
        elif (colData == 0x10):
            c = 4
        elif (colData == 0x20):
            c = 5
        elif (colData == 0x40):
            c = 6
        elif (colData == 0x80):
            c = 7
        if (c >=0):
            for r in range(NUM_ROW):
                lampOn = False
                if (rowData & 0x0000001):
                    lampOn = True
                rowData = (rowData >> 1)
                update_lamp(c, r, lampOn, t)
            num_updates += 1
    elif (mode == 2):
        r = -1
        if (rowData == 0x01):
            r = 0
        elif (rowData == 0x02):
            r = 1
        elif (rowData == 0x04):
            r = 2
        elif (rowData == 0x08):
            r = 3
        elif (rowData == 0x10):
            r = 4
        elif (rowData == 0x20):
            r = 5
        elif (rowData == 0x40):
            r = 6
        elif (rowData == 0x80):
            r = 7
        elif (rowData == 0x100):
            r = 8
        elif (rowData == 0x200):
            r = 9
        if (r >=0):
            for c in range(NUM_COL):
                lampOn = False
                if (colData & 0x0000001):
                    lampOn = True
                colData = (colData >> 1)
                update_lamp(c, r, lampOn, t)
            num_updates += 1

def print_histogram(array):
    # Ensure the input is a NumPy array
    if not isinstance(array, np.ndarray):
        raise ValueError("Input must be a NumPy array.")

    # Flatten the array to 1D
    array = array.flatten()
    
    # Get the unique values and their counts
    values, counts = np.unique(array, return_counts=True)
    
    # Determine the maximum count for scaling the histogram
    max_count = counts.max()
    
    # Print the histogram
    print(Fore.CYAN + "Ontime [ms] | Count | Histogram")
    print("-------------------------------")
    for value, count in zip(values, counts):
        bar = '#' * int((count / max_count) * 50)  # Scale the bar to a max of 50 characters
        #print(f"{value:11.1f} | {count:5} | {bar}")
        print(Fore.CYAN + f"{value:11.1f} | {count:5} | ", end='')
        print(Fore.BLUE + f"{bar}")

################################################################################

num_args=len(sys.argv)
if (num_args<=3):
    print(Fore.RED + "Usage: record_replay.py <binary recording file> <lamp column> <lamp row>\nUse a for all columns and rows\n")
    sys.exit()


mode = 0
lastWord = 0;
consistentWords = 0
with open(sys.argv[1], "rb") as f:

    # check for a valid header
    bd = f.read(4)
    bds = int.from_bytes(bd, "little")
    if (bds != 0x50524741):
        print(Fore.RED + "Invalid header!")
        exit(1)

    # read the size
    bd = f.read(4)
    bds = int.from_bytes(bd, "little")
    print(Fore.WHITE + "Magic found. Recording size: " + str(bds))

    while (bd := f.read(4)):
        # read 4 bytes
        word = int.from_bytes(bd, "little")

        # process only consistent new data
        if (word == lastWord):
            consistentWords += 1
        else:
            consistentWords = 0
            lastWord = word;

        if (consistentWords == 4):
            colData = (word & 0x000000ff)
            rowData = ((~word & 0x0003ff00) >> 8)

            # mode auto-detection
            if (mode == 0):
                if (count_set_bits(colData) == 1):
                    print(Fore.GREEN + "WPC Mode detected")
                    mode = 1
                elif (count_set_bits(rowData) == 1):
                    print(Fore.GREEN + "Whitestar Mode detected")
                    mode = 2
                else:
                    print(Fore.RED + "No valid mode detected!")
                    exit(0)

            if (mode != 0):
                # update the lamp matrix
                update_lamp_matrix(colData, rowData, mode, ttag)
        
        ttag += sample_int

print(Fore.WHITE +  "%d updates done\n" % (num_updates))
print("--------------------\n")

################## PRINTOUT ################

evalCol = -1
if (sys.argv[2] != "a"):
    evalCol = int(sys.argv[2])
evalRow = -1
if (sys.argv[3] != "a"):
    evalRow = int(sys.argv[3])


# get evaluation data
ontimes = np.array([])
offtimes = np.array([])
ontimetags = np.array([])
if ((evalCol == -1) and (evalRow == -1)):
    # entire matrix
    for c in range(NUM_COL):
        for r in range(NUM_ROW):
            ontimes = np.concatenate((ontimes, np.array(lm_ontimes[c][r])))
            ontimetags = np.concatenate((ontimes, np.array(lm_ontimes_tags[c][r])))
            offtimes = np.concatenate((offtimes, np.array(lm_offtimes[c][r])))
elif (evalCol == -1):
    # full row
    for c in range(NUM_COL):
        ontimes = np.concatenate((ontimes, np.array(lm_ontimes[c][evalRow])))
        ontimetags = np.concatenate((ontimes, np.array(lm_ontimes_tags[c][evalRow])))
        offtimes = np.concatenate((offtimes, np.array(lm_offtimes[c][evalRow])))
elif (evalRow == -1):
    # full column
    for r in range(NUM_ROW):
        ontimes = np.concatenate((ontimes, np.array(lm_ontimes[evalCol][r])))
        ontimetags = np.concatenate((ontimes, np.array(lm_ontimes_tags[evalCol][r])))
        offtimes = np.concatenate((offtimes, np.array(lm_offtimes[evalCol][r])))
else:
    ontimes = np.array(lm_ontimes[evalCol][evalRow])
    offtimes = np.array(lm_offtimes[evalCol][evalRow])
    ontimetags = np.array(lm_ontimes_tags[evalCol][evalRow])



# HISTOGRAM

print(Fore.CYAN + "Lamp ONTIME histogram for col %d row %d:\n" % (evalCol, evalRow))
print_histogram(ontimes)

# SHORT ONTIME HISTOGRAMS

offt_10 = []
offt_20 = []
offt_30 = []
i=0
for ot in ontimes:
    if (i >= (len(offtimes) -1)):
        break
    if (ot < 12):
        offt_10.append(offtimes[i])
    elif (ot < 22):
        offt_20.append(offtimes[i])
    elif (ot < 32):
        offt_30.append(offtimes[i])
    i += 1

if (len(offt_10)>0):
    print(Fore.CYAN + "\nOFFTIME patterns following 10ms ONTIME:\n")
    print_histogram(np.array(offt_10))

if (len(offt_20)>0):
    print(Fore.CYAN + "\nOFFTIME patterns following 20ms ONTIME:\n")
    print_histogram(np.array(offt_20))

if (len(offt_30)>0):
    print(Fore.CYAN + "\nOFFTIME patterns following 30ms ONTIME:\n")
    print_histogram(np.array(offt_30))


# Pulsing PATTERN
pulse = False
i = 0
pp = 0
for ot in ontimes:
    if (ot < 62):
        if (pp == 0):
            print(Fore.CYAN + "\npulsing pattern sequence:\n")
        pulse = True
        #print("T %.1f On %.1f -> Off %.1f" % (ontimetags[i]/1000, ot, offtimes[i]))
        print(Fore.CYAN + "T %.1f " % (ontimetags[i]/1000), end='')
        print(Fore.GREEN + "On %.1f " % ot, end='')
        print(Fore.RED + "-> Off %.1f " % offtimes[i])
        if (offtimes[i] > 500):
            print("-----")
        pp += 1
    i += 1
