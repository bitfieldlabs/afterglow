#!/usr/bin/python
import sys
from os import environ
from colorama import Fore, Back, Style

num_args=len(sys.argv)
if (num_args!=2):
    print(Fore.YELLOW + "  Usage: record_analysis.py <input_file>")
    sys.exit()

fn=str(sys.argv[1])
print ('Reading ', fn)

# read the file
try:
    f = open(fn, 'rb')
except OSError:
    print(Fore.RED + "  ERR Could not open/read file:", fn)
    sys.exit()

rec_data=list(f.read())
f.close()

num_bytes=len(rec_data)

# Check header
num_bytes_header = 0
num_rec = 0
if ((rec_data[0]==65) and (rec_data[1]==71) and (rec_data[2]==82) and (rec_data[3]==80)):
    num_bytes_header=int.from_bytes(rec_data[4:8], byteorder='little', signed=False)
    num_rec = int((num_bytes_header-8)/4)
    print(Fore.GREEN + "  REC data found: %d bytes, %d records" % (num_bytes_header, num_rec))
    print(Style.RESET_ALL)
else:
    print(Fore.RED + "  ERR No REC data found!")
    sys.exit()

byte_pos = 8; # 8 bytes header

# extract mode
sample_freq = 8000
t = 0
dt = int(1000000 / sample_freq)  # [us]
print("index, time[us], column bits, row bits")
for i in range(num_rec):
    rec = int.from_bytes(rec_data[byte_pos:byte_pos+4], byteorder='little', signed=False)
    colData = (rec & 0x000000ff)
    rowData = (rec >> 8)
    #colStr = hex(colData)
    colStr = "0x"+format(colData, '02x')+" "+"{0:08b}".format(colData)
    rowStr = "0x"+format(rowData, '03x')+" "+"{0:10b}".format(rowData)
    print("%d, %.6lf, %s, %s" % (i, t/1000000.0, colStr, rowStr))

    byte_pos += 4
    t += dt
