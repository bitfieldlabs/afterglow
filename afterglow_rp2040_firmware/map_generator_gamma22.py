#!/usr/bin/python
import sys
from os import environ
from colorama import Fore, Back, Style

num_args=len(sys.argv)
if (num_args<=2):
    print(Fore.RED + "Usage: map_generator.py <mapSize> <numSteps>")
    sys.exit()

mapSize = int(sys.argv[1])
numSteps= int(sys.argv[2])

print("static const uint8_t skMap_%d_%d[%d] =" % (mapSize,numSteps,mapSize))
print("{", end ="")
for i in range(0, mapSize):
    s = (pow((1/8)/mapSize*(i+1),2.2) * (1/pow(1/8,2.2)) * (numSteps-1))
    if ((i % 32) == 0):
        print("\n    ", end ="")
    if (i<(mapSize-1)):
        print("%d" % int(s+0.5), end =",")
    else:
        print("%d" % int(s+0.5), end ="")
print("")
print("};")
