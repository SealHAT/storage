#print random data to file
#   max block size: 64 pages
#   max page size:  2048 bytes (2176 including spare)
#   block RA[16:6]
#   page  RA[5:0]

from random import *
import ast

pageSize  = 2048            #size of a page in flash
dataRange = pageSize*5;     #size of ten pages of data
count = 1                   #counter for data formatting

#open input and output files
originalData = open("OriginalData.txt", "r");
originalData.read();

#
for i in range(dataRange):
    #randomly generate 8-bit uint and write to file
    randomNum = randint(0, 255);
    file.write(str(randomNum));
    unformattedData.write(str(randomNum));

    #if not the last data entry, add a comma after the number
    #or a newline depending on which file we're writing to
    if i != (dataRange - 1):
        file.write(", ");
        unformattedData.write("\n");

    #if the 20th data entry in line, move to new line unless
    #we're at the last data entry
    if count == 20 and i != (dataRange - 1):
        file.write("\n")
        count = 1
    else:
        count += 1

#
originalData.close()
