#print address and random data to file
#   max block size: 64 pages
#   max page size:  2048 bytes (2176 including spare)
#   block RA[16:6]
#   page  RA[5:0]

from random import *

pageSize  = 2048            #size of a page in flash
dataRange = pageSize*10;    #size of ten pages of data
count = 1                   #counter for data formatting

#open output file and write first part of array declaration
file = open("testdata.txt", "w")
file.write("uint8_t TEST_DATA[] = {\n")

#generate random 8-bit numbers and format them like an array
#declaration. there will be 20 numbers per line, comma 
#delimited
for i in range(dataRange):
    #randomly generate 8-bit uint and write to file
    file.write(str(randint(0, 255)))

    #if not the last data entry, add a comma after the number
    if i != (dataRange - 1):
        file.write(", ")

    #if the 20th data entry in line, move to new line unless
    #we're at the last data entry
    if count == 20 and i != (dataRange - 1):
        file.write("\n")
        count = 1
    else:
        count += 1

#add the closing array declaration bracket and close output file
file.write("};")
file.close()
