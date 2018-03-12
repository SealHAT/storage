#print random data to file
#   max block size: 64 pages
#   max page size:  2048 bytes (2176 including spare)
#   block RA[16:6]
#   page  RA[5:0]

import datetime

pageSize = 2048
dataRange = pageSize*10
originalData = []
storedData = []
errorCount = 0

#open the file containing the data that the generate script created
file = open("testData.txt", "r")

#each value will be stored on its own line in the text file. split the
#values into a list of numbers by using the newline character as the 
#delimiter. then cast all strings to integers since the "read" function
#reads all data in as a string
originalData = list(map(int, file.read().splitlines())) #so pythonic

file.close()

#open the file containing the data output from the microcontroller
file = open("FeatherOutput.txt", "r")

storedData = list(map(int, file.read().splitlines())) #so pythonic

file.close()

#check for data integrity
for i < len(storedData) and i < len(originalData):
    if storedData[i] != originalData[i]:
        errorCount += 1
        print("Error at index " + str(i) + "\n")

if len(storedData) != len(originalData):
    errorCount += 1
    print("The new and original data sizes do not match!\nOriginal: " + str(len(originalData)) + " New: " + str(len(storedData)))

#output results to an output file
file = open("TestResults.txt", "a")

file.write("Test at " + str(datetime.datetime.now()) + "\tThere were " + str(errorCount) + " errors.\n")

file.close()
