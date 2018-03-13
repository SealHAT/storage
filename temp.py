file = open("FeatherOutput.txt", "r");

fileData = file.read();
file.close();
fileData.replace(" ", "");
fileData.replace("\n", "");
splitData = []
splitData = fileData.split(",")

print("Length of splitData: " + str(len(splitData)) + "\n");
file = open("FeatherOutput.txt", "w");

i = 0;
while i < len(splitData):
    file.write(splitData[i]);
    file.write("\n");
    i+=1

file.close();
