import sys, os

b = os.popen('cd ../data/real/Annotations/ && ls | grep .xml').readlines()
# print(type(b))
# print(b[0])
file = open("../data/real/ImageSets/Main/trainval.txt","w")
for i in range(len(b)):
    substring = ".xml"
    output_string = ""
    str_list = b[i].split(substring)
    for element in str_list:
        output_string += element
    b[i] = output_string
for L in b:
    file.writelines(L) 
file.close()
os.system('cp \
        ../data/real/ImageSets/Main/trainval.txt \
        ../data/real/ImageSets/Main/test.txt')