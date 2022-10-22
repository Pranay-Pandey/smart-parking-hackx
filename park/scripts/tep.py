#!python3
from data_acquisition_and_update import normalized

new = []

d = {1:1, 2:13, 3:2, 4:14, 5:3, 6:15, 7:4, 8:16, 9:5, 10:17, 11:6, 12:18, 13:7, 14:19, 15:8, 16:20, 17:9, 18:21, 19:10, 20:22, 21:11, 22:23, 23:12, 24:24}

for i in d.keys():

    if i == 1:
        new.append(normalized[0])
        new.append(normalized[1])
        continue

    if d[i]%2 == 0:
        new.append(normalized[25 - d[i]])
        new.append(normalized[25 - d[i] + 1])
    else:
        new.append(normalized[25 - d[i] + 1])
        new.append(normalized[25 - d[i] + 2])

print(new)