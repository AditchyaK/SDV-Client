import time

data = "AB1BB2CB3DA4END"
keys = ("AB", "BB", "CB", "DA", "END")
datatru = []

"""
datanu = data.split("A")

datanu = datanu[1].split("B")
datatru.append(datanu[0])

datanu = datanu[1].split("C")
datatru.append(datanu[0])

datanu = datanu[1].split("D", 1)
datatru.append(datanu[0])

datanu = datanu[1].split("END")
datatru.append(datanu[0])

datanu = data.split("")
"""
datanu = data.split(keys[0])

for i in range(len(keys)-1):
    i += 1
    datanu = datanu[1].split(keys[i], 1)
    datatru.append(datanu[0])

print(datatru)
