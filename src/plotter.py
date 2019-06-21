import matplotlib.pyplot as plt

f1 = open("../test/mission5/commands.txt", "r")
rawComs = f1.read().split(",")

commands = [0]

for com in rawComs:
    if com == "None" or com == "":
        com = commands[-1]
    commands.append(int(com)/10) #offset from last one

frames = range(len(commands))

plt.plot(commands, frames)
plt.xlabel("Commands")
plt.ylabel("Frames")
plt.show()
