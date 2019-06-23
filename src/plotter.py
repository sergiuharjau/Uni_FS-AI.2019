import matplotlib.pyplot as plt
import sys

def plotCommands(path1, path2=None):

    f1 = open(path1, "r")
    rawComs = f1.read().split(",")

    commands = [0]

    for com in rawComs:
        if com == "None" or com == "":
            com = commands[-1]
        commands.append(int(com)) #offset from last one

    commands[:] = [x / 100 for x in commands]
    frames = range(len(commands))

    plt.plot(commands, frames, "green") #green for benchmark
    plt.xlabel("Commands")
    plt.ylabel("Frames")

    if path2:
        f2= open(path2, "r")
        rawComs2 = f2.read().split(",")

        commands = [0]

        for com in rawComs2:
            if com == "None" or com == "":
                com = commands[-1]
            commands.append(int(com)) #offset from last one

        commands[:] = [x / 100 for x in commands]
        frames = range(len(commands))

        plt.plot(commands, frames, "blue") #blue for newCom

    plt.show()

if __name__ == "__main__":

    setBench = False #sets past mission commands to Benchmark for given mission
    mission = 5

    for argument in sys.argv[1:]:
        exec(argument)

    if setBench:
        open("../test/mission" + str(mission) + "/commands.txt", "w").write(open("../test/pastMission.txt", "r").read())
        print("Past mission commands.txt is now the new benchmark.")
        quit()

    plotCommands("../test/mission" + str(mission) + "/commands.txt", "../test/pastMission.txt")
