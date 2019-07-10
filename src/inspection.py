from cmds import issueCommands 

if __name__ == "__main__":

	print("Starting inspection.")
    for i in range(6):
        if i == 0 :
            issueCommands(0, 0) #connects and sets up CAN 
        elif i == 1:
            issueCommands(24, 0)
            time.sleep(2)
        elif i == 2:
            issueCommands(-24, 0)
            time.sleep(2)
        elif i == 3:
            issueCommands(0, 0) #briefly straightens up
            time.sleep(1)
        elif i == 4:
            issueCommands(0, 100)
            time.sleep(5)
        elif i == 5:
            issueCommands(0, 0) #waits for wheels to slow down 
            time.sleep(3)
    issueCommands(0, 0, True)
    print("Inspection finished.")