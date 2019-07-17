from cmds import issueCommands 

if __name__ == "__main__":

	print("Starting inspection.")
    for i in range(6):
        if i == 0:
            issueCommands(0, 0) #connects and sets up CAN 
        elif i == 1:
            issueCommands(24, 0) #sweeping through steering
            time.sleep(2)
        elif i == 2:
            issueCommands(-24, 0) #sweeping through steering
            time.sleep(2)
        elif i == 3:
            issueCommands(0, 0) #straightens up
            time.sleep(1)
        elif i == 4:
            issueCommands(0, 200) #ramps up to 200 within 10s
            time.sleep(5)
        elif i == 5:
            issueCommands(0, 0) #stops within 5 seconds
            time.sleep(3)
        elif i == 6:
            issueCommands.car.set_AS_Finished()
            time.sleep(1)
        elif i == 7:
            issueCommands(0,50) #ramp up to 50 for 3s
            time.sleep(3)
        elif i == 8:
            issueCommands.car.deploy_ebs() #deploy EBS
            time.sleep(2)

    print("Inspection finished.")