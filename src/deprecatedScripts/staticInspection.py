from cmds import issueCommands 
import time

if __name__ == "__main__":

    print("Starting inspection.")
    for i in range(9):
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
            issueCommands(0, 70) #ramp up to 50 for 3s
            time.sleep(3)
        elif i == 7:
            issueCommands.car.deploy_EBS() #deploy EBS
            time.sleep(5)

    print("Inspection finished.")
