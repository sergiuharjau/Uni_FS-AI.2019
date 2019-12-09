# FS-AI.2019

<h1>Start up sequence:</h1>

<h2><b>Mechanical: </h2></b> 

-Wheels need to be straight.

-PeakCAN device is plugged in.

-CAN needs to be terminated properly.

-Gross funk is on channel 9, red button is twisted.

-Switch side levers on, right to left.

-Check the 12V power level.

-Second screen variables are all False. 



<h2><b>Software: </b></h2>

-The computer automatically boots with David Croft's drivers at start up. 

-These drivers bridge the PeakCAN -- COM1 connection, and allows us to talk to the CAN. 

-Run "python3 src/zedMain.py visual=1" to ensure it compiles properly. 

-With the visual on, double check the cones are in the pixel strip. 

-Full self driving: run python3 src/zedMain.py and follow the instructions on the screen.


<h2>Optional arguments</h2>

<b>-Using any of these arguments will disable the CAN:</b>

visual=1 - brings up a preview of the camera

rc=1 - used for sending commands to the rc CAR 

replay=X - replays recordings found in ../test/missionX

record=1 - records footage and saves it in aforementioned folder

<b>-These arguments run with CAN enabled: </b>

cFlip=1 - flips the colours to Yellow on the Right, Blue on the Left

green=1 - enables stopping the car on green detection
