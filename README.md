# Coventry University Formula Student AI 2019

## About repository

This code is currently considered deprecated. For future developments email sergiuharjau@gmail.com, we are porting our system on a ROS-Melodic architecture. 

This code shall stand as a testament of our achievements at FS-AI 2019, having given us a win in the Acceleration and Sprint events, providing us the fastest timed lap in the Endurance Race, and trying its damn best in the Figure of Eight event. 

As such, proceed with care. This code was developed in 6 weeks. Here be dragons.

## Autonomous start up procedure 

Here for legacy reasons. 

Set up a track with Yellow on the left and Blue on the right.
<br>

python3 src/autocross.py --> follow on screen commands 

(run with "nohup [command] &" to ensure shell does not kill the process)
<br>

<pre>
Args:

visual=1 -->  Does not start the CAN procedure, display a visual.

rc=1     -->  Instead of CAN it sends PWM to the RC car.

replay=? -->  Replays mission number ? from past recordings.

record=1 -->  Does not send commands to the car, just records video.

loop=?   -->  Instead of looping forever, loops for given ?.

cflip=1  -->  When Blue on left and Yellow on right.

green=1  -->  Stops the car whenever it has green in front of it.
</pre>
<br>

Different scrips exist (such as acceleration or figureof8), but expect them to not work from the get-go, they were done in a very rushed manner to simply compete at all. 

## Notable things about the car

On the 2019 car we had to ensure a multitude of things occured:

Grossfunk is powered on.

Wheels are within 5 degrees from centre.

Grossfunk/E-Stops on the side are not engaged.

Autonomous Dongle terminates the CAN Bus.
