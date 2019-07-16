#!/bin/bash

for pid in $(ps -aux | grep -v grep | grep zed | sed s/  */ /g | cut -d" " -f2)
do 
	kill -sigkill $pid
done
