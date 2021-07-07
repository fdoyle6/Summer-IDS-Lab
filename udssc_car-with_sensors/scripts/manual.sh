#!/bin/bash

args=("$@")


rosrun joy joy_node &

python manual_drive.py ${args[0]} &


nc -l 2222 | mplayer -fps 200 -demuxer h264es -
