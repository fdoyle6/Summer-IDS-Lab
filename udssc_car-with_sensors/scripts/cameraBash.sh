#!/bin/sh

/opt/vc/bin/raspivid -t 0 -w 300 -h 300 -fps 20 -o - | nc 192.168.1.188 2222

