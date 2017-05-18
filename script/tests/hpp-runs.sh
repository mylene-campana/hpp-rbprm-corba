#!/bin/bash
# File to launch n times hpp and a script.
#(assuming no need of RViz terminal)

# Command: sh hpp-runs.sh

path="/local/mcampana/devel/hpp"
testPath="/local/mcampana/devel/hpp/src/hpp-runs/scripts"


for i in `seq 1 30`
    do
    echo "Loop begining: "$i
    # In first terminal : server
    xterm -e gepetto-viewer-server &
    sleep 1s
    # In second terminal : server
    xterm -e hpp-rbprm-server &
    sleep 1s
    # In third terminal : python script
    #xterm -e python skeleton_desert_interp.py
    #xterm -e python ant_cave_interp.py
    xterm -e python spiderman_backJump_interp.py
    #wait until Python script is finished
    echo "Loop done"
    killall xterm
    sleep 1s
done # endFor

echo "Jumperman finished -----"

for i in `seq 1 30`
    do
    echo "Loop begining: "$i
    # In first terminal : server
    xterm -e gepetto-viewer-server &
    sleep 1s
    # In second terminal : server
    xterm -e hpp-rbprm-server &
    sleep 1s
    # In third terminal : python script
    xterm -e python skeleton_desert_interp.py
    #xterm -e python ant_cave_interp.py
    #xterm -e python spiderman_backJump_interp.py
    #wait until Python script is finished
    echo "Loop done"
    killall xterm
    sleep 1s
done # endFor

echo "Skeleton finished -----"

for i in `seq 1 30`
    do
    echo "Loop begining: "$i
    # In first terminal : server
    xterm -e gepetto-viewer-server &
    sleep 1s
    # In second terminal : server
    xterm -e hpp-rbprm-server &
    sleep 1s
    # In third terminal : python script
    #xterm -e python skeleton_desert_interp.py
    xterm -e python ant_cave_interp.py
    #xterm -e python spiderman_backJump_interp.py
    #wait until Python script is finished
    echo "Loop done"
    killall xterm
    sleep 1s
done # endFor



# Notes - when writing in a file : > will erase the file, >> will concatenate, 2> will write errors.

