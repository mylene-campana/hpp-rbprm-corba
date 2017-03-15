#!/bin/bash
# File to launch gepetto-viewer, hpp-corbaserver and a script.

# Command: sh autoRun.sh

# In first terminal : viewer-server
xterm -e gepetto-viewer-server &
sleep 1s
# In first terminal : corbaserver
xterm -e hpp-rbprm-server &
sleep 1s
# In second terminal : python script
#xterm -e python -i skeleton_test_path.py &
#xterm -e python -i skeleton_desert_path.py &
#xterm -e python -i skeleton_parkourWalls_path.py &
#xterm -e python -i lamp_test_path.py &
#xterm -e python -i lamp_plateforms_path.py &
#xterm -e python -i lamp_test_interp.py &
#xterm -e python -i ant_test_interp.py &
#xterm -e python -i spiderman_backJump_interp.py &
xterm -e python -i kangaroo_desert_path.py &
#xterm -e python -i frog_test_path.py &

sleep 900s
killall xterm
