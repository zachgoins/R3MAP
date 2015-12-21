#!/bin/bash
cd ..
cd ~/repos/originals/R3MAP/src/R3MAP/navigation
roslaunch r3map_launch nav.launch &
sudo build/main --mlock sylphase-usbgpsimu2 --antenna-position '[0.127, 0.01, 0.254]' ! tracker ! kf2 --decimation 10 ! listen-solution-tcp 1234



