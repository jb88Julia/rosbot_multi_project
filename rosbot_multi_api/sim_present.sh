#!/bin/bash
echo "Launching simulation essentials..."
# initializes simulation world, spawns robots, starts map server, amcl and movebase:
sh ./sim_nav_base.sh
sleep 1
#starts Mission Control server:
#echo "Launching presentation sequence ..."
#gnome-terminal -- roslaunch rosbot_multi_api .launchsim_presentation
gnome-terminal -- roslaunch rosbot_multi_api sim_presentation.launch --screen