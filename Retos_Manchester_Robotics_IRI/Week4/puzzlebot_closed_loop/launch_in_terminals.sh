#!/bin/bash

# Abre cada nodo en una nueva terminal (usa gnome-terminal; puedes cambiarlo por xterm, konsole, etc.)
gnome-terminal -- bash -c "source ~/ros2_w/install/setup.bash; ros2 run puzzlebot_closed_loop puzzlebot_odometry; exec bash"
gnome-terminal -- bash -c "source ~/ros2_w/install/setup.bash; ros2 run puzzlebot_closed_loop controller; exec bash"
gnome-terminal -- bash -c "source ~/ros2_w/install/setup.bash; ros2 run puzzlebot_closed_loop path_generator; exec bash"
gnome-terminal -- bash -c "source ~/ros2_w/install/setup.bash; ros2 run puzzlebot_closed_loop prueba; exec bash"
