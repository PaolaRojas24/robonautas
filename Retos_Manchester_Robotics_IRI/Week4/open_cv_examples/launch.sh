#!/bin/bash

# Abre cada nodo en una nueva terminal (usa gnome-terminal; puedes cambiarlo por xterm, konsole, etc.)
gnome-terminal -- bash -c "source ~/ros2_w/install/setup.bash; ros2 run open_cv_examples puzzlebot_odometry; exec bash"
gnome-terminal -- bash -c "source ~/ros2_w/install/setup.bash; ros2 run open_cv_examples controller; exec bash"
gnome-terminal -- bash -c "source ~/ros2_w/install/setup.bash; ros2 run open_cv_examples path_generator; exec bash"
gnome-terminal -- bash -c "source ~/ros2_w/install/setup.bash; ros2 run open_cv_examples color_detection; exec bash"
gnome-terminal -- bash -c "source ~/ros2_w/install/setup.bash; ros2 run rqt_image_view rqt_image_view; exec bash"
