# three_tof_display
A ROS2 - Tkinter GUI for displaying range data from 8x8 ToF sensors

This GUI is based on the article at https://realpython.com/python-gui-tkinter/

The package contains a data generator which cycles through publishing different ranges.

The GUI takes range data from a ROS2 message and displays it in an 8x8 colorized
grid, in which the range is shown for each cell, and the color varies from red
for close, through green to blue for far away.
