import rclpy
from rclpy.node import Node

import tkinter as tk
from tkinter import ttk
import threading

from std_msgs.msg import Int16MultiArray

class ThreeTofDisplay(Node):

    def __init__(self):
        super().__init__('three_tof_display')
        self.get_logger().info('Starting three_tof_display.')

        self.color = -1
        self.range_array = [[x + 0.1*y for x in range(8)] for y in range(8)]
        self.frame_array = [['' for x in range(8)] for y in range(8)]
        self.label_array = [['' for x in range(8)] for y in range(8)]
        self.color_sub = self.create_subscription(
            Int16MultiArray, 'range_array', self.range_listener_cb, 10
        )
        self.window_thread = threading.Thread(target=self.window_thread_entry)
        self.window_thread.start()

    def range_listener_cb(self, msg):
        #self.get_logger().info('got range msg')
        for i in range(8):
            for j in range(8):
                range_mm = float(msg.data[i*8+j])
                self.range_array[i][j] = range_mm

    """
    range is in mm.
    5+m: dark grey
    """
    def range2color(self, range):
        named_color = False
        red = green = blue = 250
        range_increment = int((range % 1000) / 4)    # map increment over a meter to 0-250

        if (range < 0):
            #negative: black
            range_color = "black"
            named_color = True
        elif (range < 1000):
            # 0 - 1m: full red with increasing green
            blue = 0
            green = range_increment   # convert to span 0 - 250, increasing green
        elif (range < 2000):
            # 1 - 2m: decreasing red with full green
            blue = 0
            red = 250 - range_increment
        elif (range < 3000):
            # 2 - 3m: full green with increasing blue
            red = 0
            blue = range_increment
        elif (range < 4000):
            # 3 - 4m: full blue with decreasing green
            red = 0
            green = 250 - range_increment
        elif (range < 5000):
            # 4 - 5m: decreasing blue
            red = 0
            green = 0
            blue = 250 - range_increment
        else:
            range_color = "dark grey"
            named_color = True
        
        if (named_color):
            range_color = named_color
        else:
            range_color = '#%02x%02x%02x'%(red, green, blue)
            print(range, red, green, blue, range_color)
        return range_color

    def update_display(self):
        self.get_logger().info("updating display")
        for i in range(8):
            for j in range(8):
                range_string = '{0:.3f}'.format(self.range_array[i][j]/1000)
                range_color = self.range2color(self.range_array[i][j])
                self.label_array[i][j]['text'] = range_string
                self.label_array[i][j]['fg'] = "white"
                try:
                    self.label_array[i][j]['bg'] = range_color
                except Exception as e:
                    print(range_color)
                    print("Exception: ", e)
                self.label_array[i][j].pack()

        self.window.after(500, self.update_display)
        return

    def window_thread_entry(self):
        self.window = tk.Tk()

        range_string = '{0:.3f}'.format(self.color)
        for i in range(8):
            self.window.columnconfigure(i, weight=1, minsize=50)
            self.window.rowconfigure(i, weight=1, minsize=50)
            for j in range(8):
                self.frame_array[i][j] = tk.Frame(
                    master = self.window,
                    relief = tk.RAISED,
                    borderwidth = 1
                )
                self.frame_array[i][j].grid(row=i, column=j, padx=5, pady=5)
                self.label_array[i][j] = tk.Label(master=self.frame_array[i][j], text='{0:.2f}'.format(self.range_array[i][j]))
                self.label_array[i][j].pack(padx=5, pady=5)

        self.window.after(500, self.update_display)
        self.window.mainloop()


def main():
    test_range2color = False

    rclpy.init()

    nh = ThreeTofDisplay()

    if test_range2color:
        print("range2color test")
        test_range = -1
        print(test_range, nh.range2color(test_range))
        test_range = 500
        print(test_range, nh.range2color(test_range))
        test_range = 1500
        print(test_range, nh.range2color(test_range))
        test_range = 2500
        print(test_range, nh.range2color(test_range))
        test_range = 3500
        print(test_range, nh.range2color(test_range))
        test_range = 4500
        print(test_range, nh.range2color(test_range))
        test_range = 5500
        print(test_range, nh.range2color(test_range))
        system.exit()

    rclpy.spin(nh)

if __name__ == '__main__':
    main()
