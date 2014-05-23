#! /usr/bin/env python3
# -*- coding: utf-8 -*-

# window.py

#############################################################################
# Copyright (C) Labomedia January 2014
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software Foundation,
#  Inc., 51 Franproplin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
#############################################################################

import cv2
import numpy as np
from config import load_config, save_config
from webcam import apply_all_cam_settings, apply_cam_setting

class Window():
    '''Manage a OpenCV window with trackbar.'''
    def __init__(self, name, width, height, k, trackbarList, conf, section):
        self.name = name
        self.trackbarList = trackbarList
        self.section = section
        self.k = k
        self.w = int(width * self.k)
        self.h = int(height * self.k)
        self.new_im = None  # Resized image
        self.win_set = {}  # win_set = dict of tackbar key, value
        self.create_all_trackbars(conf)

    def display(self, im, conf):
        # Resize
        self.new_im = cv2.resize(im, (self.w, self.h))
        # Get trackbar position
        self.get_trackbar_position(conf)
        # Display
        cv2.imshow(self.name, self.new_im)

    def create_trackbar(self, title, maxi):
        cv2.namedWindow(self.name)
        cv2.createTrackbar(title, self.name, 0, maxi, nothing)

    def create_all_trackbars(self, conf):
        if self.trackbarList != None:
            for item in self.trackbarList:
                self.create_trackbar(item[0], item[1])
                # Init trackbar position from config
                # item[0] = "Brightness", item[2] = "brightness"
                cv2.setTrackbarPos(item[0], self.name, conf[item[2]])
                self.win_set[item[2]] = conf[item[2]]

    def get_trackbar_position(self, conf):
        if self.trackbarList != None:
            for item in self.trackbarList:
                # get current positions of trackbars
                new = cv2.getTrackbarPos(item[0], self.name)
                # If I move the cursor
                if new != self.win_set[item[2]]:
                    self.apply_cursor_change(conf, new, item[0], item[2])
                    self.win_set[item[2]] = new

    def apply_cursor_change(self, conf, new, trackbar_name, ini_name):
        # Save in conf dict
        conf[ini_name] = new
        # Apply cam settings if cam parameter
        if self.section == conf["webcam"]:
            print "apply"
            # apply_cam_setting("Brightness" = trackbar_name, new)
            # with trackbar_name = trackbarList[0][0]
            apply_cam_setting(int(conf["cam"]), trackbar_name, new)
        # Save in scan.ini
        # save_config(section, "brightness" = trackbarList[0][1], value)
        if self.section:
            # HD5000 255 29
            save_config(self.section, ini_name, new)

    def add_lines(self, im, conf):
        w = int(self.w/self.k)
        h = int(self.h/self.k)
        mah = conf["motor_axis_h"]
        mav = conf["motor_axis_v"]
        ph = conf["persp_h"]
        pv = conf["persp_v"]
        lat = conf["cut_lateral"]
        up = conf["cut_up"]
        down = conf["cut_down"]

        # Middle lines black
        cv2.line(im, (0, int(h/2)), (w, int(h/2)), (0,0,0), 1)
        cv2.line(im, (int(w/2), 0), (int(w/2), h), (0,0,0), 1)
        # Little lines black
        for t in [2, 26]:
            # Down
            for i in range(-40, 40, 2):
                p = int(w/2 + i*2)
                u = int(t*h/32)
                cv2.line(im, (p, -5 + u), (p, 5 + u), (0, 0, 0), 1)
            # Up
            for i in range(-4, 5, 2):
                p = int(w/2 + i*20)
                j = int(t*h/32)
                cv2.line(im, (p, -40 + j), (p, 40 + j), (0, 0, 0), 1)

        # Motor axis H with vertical line gray
        mahg = mah + int(w/2)  - 15
        cv2.line(im, (mahg, 0), (mahg, h), (100, 100, 100), 1)
        # Motor axis V with horizontal line green
        mavg = h - mav
        cv2.line(im, (0, mavg), (w, mavg), (0, 255, 0), 1)

        # Perspective lines
        # Perspective Horizontal set with vertical red line
        i = int(h - pv)
        cv2.line(im, (0, i), (w, i), (255, 0, 0), 1)
        # Perspective Vertical set with horizontal blue line
        j = int(w/2 - ph + 15)
        cv2.line(im, (j, 0), (j, h), (0, 0, 255), 1)

        # Crop lines
        # Up
        u = int(up)
        cv2.line(im, (0, u), (w, u), (255,255,255), 1)
        # Down
        d = int(h - down)
        cv2.line(im, (0, d), (w, d), (255,255,255), 1)
        # Lateral
        l = int(w - lat)
        cv2.line(im, (l, 0), (l, h), (255,255,255), 1)

        return im

def nothing(x):
    pass

if __name__=='__main__':
    conf = load_config("./scan.ini")

    name = "Test"
    COLOR = [("Brightness", 255, "brightness"),
            ("Saturation", 255, "saturation"),
            ("White Balance Temperature", 10000, "white_bal_temp"),
            ("Focus (absolute)", 40, "focus_abs")]
    win1 = Window(name, 720, 1280, 0.5, COLOR, conf, conf["webcam"])

    geometry = None
    win2 = Window("deux", 1280, 720, 0.4, geometry, conf, None)

    capture = cv2.VideoCapture(0)
    capture.set(3, 1280)
    capture.set(4, 700)
    while True:
        ret, frame = capture.read()
        if ret:
            rotated = cv2.transpose(frame)
            win1.display(rotated, conf)
            win2.display(frame, conf)
            # wait for esc key to exit
            key = np.int16(cv2.waitKey(33))
            if key == 27:
                break
        else:
            print("Webcam is busy\nor ")
    cv2.destroyAllWindows()
