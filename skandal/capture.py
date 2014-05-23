#! /usr/bin/env python3
# -*- coding: utf-8 -*-

# capture.py

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

import subprocess
from time import time, sleep
import cv2
import numpy as np
from window import Window
from config import load_config, save_config
from webcam import apply_all_cam_settings, apply_cam_setting
from arduino import Arduino


COLOR = [("Brightness", 255, "brightness"), # 0
        ("Saturation", 255, "saturation"), # 1
        ("White Balance Temperature", 10000, "white_bal_temp"), # 2
        ("Focus (absolute)", 40, "focus_abs")] # 3

GEOM = [("Laser Left Right", 1, "laser"), # 0
        ("Motor Axis H", 30, "motor_axis_h"), # 1
        ("Motor Axis V", 300, "motor_axis_v"), # 2
        ("Perspective H", 400, "persp_h"), # 3
        ("Perspective V", 100, "persp_v"), # 4
        ("Lateral Crop", 200, "cut_lateral"), # 5
        ("Up Crop", 200, "cut_up"), # 6
        ("Down Crop", 200, "cut_down")] # 7


class Capture():
    '''Set cam, shot and save shot.'''
    def __init__(self, cf):
        self.cf = cf
        if not self.cf["inverse"]:
            self.w = self.cf["width"]
            self.h = self.cf["height"]
        else:
            self.w = self.cf["height"]
            self.h = self.cf["width"]

        # Init device
        self.capture = cv2.VideoCapture(int(self.cf["cam"]))
        self.set_capture()
        self.arduino()
        # Init windows: TODO scale en auto ?
        self.rawWin = Window("Raw", self.w, self.h, 0.6, None, cf, None)
        self.grayWin = Window("Gray", self.w, self.h, 0.5, COLOR, self.cf,
                                                        self.cf["webcam"])
        self.barWin = Window("Trackbar", 500, 1, 1, GEOM, self.cf, "scan")
        self.shotWin = Window("Shot in progress", self.w, self.h, 0.5, None,
                                                            self.cf, None)

    def set_capture(self):
        self.capture.set(3, self.cf["width"])
        self.capture.set(4, self.cf["height"])
        apply_all_cam_settings(self.cf)

    def arduino(self):
        self.arduino = Arduino(self.cf["ard_dev"])
        # Left Laser on so we can see that it's ok
        self.arduino.write('G')

    def set_cam_position(self):
        while True:
            ret, frame = self.capture.read()
            if ret:
                # Rotate and flip
                if self.cf["inverse"]:
                    rot = cv2.flip(cv2.transpose(frame), 1)
                else:
                    rot = frame
                # Display raw
                rot = self.rawWin.add_lines(rot, self.cf)
                self.rawWin.display(rot, self.cf)

                # Display grayscale image
                gray = cv2.cvtColor(rot, cv2.COLOR_BGR2GRAY)
                self.grayWin.display(gray, self.cf)

                # Display trackbar
                sw_old = self.barWin.win_set["laser"]
                self.barWin.display(gray, self.cf)
                sw_new = self.barWin.win_set["laser"]

                # Update laser
                laser_L_or_R(self.arduino, sw_old, sw_new)

                # wait for esc key to exit
                key = np.int16(cv2.waitKey(33))
                if key == 27:
                    break
            else:
                print("Webcam is busy")
        cv2.destroyAllWindows()

    def shot(self):
        top = time()
        nb_shot = 0
        t_shot = time()
        set_laser("left", self.arduino)
        while True:
            way = "left"
            ret, frame = self.capture.read()
            if ret:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                # Rotate 90°
                rotated = cv2.transpose(frame)
                # Display while shoting
                self.shotWin.display(rotated, self.cf)
                # Shot only if cam is stable, it's a very long time
                if time() - top > 5:
                    if way == "left":
                        # Shot every second until shot number
                        if nb_shot < self.cf["nb_img"]:
                            nb_shot, t_shot = take_shot(self.cf["tempo"], top,t_shot,
                            rotated, nb_shot, way, self.arduino, self.cf["img_dir"])
                        else:
                            if self.cf["double"]:
                                way = "right"
                            else:
                                break
                    if way == "right":
                        set_laser(way, self.arduino)
                        # Shot every second until shot number
                        if nb_shot < self.cf["nb_img"] * 2:
                            nb_shot, t_shot = take_shot(self.cf["tempo"], top,t_shot,
                            rotated, nb_shot, way, self.arduino, self.cf["img_dir"])
                        else:
                            break

                # wait for esc key to exit
                key = np.int16(cv2.waitKey(33))
                if key == 27:
                    break
            else:
                print("Webcam is busy")
        print("\n{0} good shot in {1} seconds".format(\
                nb_shot , int(time() - top)))
        cv2.destroyAllWindows()

    def close(self):
        if self.ard == 1:
            set_laser("off", self.arduino)
        self.capture.release()
        cv2.destroyAllWindows()


def take_shot(tempo, top, t_shot, im, nb_shot, way, arduino, im_dir):
    if time() - t_shot > tempo:
        # Shot
        if way == "right":
            im = cv2.flip(im, 1)
        write_shot(im_dir, im, nb_shot, way)
        # Turn one step
        arduino.write("H")
        # Done
        t_shot = time()
        nb_shot += 1
    return nb_shot, t_shot

def set_laser(way, arduino):
    if way == "left":
        # Set left laser on
        arduino.write('G')
        # Set right laser off
        arduino.write('C')
    if way == "right":
        # Set right laser on
        arduino.write('D')
        # Set left laser off
        arduino.write('B')
    if way == "off":
        # Set right laser on
        arduino.write('C')
        # Set left laser off
        arduino.write('B')
        sleep(0.1)
        arduino.close()

def write_shot(im_dir, img, nb_shot, way):
    name = im_dir + "/s_" + str(nb_shot) + ".png"
    cv2.imwrite(name, img)
    print(("Shot {1} {0} taken".format(nb_shot, way)))

def laser_L_or_R(arduino, sw_old, sw_new):
    if sw_old != sw_new:
        if sw_new == 0:
            # Left on
            arduino .write('G')
            arduino .write('C')
        if sw_new == 1:
            # Right on
            arduino .write('D')
            arduino .write('B')

if __name__=='__main__':
    conf = load_config("./scan.ini")
    cap = Capture(conf)
    cap.set_cam_position()
    #cap.shot()
