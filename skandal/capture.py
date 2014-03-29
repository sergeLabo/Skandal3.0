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


from time import time, sleep
import cv2
import numpy as np
from config import load_config, save_config
from cam_settings import apply_all_cam_settings, apply_cam_setting
from arduino import Arduino

class Capture():
    '''Set cam, shot and save shot.'''
    def __init__(self, cf):
        self.cf = cf
        self.cam = cf["cam"]
        self.width = self.cf["width"]
        self.height = self.cf["height"]
        self.steps = self.cf["nb_img"]
        self.double = self.cf["double"]
        self.capture = cv2.VideoCapture(self.cam)
        self.capture.set(3, self.width)
        self.capture.set(4, self.height)
        apply_all_cam_settings(cf)
        self.arduino = init_arduino(self.cf)

    def set_cam_position(self):
        create_trackbar_raw()
        create_trackbar_gray()
        # set = 0 br, 1 co, 2 sh, 3 fo, 4 sw, 5 ph, 6 pv, 7 ma
        old_set = set_init_trackbar(self.cf)

        while True:
            ret, frame = self.capture.read()
            if ret:
                # Display original image
                ph, pv, ma = display_raw(frame, self.width, self.height,
                                                                    self.cf)
                # Display grayscale image
                br, co, sh, fo, sw = display_gray(frame, self.width,
                                                                self.height)
                new_set = br, co, sh, fo, sw, ph, pv, ma
                # Apply change and update conf
                self.cf = apply_conf_change(self.cf, old_set, new_set)
                # Update laser
                laser_D_on_off(self.arduino, old_set, sw)
                old_set = new_set
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
        while True:
            way = "left"
            ret, frame = self.capture.read()
            if ret:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                display_raw(frame, self.width, self.height, self.cf)
                # Shot only if cam is stable, it's a very long time
                if time() - top > 5:
                    if way == "left":
                        # Shot every second until shot number
                        if nb_shot < self.steps:
                            nb_shot, t_shot = take_shot(self.cf, top, t_shot,
                                frame, nb_shot, way, self.arduino)
                        else:
                            if self.double:
                                way = "right"
                            else:
                                doloop = end_shot(way, self.arduino)
                    if way == "right":
                        set_laser(way, self.arduino)
                        # Shot every second until shot number
                        if nb_shot < self.steps * 2:
                            nb_shot, t_shot = take_shot(self.cf, top, t_shot,
                                frame, nb_shot, way, self.arduino)
                        else:
                            break
                # wait for esc key to exit
                key = np.int16(cv2.waitKey(33))
                if key == 27:
                    break
            else:
                print("Webcam is busy")
        print("\n{0} good shot in {1} seconds".format(\
                nb_shot * (self.double + 1), int(time() - top)))
        cv2.destroyAllWindows()

    def close(self):
        if self.ard == 1:
            set_laser("off", self.arduino)
        cv2.destroyAllWindows()
        self.capture.release()

def apply_conf_change(cf, old_set, new_set):
    # set = 0 br, 1 co, 2 sh, 3 fo, 4 sw, 5 ph, 6 pv, 7 ma
    if old_set[0] != new_set[0]:
        cf["brightness"] = new_set[0]
        apply_cam_setting("Brightness", new_set[0])
        save_config(cf["webcam"], "brightness", new_set[0])
    if old_set[1] != new_set[1]:
        cf["contrast"] = new_set[1]
        apply_cam_setting("Contrast", new_set[1])
        save_config(cf["webcam"], "contrast", new_set[1])
    if old_set[2] != new_set[2]:
        cf["sharpness"] = new_set[2]
        apply_cam_setting("Sharpness", new_set[2])
        save_config(cf["webcam"], "sharpness", new_set[2])
    if old_set[3] != new_set[3]:
        cf["focus_abs"] = new_set[3]
        apply_cam_setting("Focus (absolute)", new_set[3])
        save_config(cf["webcam"], "focus_abs", new_set[3])
    if old_set[5] != new_set[5]:
        cf["persp_h"] = new_set[5]
        save_config("scan", "persp_h", new_set[5])
    if old_set[6] != new_set[6]:
        cf["persp_v"] = new_set[6]
        save_config("scan", "persp_v", new_set[6])
    if old_set[7] != new_set[7]:
        cf["motor_axis"] = new_set[7]
        save_config("scan", "motor_axis", new_set[7])
    return cf

def create_trackbar_raw():
    cv2.namedWindow("Raw Webcam")
    cv2.createTrackbar("Motor Axis", "Raw Webcam", 0, 200, nothing)
    cv2.createTrackbar("Perspective H", "Raw Webcam", 0, 400, nothing)
    cv2.createTrackbar("Perspective V", "Raw Webcam", 0, 100, nothing)

def create_trackbar_gray():
    cv2.namedWindow("Gray Scale")
    cv2.createTrackbar("Brightness", "Gray Scale", 30, 255, nothing)
    cv2.createTrackbar("Contrast", "Gray Scale", 0, 10, nothing)
    cv2.createTrackbar("Sharpness", "Gray Scale", 0, 50, nothing)
    cv2.createTrackbar("Focus (absolute)", "Gray Scale", 0, 40, nothing)
    switch = '0: 1 laser \n1: 2 laser'
    cv2.createTrackbar(switch, "Gray Scale", 0, 1, nothing)

def set_init_trackbar(cf):
    ph = cf["persp_h"]
    pv = cf["persp_v"]
    ma = cf["motor_axis"]
    cv2.setTrackbarPos("Perspective H", "Raw Webcam", ph)
    cv2.setTrackbarPos("Perspective V", "Raw Webcam", pv)
    cv2.setTrackbarPos("Motor Axis", "Raw Webcam", ma)
    br = cf["brightness"]
    co = cf["contrast"]
    sh = cf["sharpness"]
    fo = cf["focus_abs"]
    sw = 0
    cv2.setTrackbarPos("Brightness", "Gray Scale", br)
    cv2.setTrackbarPos("Contrast", "Gray Scale", co)
    cv2.setTrackbarPos("Sharpness", "Gray Scale", sh)
    cv2.setTrackbarPos("Focus (absolute)", "Gray Scale", fo)
    return br, co, sh, fo, sw, ph, pv, ma

def display_raw(im, width, height, cf):
    '''k = coeff multiplicateur'''
    # get current positions of trackbars
    ph = cv2.getTrackbarPos("Perspective H", "Raw Webcam")
    pv = cv2.getTrackbarPos("Perspective V", "Raw Webcam")
    ma = cv2.getTrackbarPos("Motor Axis", "Raw Webcam")
    # Display with resize
    k = 1
    w = int(width * k)
    h = int(height * k)
    im = cv2.resize(im, (w, h))
    if 1400 <= width :
        k = 0.5
    elif 1280 <= width < 1400:
        k = 0.9
    else:
        k = 1
    W = int(width * k)
    H = int(height * k)
    im = cv2.resize(im, (W, H))
    im = add_lines(im, W, H, cf)
    cv2.imshow("Raw Webcam", im)
    #cv2.moveWindow("Raw Webcam", 0, 0)
    return ph, pv, ma

def display_gray(im, width, height):
    # get current positions of trackbars
    br = cv2.getTrackbarPos("Brightness", "Gray Scale")
    co = cv2.getTrackbarPos("Contrast", "Gray Scale")
    sh = cv2.getTrackbarPos("Sharpness", "Gray Scale")
    fo = cv2.getTrackbarPos("Focus (absolute)", "Gray Scale")
    switch = '0: 1 laser \n1: 2 laser'
    sw = cv2.getTrackbarPos(switch, "Gray Scale")
    # Display
    im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    if width < 1200:
        k = 1
    else:
        k = 0.5
    W = int(width * k)
    H = int(height * k)
    im = cv2.resize(im, (W, H))
    cv2.imshow("Gray Scale", im)
    #cv2.moveWindow("Gray Scale", 100, 0)
    return br, co, sh, fo, sw

def take_shot(cf, top, t_shot, im, nb_shot, way, arduino):
    if time() - t_shot > cf["tempo"]:
        # Shot
        if way == "right":
            im = cv2.flip(im, 1)
        write_shot(cf, im, nb_shot, way)
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

def write_shot(cf, img, nb_shot, way):
    name = cf["img_dir"] + "/s_" + str(nb_shot) + ".png"
    cv2.imwrite(name, img)
    print(("Shot {1} {0} taken".format(nb_shot, way)))

def laser_D_on_off(arduino, old_set, sw):
    if old_set[0] != sw:
        if sw == 0:
            # Right laser off
            arduino .write('C')
        if sw == 1:
            # Right laser on
            arduino .write('D')

def init_arduino(cf):
    arduino = Arduino(cf)
    # Left Laser on
    arduino.write('G')
    return arduino

def add_lines(im, width, height, cf):
    h = height
    w = width
    ma = cf["motor_axis"]

    # Horizontal lines
    # Middle
    s = int(h/2)
    cv2.line(im, (0, s), (w, s), (0, 0, 0), 1)
    # Motor axis green
    mag = h - ma
    cv2.line(im, (0, mag), (w, mag), (0, 255, 0), 1)

    # Vertical line
    t = int(w/2)
    cv2.line(im, (t, 0), (t, h), (0, 0, 0), 1)

    # Perspective lines
    # Perspective Horizontal set with vertical red line
    table_center_height = 100
    i = int(h - table_center_height + cf["persp_v"])
    cv2.line(im, (0, i), (w, i), (255, 0, 0), 1)
    # Perspective Vertical set with horizontal blue line
    j = int(w/2 - cf["persp_h"])
    cv2.line(im, (j, 0), (j, h), (0, 0, 255), 1)

    # Little lines
    for t in [2, 16, 26]:
        for i in range(-40, 40, 2):
            p = int(w/2 + i*2)
            u = int(t*h/32)
            cv2.line(im, (p,  -5 + u), (p, 5 + u), (0, 0, 0), 1)
        for i in range(-8, 8, 2):
            p = int(w/2 + i*10)
            v = int(t*h/32)
            cv2.line(im, (p, -20 + v), (p, 20 + v), (0, 0, 0), 1)
        for i in range(-4, 5, 2):
            p = int(w/2 + i*20)
            j = int(t*h/32)
            cv2.line(im, (p, -40 + j), (p, 40 + j), (0, 0, 0), 1)
    return im

def nothing(x):
    pass


if __name__=='__main__':
    conf = load_config("./scan.ini")
    cap = Capture(conf)
    cap.set_cam_position()
    #cap.shot()
