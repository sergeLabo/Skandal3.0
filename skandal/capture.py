#! /usr/bin/env python
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
        create_trackbar_gray()
        # set = 0 br, 1 co, 2 sh, 3 fo, 4 sw, 5 ph, 6 pv, 7 mah, 8 mav, 9 rot
        old_set = set_init_trackbar(self.cf)

        while True:
            ret, frame = self.capture.read()
            if ret:
                # Display original image
                frame = display_raw(frame, self.cf)
                # Display grayscale image
                br, co, sh, fo, sw, ph, pv, mah, mav, rot = display_gray(frame,
                                                                    self.cf)
                new_set = br, co, sh, fo, sw, ph, pv, mah, mav, rot
                # Apply change and update conf
                self.cf = apply_conf_change(self.cf, old_set, new_set)
                # Update laser
                laser_L_or_R(self.arduino, old_set, sw)
                old_set = new_set
                # wait for esc key to exit
                key = np.int16(cv2.waitKey(33))
                if key == 27:
                    break
            else:
                print("Webcam is busy\nor ")
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
                                break
                    if way == "right":
                        set_laser(way, self.arduino)
                        # Shot every second until shot number
                        if nb_shot < self.steps * 2:
                            nb_shot, t_shot = take_shot(self.cf, top, t_shot,
                                frame, nb_shot, way, self.arduino)
                        else:
                            break
                                # TODO: Bug if resize in display_raw
                # Display
                frame = display_raw(frame, self.cf)
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


def apply_conf_change(cf, old_set, new_set):
    # set = 0 br, 1 co, 2 sh, 3 fo, 4 sw, 5 ph, 6 pv, 7 mah, 8 mav, 9 rot
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
        cf["motor_axis_h"] = new_set[7]
        save_config("scan", "motor_axis_h", new_set[7])
    if old_set[8] != new_set[8]:
        cf["motor_axis_v"] = new_set[8]
        save_config("scan", "motor_axis_v", new_set[8])
    if old_set[9] != new_set[9]:
        cf["rotate"] = new_set[9]
        save_config("scan", "rotate", new_set[9])
    return cf

def create_trackbar_gray():
    cv2.namedWindow("Gray Scale")
    cv2.createTrackbar("Brightness", "Gray Scale", 30, 255, nothing)
    cv2.createTrackbar("Contrast", "Gray Scale", 0, 10, nothing)
    cv2.createTrackbar("Sharpness", "Gray Scale", 0, 50, nothing)
    cv2.createTrackbar("Focus (absolute)", "Gray Scale", 0, 40, nothing)
    switch = '0: Left Laser \n1: Right laser'
    cv2.createTrackbar(switch, "Gray Scale", 0, 1, nothing)
    cv2.createTrackbar("Motor Axis H", "Gray Scale", 0, 30, nothing)
    cv2.createTrackbar("Motor Axis V", "Gray Scale", 0, 200, nothing)
    cv2.createTrackbar("Perspective H", "Gray Scale", 0, 400, nothing)
    cv2.createTrackbar("Perspective V", "Gray Scale", 0, 100, nothing)
    cv2.createTrackbar("Rotate", "Gray Scale", 0, 100, nothing)

def set_init_trackbar(cf):
    ph = cf["persp_h"]
    pv = cf["persp_v"]
    mav = cf["motor_axis_v"]
    mah = cf["motor_axis_h"]
    rot = cf["rotate"]
    cv2.setTrackbarPos("Perspective H", "Gray Scale", ph)
    cv2.setTrackbarPos("Perspective V", "Gray Scale", pv)
    cv2.setTrackbarPos("Motor Axis H", "Gray Scale", mah)
    cv2.setTrackbarPos("Motor Axis V", "Gray Scale", mav)
    cv2.setTrackbarPos("Rotate", "Gray Scale", rot)
    br = cf["brightness"]
    co = cf["contrast"]
    sh = cf["sharpness"]
    fo = cf["focus_abs"]
    sw = 0
    cv2.setTrackbarPos("Brightness", "Gray Scale", br)
    cv2.setTrackbarPos("Contrast", "Gray Scale", co)
    cv2.setTrackbarPos("Sharpness", "Gray Scale", sh)
    cv2.setTrackbarPos("Focus (absolute)", "Gray Scale", fo)
    return br, co, sh, fo, sw, ph, pv, mah, mav, rot

def im_size(cf):
    '''Multiplicator with cam and screen size.
        TODO
    '''
    # Default value
    kx = 1
    ky = 1
    return kx, ky

def display_raw(im, cf):
    kx, ky = im_size(cf)
    W = int(cf["width"] * kx)
    H = int(cf["height"] * ky)
    im = cv2.resize(im, (W, H))

    # Rotate
    rot = (float(cf["rotate"]) - 50) / 50
    # Get the dimensions of the image and calculate the center of the image
    (h, w) = im.shape[:2]
    center = (int(w / 2), int(h / 2))

    # rotate the image by rot degrees
    M = cv2.getRotationMatrix2D(center, rot, 1.0) # center = x,y angle, scale
    rotated = cv2.warpAffine(im, M, (w, h))

    # Add lines
    rotated = add_lines(rotated, cf)

    # Display
    cv2.imshow("Raw Webcam", rotated)

    return rotated

def display_gray(im, cf):
    width, height = cf["width"], cf["height"]
    # get current positions of trackbars
    br = cv2.getTrackbarPos("Brightness", "Gray Scale")
    co = cv2.getTrackbarPos("Contrast", "Gray Scale")
    sh = cv2.getTrackbarPos("Sharpness", "Gray Scale")
    fo = cv2.getTrackbarPos("Focus (absolute)", "Gray Scale")
    switch = '0: Left Laser \n1: Right laser'
    sw = cv2.getTrackbarPos(switch, "Gray Scale")

    # get current positions of trackbars for line in raw
    ph = cv2.getTrackbarPos("Perspective H", "Gray Scale")
    pv = cv2.getTrackbarPos("Perspective V", "Gray Scale")
    mah = cv2.getTrackbarPos("Motor Axis H", "Gray Scale")
    mav = cv2.getTrackbarPos("Motor Axis V", "Gray Scale")
    rot = cv2.getTrackbarPos("Rotate", "Gray Scale")

    # Resize
    if width < 1200:
        k = 1
    else:
        k = 0.8
    W = int(width * k)
    H = int(height * k * 0.5)  # To create places for trackbar
    im = cv2.resize(im, (W, H))

    # Convert
    im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)

    # Display
    cv2.imshow("Gray Scale", im)

    return br, co, sh, fo, sw, ph, pv, mah, mav, rot

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

def laser_L_or_R(arduino, old_set, sw):
    if old_set[4] != sw:
        if sw == 0:
            # Left on
            arduino .write('G')
            arduino .write('C')
        if sw == 1:
            # Right on
            arduino .write('D')
            arduino .write('B')

def init_arduino(cf):
    arduino = Arduino(cf)
    # Left Laser on
    arduino.write('G')
    return arduino

def add_lines(im, cf):
    h = cf["height"]
    w = cf["width"]
    mav = cf["motor_axis_v"]
    mah = cf["motor_axis_h"]
    pv = cf["persp_v"]
    ph = cf["persp_h"]
    # Horizontal lines
    # Middle
    s = int(h/2)
    cv2.line(im, (0, s), (w, s), (0, 0, 0), 1)

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

    # Middle vertical line
    t = int(w/2)
    cv2.line(im, (t, 0), (t, h), (0, 0, 0), 1)

    # Little lines
    # Up and down
    for t in [2, 26]:
        # Small
        for i in range(-40, 40, 2):
            p = int(w/2 + i*2)
            u = int(t*h/32)
            cv2.line(im, (p,  -5 + u), (p, 5 + u), (0, 0, 0), 1)
        # Middle
        for i in range(-8, 8, 2):
            p = int(w/2 + i*10)
            v = int(t*h/32)
            cv2.line(im, (p, -20 + v), (p, 20 + v), (0, 0, 0), 1)
        # Hight
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
    cap.shot()
