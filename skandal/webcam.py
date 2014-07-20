#! /usr/bin/env python
# -*- coding: utf-8 -*-

# webcam.py

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
from time import sleep
from config import load_config


CAM_PARAM = [["Focus, Auto", "focus_auto"],
            ["Focus (absolute)", "focus_abs"],
            ["White Balance Temperature, Auto", "w_bal_temp_aut"],
            ["Exposure, Auto", "expos_auto"],
            ["White Balance Temperature", "white_bal_temp"],
            ["Exposure (Absolute)", "exposure_abs"],
            ["Power Line Frequency", "power_line_freq"],
            ["Brightness", "brightness"],
            ["Contrast", "contrast"],
            ["Sharpness", "sharpness"],
            ["Backlight Compensation", "backlight_comp"],
            ["Saturation", "saturation"],
            ["Pan (Absolute)", "pan_abs"],
            ["Tilt (Absolute)", "tilt_abs"],
            ["Zoom, Absolute", "zoom_abs"]]

def apply_all_cam_settings(cf):
    '''Apply all settings on cam.'''
    # Device number = str !
    cam = cf["cam"]
    # List of all cam settings, disable auto settings first
    # An item is a tuple, ("uvcdynctrl parameter name", "variable in scan.ini")
    for item in CAM_PARAM:
        subprocess.call('uvcdynctrl -d video"{0}" -s  "{1}" {2}'.format(
                            cam, item[0], cf[item[1]]), shell=True)
        print("Cam{0} settings: {1} = {2}".format(cam, item[0], cf[item[1]]))
        sleep(0.1)

def apply_cam_setting(cam, name, value):
    '''Apply only one setting on cam.'''
    subprocess.call('uvcdynctrl -d video"{0}" -s  "{1}" {2}'.format(cam,
                                                name, value), shell=True)
    print("Cam{0} settings: {1} = {2}".format(cam, name, value))

if __name__ == '__main__':
    cf = load_config("./scan.ini")
    apply_all_cam_settings(cf)
    apply_cam_setting(0, "Brightness", 33)
