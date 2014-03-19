#! /usr/bin/env python3
# -*- coding: utf-8 -*-

# cam_settings.py

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

'''
TODO les keys sont utilisées dans plusieurs autres fonctions
tout doit être géré iciet avec scan.ini
'''

def apply_all_cam_settings(cf):
    ''' Set cam with all possible parameter: See wiki.'''

    # It's a list to disable auto settings first
    all_param = [["Focus, Auto", "focus_auto"],
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

    # Possible param with this cam
    possible = []
    for prm in cf["cam_items"]:
        possible.append(prm[0])

    # System command line: uvcdynctrl
    for item in all_param:
        if item[1] in possible:
            subprocess.call('uvcdynctrl -s "{0}" {1}'.format(item[0],
                                            cf[item[1]]), shell=True)
            print("Cam settings: {0} = {1}".format(item[0], cf[item[1]]))
            sleep(0.1)

def apply_cam_setting(name, value):
    '''Apply only one setting on cam.'''
    subprocess.call('uvcdynctrl -s "{0}" {1}'.format(name, value), shell=True)
    print("Cam settings: {0} = {1}".format(name, value))

if __name__=='__main__':
    cf = load_config("./scan.ini")
    apply_all_cam_settings(cf)
    apply_cam_setting("Brightness", 33)
