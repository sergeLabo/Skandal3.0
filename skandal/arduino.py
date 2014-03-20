#! /usr/bin/env python
# -*- coding: utf-8 -*-

# arduino.py

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


from time import sleep
import serial
from config import load_config


class Arduino():
    '''Manage socket to Arduino card.'''
    def __init__(self, conf):
        self.device = conf["ard_dev"]
        try:
            self.arduino = serial.Serial(self.device, 9600)
            print("\nInitialising Arduino device...\n")
            sleep(2)
        except:
            self.arduino = None
            print("\nNo Arduino device plugged but you can test !\n")

    def write(self, char):
        if self.arduino:
            if char in ['H', 'L', 'G', 'D', 'C', 'B']:
                self.arduino.write(str(char).encode())
            else:
                print(("{0} isn't in list of available characters".format(char)))
        else:
            print("Test without arduino card")

    def close(self):
        if self.arduino:
            self.arduino.close()
            print("Arduino closed")


if __name__=='__main__':
    conf = load_config("./scan.ini")
    arduino = Arduino(conf)
    for i in range(2):
        sleep(0.1)
        print(i)
        arduino.write("H")
    arduino.close()
