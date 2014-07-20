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
    def __init__(self, device):
        self.device = device
        try:
            self.arduino = serial.Serial(self.device, 9600)
            print("\nInitialising Arduino device...")
            sleep(2)
            print("Done\n")
        except:
            self.arduino = None
            print("\nNo Arduino device plugged but you can test !")
            print("Or you must add your user to dialout:\n")
            print("   sudo usermod -a -G dialout your_user")
            print("and restart\n")

    def write(self, char):
        if self.arduino:
            if char in ['H', 'L', 'G', 'D', 'C', 'B']:
                self.arduino.write(str(char).encode())
            else:
                print(("{0} isn't in list of available characters".format(
                      char)))
        else:
            print("Test without arduino card")

    def close(self):
        if self.arduino:
            self.arduino.close()
            print("Arduino closed")


if __name__ == '__main__':
    conf = load_config("./scan.ini")
    device = conf["ard_dev"]
    arduino = Arduino(device)
    for i in range(200):
        sleep(1)
        print(i)
        arduino.write("H")
    arduino.close()
