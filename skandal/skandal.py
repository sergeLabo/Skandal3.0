#! /usr/bin/env python3
# -*- coding: utf-8 -*-

# skandal.py

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


import os, sys
import threading
from config import load_config, get_available_name, save_config
from capture import Capture
from process import Process


class Skandal():
    '''Change config, capture, process.'''
    def __init__(self):
        self.cf = load_config("./scan.ini")
        self.cap = Capture(self.cf)
        self.proc = Process(self.cf)

    def change_config(self):
        pass

    def set_cam_position(self):
        self.cap.set_cam_position()

    def shot(self):
        self.cap.shot()

    def process_images(self):
        self.proc.get_laser_line()

    def process_PLY(self):
        self.proc.get_PLY()

    def scan(self):
        self.shot()
        self.process_images()
        self.process_PLY()


def clear():
    if (os.name == 'nt'):
        c = os.system('cls')
    else:
        c = os.system('clear')

def menu_terminal():
    conf = load_config("./scan.ini")
    skandal = Skandal()
    name_input(conf)
    while True:
        clear()
        print("Your project is {0}".format(conf["name"]))
        menu = """

        1. Set Camera Position

        2. Shot

        3. Process Images to get laser line

        4. Process PLY to get *.ply file

        5. Scan

        0. Exit
        """
        print(("{0}".format(menu)))

        is_valid = False
        while not is_valid :
            try :
                choice = int(input('Enter your choice [0-6] : '))
                is_valid = True
            except ValueError as e :
                print("This is not a valid integer.")
                choice = None
            if choice == 1:
                skandal.set_cam_position()
            elif choice == 2:
                skandal.shot()
            elif choice == 3:
                skandal.process_images()
            elif choice == 4:
                skandal.process_PLY()
            elif choice == 5:
                skandal.scan()
            elif choice == 0:
                sys.exit(0)
            else:
                is_valid = False
                print("Invalid number. Try again...")

def name_input(cf):
    clear()
    is_valid = 0
    while not is_valid :
        try :
            choice = input('\n\n  Enter your project name :  ')
            is_valid = 1
        except ValueError as e :
            print(("'%s' is not a valid name." % e.args[0].split(": ")[1]))
    name = get_available_name(choice)
    print("Your project name is {0}".format)
    cf["name"] = name
    save_config("scan", "name", name)
    # Create directory, variable with this name
    conf = load_config("./scan.ini")

if __name__=='__main__':
    menu_terminal()

