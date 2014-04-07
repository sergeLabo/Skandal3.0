#! /usr/bin/env python3
# -*- coding: utf-8 -*-

# config.py

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

import os
import ast
from configparser import SafeConfigParser
import subprocess
import numpy as np

def load_config(ini):
    ''' Read *.ini file. Create dict with all parameter.
        ini = "./scan.ini" in this project.
    '''

    parser = SafeConfigParser()
    parser.read(ini)
    conf = {}

    # Scan config
    for key in parser.items('scan'):
        conf[key[0]] = ast.literal_eval(key[1])

    # Cam config
    conf["cam_items"] = parser.items(conf["webcam"])
    for key in conf["cam_items"]:
        conf[key[0]] = ast.literal_eval(key[1])

    # Create correct name, 12 characters maxi
    conf["a_name"] = get_available_name(conf["name"])

    # Create directory
    Skandal, img_dir, txt_dir ,ply_dir, stl_dir = \
                                    directory_management(conf["a_name"])
    conf["img_dir"] = img_dir
    conf["txt_dir"] = txt_dir
    conf["ply_dir"] = ply_dir
    conf["plyFile"] = ply_dir + "/m_" + conf["a_name"] + ".ply"
    conf["stl_dir"] = stl_dir
    conf["stlFile"] = stl_dir + "/m_" + conf["a_name"] + ".stl"
    conf["mlxFile"] = Skandal + "/skandal/skandal.mlx"

    # Get existing project list, id available image in image directory
    image_file = img_dir
    conf["proj_list"] = get_project_list(image_file)

    # Angle in radians
    conf["ang_rd"] = float(conf["angle"] * np.pi) / 180

    print("Configuration from scan.ini loaded.\n")
    return conf

def get_available_name(name):
    '''From raw name, get a name available to file name.'''
    name = name.encode('ascii', errors='ignore')
    # TODO revoirpour accepter nombre si pas en premier
    name = str(name)[1:]
    name = "".join(c for c in name if c.isalpha())
    name = name[:12]
    return name

def get_project_list(image_file):
    '''Get all project list available.'''
    proj_list = []
    for key in os.listdir(image_file):
        proj_list.append(key)
    return proj_list

def directory_management(name):
    ''' Create all needed directories if not. Return this directories.'''
    # TODO: what append if a creative guy change the name config.py
    Skandal = os.path.dirname(os.getcwd()+"config.py")
    print("\n\nDirectories:\n")
    print("Master directory is {}".format(Skandal))
    # Working directory
    try:
        os.mkdir(Skandal + "/work")
    except:
        print("Directory {0} exist".format(Skandal + "/work"))
        pass
    # Master directories
    all_dir = [ "/image",
                "/ply",
                "/stl",
                "/txt",
                "/blend"]
    for d in all_dir:
        try:
            os.mkdir(Skandal + "/work" + d)
        except:
            print("Directory {0} exist".format(Skandal + "/work" + d))
            pass

    # Sub-directories
    for d in ["/image/","/txt/"]:
        sub_dir = Skandal + "/work" + d + "p_" + str(name)
        try:
            os.mkdir(sub_dir)
        except:
            print("Directory {0} exist".format(sub_dir))
            pass

    img_dir = Skandal + "/work" + "/image/p_" + name
    txt_dir = Skandal + "/work" + "/txt/p_" + name
    ply_dir = Skandal + "/work" + "/ply"
    stl_dir = Skandal + "/work" + "/stl"
    print("\n\n\n  Directories for this work:\n")
    print("image: {0}".format(img_dir))
    print("txt: {0}".format(txt_dir))
    print("ply: {0}\n".format(ply_dir))
    print("stl: {0}\n".format(stl_dir))

    return Skandal, img_dir, txt_dir ,ply_dir, stl_dir

def save_config(section, key, value):
    '''Save config in *.ini file with section, key, value.'''
    if isinstance(value, int):
        val = str(value)
    if isinstance(value, float):
        val = str(value)
    if isinstance(value, str):
        val = '"' + value + '"'

    config = SafeConfigParser()
    config.read("./scan.ini")
    config.set(section, key, val)
    with open("./scan.ini", 'w') as f:
       config.write(f)
    f.close()
    print("\n{1} = {2} saved in scan.ini in section {0}\n".format(section,
                                                                key, val))


if __name__=='__main__':
    conf = load_config("./scan.ini")
