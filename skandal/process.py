#! /usr/bin/env python3
# -*- coding: utf-8 -*-

# process.py

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

'''
Fast finding pixel position with current color interval from:
http://stackoverflow.com/questions/21147259/fast-finding-pixel-position-with-current-color-interval
'''


import os
from time import time
import subprocess
import glob
import cv2
import numpy as np
from config import load_config, save_config
from group import group


class Process():
    '''Compute frame and PLY'''
    def __init__(self, cf):
        self.cf = cf
        self.width = self.cf["width"]
        self.height = self.cf["height"]
        self.steps = self.cf["nb_img"]
        self.double = self.cf["double"]
        self.plot = self.cf["plot"]
        self.split = self.cf["split"]
        self.angle = self.cf["ang_rd"] # Radians
        self.gray_max = self.cf["gray_max"]
        self.ax = None
        self.sgray = None

    def get_laser_line(self):
        '''Get laser line  in all image,
        save founded points in txt file in txt directory.
        '''

        top = time()
        if self.double:
            nb = self.steps * 2
        else:
            nb = self.steps

        # Create a black image
        blackim = np.zeros((self.height, self.width, 3), np.uint8)
        # Empty queue
        queue = np.zeros((1, 2))

        cv2.namedWindow('Laser Line')
        create_trackbar_laser_line(self.cf)
        current_set = self.cf["gray_max"], 0

        im_num = -1
        while im_num < nb - 1:
            im_num += 1
            # Process image
            x_array, y_array = get_one_laser_line(self.cf, im_num)
            # Display lines
            im, queue = lines_image(x_array, y_array, queue, blackim)
            self.gray_max, sw = display_laser_line(self.cf, im)
            im_num = apply_gray_change(self.cf, current_set, self.gray_max,
                                                                sw, im_num)
            # wait for ESC key to exit
            k = cv2.waitKey(33)
            if k == 1048603:
                break

        cv2.destroyAllWindows()
        print(("{0} shot calculated in {1} seconds".format(nb,
                                                    int(time() - top))))

    def get_PLY(self):
        ''' Read all txt file, create volume and write PLY in ply directory.'''
        top = time()
        # Create empty array 3 x 0 to fill with all points
        points = np.asarray([]).reshape(0, 3)

        # Correspondance between left and right
        decal = int(self.angle * self.steps / np.pi)

        for index in range(self.steps):
            # Left frame at index
            file_L = self.cf["txt_dir"] + "/t_" +  str(index) + ".txt"
            points_L = np.loadtxt(file_L)
            if points_L == None:
                print("No {0} file".format(file_L))
            # Empty array
            p_empty = np.asarray([]).reshape(0, 2)
            # two run
            if self.double:
                # Right frame 50 frame after
                indexR = self.steps + decal + index
                if indexR >= 2*self.steps:
                    indexR = indexR - self.steps
                file_R = self.cf["txt_dir"] + "/t_" +  str(indexR) + ".txt"
                # Erase first
                points_R = np.loadtxt(file_R)
                if points_R == None:
                    print("No {0} file".format(file_R))

                # Compute the two frames
                if not self.split:
                    # Index is very important
                    # to concatenate the two matching images
                    print(("Concatenate frame {0} and {1}".format(index,
                                                                    indexR)))
                    points = compute_3D(self.cf, index, points_L, points_R,
                                                                    points)
                if self.split:
                    points = compute_3D(self.cf, index, points_L, p_empty,
                                                                    points)
                    # indexR only to calculated teta,
                    # 1/4 of circumference before (with angle =45°)
                    points = compute_3D(self.cf, index, points_R, p_empty,
                                                                    points)
            # one run
            else:
                points = compute_3D(self.cf, index, points_L, p_empty, points)

        # Create string used in ply
        points_str = array_to_str(points)
        write_ply(self.cf["plyFile"], points_str)

        t_total = int(time() - top)
        print(("Compute in {0} seconds".format(t_total)))
        print("\n\n  Good job, thank's\n\n")
        try:
            subprocess.call('meshlab {0}'.format(self.cf["plyFile"]),
                                                                shell=True)
        except:
            print("You must install meshlab,")
            print("sudo apt-get install meshlab")

def lines_image(x_array, y_array, queue, blackim):
    im = blackim.copy()
    for i in range(x_array.shape[0]):
        im.itemset(y_array[i], x_array[i], 1, 255)
        im.itemset(y_array[i], x_array[i], 2, 255)
    return im, queue

def apply_gray_change(cf, current_set, gray_max, sw, im_num):
    if gray_max != current_set[0]:
        save_config("scan", "gray_max", gray_max)
        cf["gray_max"] = gray_max
    if sw != current_set[1]:
        im_num = -1
    cv2.setTrackbarPos('Restart', 'Laser Line', 0)
    return im_num

def create_trackbar_laser_line(cf):
    cv2.createTrackbar('Gray Max', 'Laser Line', 1, 254, nothing)
    cv2.createTrackbar('Restart', 'Laser Line', 0, 1, nothing)
    gray_max = cf["gray_max"]
    cv2.setTrackbarPos('Gray Max', 'Laser Line', gray_max)

def display_laser_line(cf, im):
    # get current positions of trackbars
    gray_max = cv2.getTrackbarPos('Gray Max', 'Laser Line')
    sw = cv2.getTrackbarPos('Restart', 'Laser Line')
    # TODO set k with width
    width = int(cf["width"]*0.8)
    height = int(cf["height"]*0.8)
    im = cv2.resize(im, (width, height))
    cv2.imshow('Laser Line', im)
    #cv2.moveWindow('Laser Line', 200, 30)
    return gray_max, sw

def compute_3D(cf, index, points_L, points_R, points):
    ''' Compute one frame:
    From 2D frame points coordinates left and right,
        - add left and right
        - get average
        - compute x y z of this point
    See sheme at:
      points = nparray(3, points_number) = all 3D points previously calculated
      points_L and points_R = nparray(2, points_number) = points in frame
      index = right frame number
    Return new points array
    '''

    # Time start in this function
    tframe = time()
    ############################ TODO j'aime pas!
    height = cf["height"]
    width = cf["width"]
    scale = cf["scale"]
    # Points under mini and upper maxi are deleted
    mini = cf["z_down"]
    maxi = height - cf["z_up"]
    z_scale = cf["z_scale"]
    # Angles
    step = cf["nb_img"]
    angle_step = 2 * np.pi / step
    alpha = cf["ang_rd"]
    sin_cam_ang = np.sin(alpha)
    teta = angle_step * index
    # Perspective correction
    ph = cf["persp_h"]
    pv = cf["persp_v"]
    tg_alpha = 0.2
    if ph != 0:
        tg_alpha = pv / ph
    print("Perspective: Tangente alpha = {0}".format(tg_alpha))
    ############################

    # Create empty array to fill with frame points 3D coordinates
    frame_points = np.asarray([]).reshape(0, 3)

    # Problem if only one point
    # TODO if one real point, it will be replace with 0, 0
    if points_L.shape == (2,):
        points_L = np.zeros((1, 2))
    if points_R.shape == (2,):
        points_R = np.zeros((1, 2))

    # Concatenate points_L and points_R
    points_LR = np.concatenate((points_L, points_R))

    # Group points on the same b with point[b, a]
    y_line, x_line, thickness = group(points_LR, flip=True)

    # x and y array in one array
    points_new = np.transpose(np.vstack((x_line, y_line)))

    # Number of points in frame
    nb = points_new.shape[0]

    # For all points (x, y) in txt file
    for pt in range(nb):
        # a_raw, b_raw = point coordinates in image with origine up, left
        a_raw = points_LR[pt][0]
        b_raw = points_LR[pt][1]

        AM = width/2 - a_raw

        if -300 < AM < 300: # Delete background laser line
            # Point position from turn table center
            OM = AM / sin_cam_ang
            FM = height - b_raw
            v = (height / 2) - b_raw
            a0 = -2* tg_alpha / height
            tg_beta = a0 * v
            OG = FM + AM * tg_beta

            if mini < OG  < maxi:
                # Changement de repère orthonormé
                x = np.cos(alpha - teta) * OM * scale
                y = np.sin(alpha - teta) * OM * scale
                z = OG * scale * z_scale

                # Add this point
                frame_points = np.append(frame_points, [[x, y, z]], axis=0)

    points = np.append(points, frame_points, axis=0)
    tfinal = int(1000*(time() - tframe))
    print(("Frame {0} compute in {1} milliseconds, {2} points founded".format(\
                    index, tfinal, frame_points.shape[0])))
    return points

def array_to_str(p_array):
    ''' From array(a lot x 3) return a string to create ply. '''
    n = p_array.shape[0]
    list_of_str = p_array.tolist()
    p_str = []
    for s in list_of_str:
        point = str(s[0]) + " " + str(s[1]) + " " + str(s[2]) +"\n"
        p_str.append(point)
    return p_str

def write_ply(ply_file, points_str):
    '''points = list of string: "1 0 5\n" '''
    file = open(ply_file,"w")
    file.write('''ply
format ascii 1.0
comment author: Skandal
element vertex {0}
property float x
property float y
property float z
element face 0
property list uchar int vertex_index
element edge 0
property int vertex1
property int vertex2
property uchar red
property uchar green
property uchar blue
end_header
{1}
'''.format(len(points_str), "".join(points_str)))

    file.close()
    print(("\n\nSaved {0} points to:\n     {1}\n".format(len(points_str),
                                                            ply_file)))

def get_one_laser_line(cf, im_num):
    tframe = time()
    imgFile = cf["img_dir"] + "/s_" + str(im_num) + ".png"
    txtFile = cf["txt_dir"] + "/t_" + str(im_num) + ".txt"

    img = cv2.imread(imgFile)
    x, y = 0, 0
    if img != None:
        white_points, x , y = find_white_points_in_gray(cf, img)
        save_points(white_points, txtFile)
        tfinal = int(1000*(time() - tframe))

        print(("Image {0}: {1} points founded in {2} milliseconds".\
        format("/s_" + str(im_num) + ".png", white_points.shape[0], tfinal)))
    else:
        print("No image in {0} project".format(cf["a_name"]))

    return x, y

def find_white_points_in_gray(cf, im):
    # if black image, this default settings return one point at (0, 0)
    x_line = np.array([0.0])
    y_line  = np.array([0.0])
    thickness = np.array([0.0])
    color = 255 - cf["gray_max"]

    #--------------- Don't forget: y origine up left -------------#
    im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    # All pixels table: filled with 0 if black, 255 if white
    all_pixels_table = cv2.inRange(im, color, 255)

    # Get a tuple(x array, y array) where points exists
    white_points = np.where(all_pixels_table > 0)

    # Convert tuple of 2 arrays to one array
    white_points = np.transpose(white_points)  # (8, 2)

    # Line function y=f(x) isn't reflexive, id with one x, multiple y
    # For y from 0 to height, find all x of white pixel and get x average
    # group() is called only if white pixel in image
    # and white_points_xy.shape = (1, 2) minimum
    if white_points.shape[1] == 2:
        if white_points.shape[0] != 0:
            y_line, x_line, thickness = group(white_points, flip=False)
            # y_line = [0 1 3 4 5], x_line = [3 4 2 2 0]

    # x and y array in one array
    points = np.transpose(np.vstack((x_line, y_line)))

    # x_line, y_line only to plot
    return points, x_line, y_line

def save_points(points, file_name):
    np.savetxt(file_name, points, fmt="%i", delimiter=' ')

def nothing(x):
    pass


if __name__=='__main__':
    conf = load_config("./scan.ini")
    proc = Process(conf)
    proc.get_laser_line()
    proc.get_PLY()
