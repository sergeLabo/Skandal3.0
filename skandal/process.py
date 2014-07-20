#! /usr/bin/env python
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
http://goo.gl/0mfdQ2
'''


from time import time
import subprocess
import cv2
import numpy as np
from config import load_config
from group import group
from window import Window


GRAYMAX = [("Gray Max", 255, "gray_max")]


class Process():
    '''Compute frame and PLY.'''

    def __init__(self, cf):
        self.cf = cf
        self.persp = 0.2
        self.gap = 0
        # Create empty array 3 x 0 to fill with all points
        self.points = np.asarray([]).reshape(0, 3)
        # Empty array to plit
        self.p_empty = np.asarray([]).reshape(0, 2)
        self.x1, self.x2, self.y1, self.y2, self.w, self.h = 0, 0, 0, 0, 0, 0
        self.get_croped_im_size()
        self.blackim = np.zeros((self.h, self.w, 3), np.uint8)
        self.graymaxWin = Window("Gray max", self.w, self.h, 0.6, GRAYMAX,
                                 self.cf, "scan")
        self.grayWin = Window("Gray image", self.w, self.h, 0.6, None,
                              self.cf, "scan")

    def set_split_on(self):
        self.cf["split"] = True

    def set_split_off(self):
        self.cf["split"] = False

    def crop_image(self, im):
        # Crop from x1, y1 to x2, y2
        imcrop = im[self.y1:self.y2, self.x1:self.x2]
        return imcrop

    def get_croped_im_size(self):
        if not self.cf["inverse"]:
            self.x1 = self.cf["cut_lateral"]
            self.x2 = self.cf["width"] - self.cf["cut_lateral"]
            self.y1 = self.cf["cut_up"]
            self.y2 = self.cf["height"] - self.cf["cut_down"]
            self.w = self.x2 - self.x1
            self.h = self.y2 - self.y1
        else:
            self.x1 = self.cf["cut_lateral"]
            self.x2 = self.cf["height"] - self.cf["cut_lateral"]
            self.y1 = self.cf["cut_up"]
            self.y2 = self.cf["width"] - self.cf["cut_down"]
            self.w = self.x2 - self.x1
            self.h = self.y2 - self.y1
        print("Croped image size = {0} x {1} \n from {2}:{3} to {4}:{5}".\
               format(self.w, self.h, self.x1, self.y1, self.x2, self.y2))

    def lines_image(self, x_array, y_array):
        im = self.blackim.copy()
        for i in range(x_array.shape[0]):
            im.itemset(y_array[i], x_array[i], 1, 255)
            im.itemset(y_array[i], x_array[i], 2, 255)
        self.graymaxWin.display(im, self.cf)

    def get_one_laser_line(self, im_num):
        tframe = time()
        no_image = False
        imgFile = self.cf["img_dir"] + "/s_" + str(im_num) + ".png"
        txtFile = self.cf["txt_dir"] + "/t_" + str(im_num) + ".txt"

        img = cv2.imread(imgFile, 0)  # if no image, return empty matrice
        x, y = 0, 0
        if img != None:  # if img: --> bug
            imcrop = self.crop_image(img)
            self.grayWin.display(imcrop, self.cf)
            white_points, x, y = self.find_white_points_in_gray(imcrop)
            save_points(white_points, txtFile)
            tfinal = int(1000*(time() - tframe))

            print(("Image {0}: {1} points founded in {2} milliseconds".format(
                "/s_" + str(im_num) + ".png", white_points.shape[0], tfinal)))
        else:
            print("No image in {0} project\n".format(self.cf["a_name"]))
            no_image = True
        return x, y, no_image

    def find_white_points_in_gray(self, im):
        # if black image, this default settings return one point at (0, 0)
        x_line = np.array([0.0])
        y_line = np.array([0.0])
        thickness = np.array([0.0])
        # Points beetwin color and 255 are selected
        color = 255 - self.cf["gray_max"]

        #--------------- Don't forget: y origine up left -------------#
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

        return points, x_line, y_line

    def apply_gray_change(self, old_set, new_set, im_num):
        if new_set != old_set:
            self.cf["gray_max"] = new_set
            im_num = -1
            print("Gray = {0}".format(new_set))
        return im_num

    def get_laser_line(self):
        '''Get laser line  in all image,
            save founded points in txt file in txt directory.
        '''
        top = time()
        if self.cf["double"]:
            nb = self.cf["nb_img"] * 2
        else:
            nb = self.cf["nb_img"]

        im_num = -1
        while im_num < nb - 1:
            im_num += 1
            old_set = self.cf["gray_max"]
            # Process image
            x_array, y_array, no_image = self.get_one_laser_line(im_num)
            if no_image:
                break

            # Display laser lines and get gray max from trackbar
            self.lines_image(x_array, y_array)
            new_set = self.graymaxWin.win_set["gray_max"]
            im_num = self.apply_gray_change(old_set, new_set, im_num)
            old_set = new_set

            # wait for ESC key to exit
            k = cv2.waitKey(33)
            if k == 1048603:
                break

        cv2.destroyAllWindows()
        print(("{0} shot calculated in {1} seconds\n".format(im_num,
                                                    int(time() - top))))

    def get_output(self, points, top):
        print("Average", np.mean(points, axis=0))

        # Create string used in ply
        points_str = array_to_str(points)
        write_ply(self.cf["plyFile"], points_str)

        t_total = int(time() - top)
        print(("Compute in {0} seconds\n".format(t_total)))
        print("\n\n  Good job, thank's\n\n")
        if points.shape[0] > 1 and self.cf["meshlab"]:
            open_in_meshlab(self.cf["plyFile"])

    def left_right_diff(self):
        # Correspondence between left and right
        if self.cf["left_first"]:
            d = 0
        else:
            d = int(self.cf["nb_img"] / 2)
        self.gap = d + int(self.cf["ang_rd"] * self.cf["nb_img"] / np.pi)

    def get_points_in_txt(self, index):
        file_txt = self.cf["txt_dir"] + "/t_" + str(index) + ".txt"
        points_LorR, no_txt = load_points(file_txt, self.cf["a_name"])
        if no_txt:
            print("You must process image before process PLY")
            no_txt = True
        return points_LorR, no_txt

    def get_PLY(self):
        # Init
        top = time()
        # Update perspective
        self.get_perspective()
        # Set Left and Right gap
        self.left_right_diff()
        # In test, mesh average
        mesh_L = np.asarray([]).reshape(0, 3)
        mesh_R = np.asarray([]).reshape(0, 3)

        for index in range(self.cf["nb_img"]):
            # Left frame at index
            points_L, no_txt = self.get_points_in_txt(index)
            if no_txt:
                break

            # Laser left and right: double = 1
            if self.cf["double"]:
                # Right frame 50 frame after
                indexR = self.cf["nb_img"] + self.gap + index
                if indexR >= 2 * self.cf["nb_img"]:
                    indexR = indexR - self.cf["nb_img"]
                points_R, no_txt = self.get_points_in_txt(indexR)
                if no_txt:
                    break

                # Compute the two frames
                if not self.cf["split"]:  # split = 0
                    # Indexes must match to concatenate the two matching images
                    print(("Concatenate frame {0} and {1}".format(index,
                                                                    indexR)))
                    self.compute_3D(index, points_L, points_R)
                else:  # split = 1 to get left and right meshes
                    # Left
                    frame_pts = self.compute_3D(index, points_L, self.p_empty)
                    mesh_L = np.append(mesh_L, frame_pts, axis=0)
                    # Right: indexR only to calculated teta
                    i = index + self.cf["nb_img"]
                    frame_pts = self.compute_3D(i, points_R, self.p_empty)
                    mesh_R = np.append(mesh_R, frame_pts, axis=0)

            # If only left laser: double = 0
            else:
                self.compute_3D(index, points_L, self.p_empty)

        if not no_txt:
            print("Mesh Left Average  :", np.mean(mesh_L, axis=0))
            print("Mesh Right Average :", np.mean(mesh_R, axis=0))
            self.get_output(self.points, top)

    def get_perspective(self):
        # coté opposé, coté adjacent
        co = self.cf["motor_axis_v"] - self.cf["persp_v"]
        ca = self.cf["persp_h"] - self.cf["motor_axis_h"]
        if self.cf["test"]:
            co += 0
        self.persp = 0.2  # default value
        if float(ca) != 0.0:  # No 0 div
            self.persp = float(co) / float(ca)

    def compute_3D(self, index, p_L, p_R):
        ''' Compute one frame://\
        From 2D frame points coordinates left and right,
            - add left and right
            - get average
            - compute x y z of this point

        points=nparray(3, points_number)=all 3D points previously calculated
        points_L and points_R = nparray(2, points_number) = points in frame
        index = left frame number//
        Return new points array
        '''

        # Time start in this function
        tframe = time()

        # Angles
        angle_step = float(2 * np.pi) / self.cf["nb_img"]
        sin_cam_ang = np.sin(self.cf["ang_rd"])
        teta = angle_step * index

        # Concatenate and group
        points_new = concatenate_and_group(p_L, p_R)

        # Number of points in frame
        nb = points_new.shape[0]

        # Create empty array to fill with frame points 3D coordinates
        frame_points = np.asarray([]).reshape(0, 3)

        # For all points (x, y)
        for pt in range(nb):
            # x = points_new[pt][0], y = points_new[pt][1]
            AM = self.w/2 - points_new[pt][0]

            # From point in frame, get world coordinates
            point = (AM, sin_cam_ang, points_new[pt][1], teta)
            frame_points = self.get_world_coord(point, frame_points)

        self.points = np.append(self.points, frame_points, axis=0)

        tfinal = int(1000*(time() - tframe))
        print(("Frame {0} compute in {1} milliseconds, {2} points founded".\
                            format(index, tfinal, frame_points.shape[0])))
        return frame_points

    def get_world_coord(self, point, frame_points):
        (AM, sin_cam_ang, y, teta) = point

        # Bidouille intuitive
        # Correction because cube face are curved
        AM = AM - float(AM * AM) / 1720.0

        # Point position from turn table center
        OM = AM / sin_cam_ang

        # Height
        FM = self.h - y
        v = (self.h / 2) - y
        a0 = - 2 * self.persp / self.h
        tg_beta = a0 * v
        OG = FM + AM * tg_beta
        # Mesh Cleaning
        mini = self.cf["mini"]
        maxi = self.h - self.cf["maxi"]

        # Only to test with white plate
        if self.cf["test"]:
            mini = self.cf["mini"] - 10
            if OM < 0:
                OM = 0

        if mini < OG < maxi:
            # Changement de repère orthonormé
            x = np.cos(teta) * OM * self.cf["scale"]
            y = np.sin(teta) * OM * self.cf["scale"]
            z = OG * self.cf["scale"] * self.cf["z_scale"]
            # Add this point
            frame_points = np.append(frame_points, [[x, y, z]], axis=0)
        return frame_points


def load_points(file_X, project_name):
    try:
        points = np.loadtxt(file_X)
        no_txt = False
    except:
        print("No {0} txt file\n".format(project_name))
        points = None
        no_txt = True
    return points, no_txt

def open_in_meshlab(ply):
    '''http://sourceforge.net/p/meshlab/bugs/238/
    problème de locale:
    meshlab ne prend pas les "," comme des "."
    LANG=C ouvre en anglais
    '''
    subprocess.call('LANG=C;meshlab {0}'.format(ply), shell=True)

def concatenate_and_group(points_L, points_R):
    # If only one point, shape=(2,) not (1, 2)
    # I replace this point with array (1, 2) fill with 0
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

    return points_new

def array_to_str(p_array):
    ''' From array(a lot x 3) return a string to create ply. '''
    list_of_str = p_array.tolist()
    p_str = []
    for s in list_of_str:
        point = str(s[0]) + " " + str(s[1]) + " " + str(s[2]) + "\n"
        p_str.append(point)
    return p_str

def write_ply(ply_file, points_str):
    '''points = list of string: "1 0 5\n" '''
    header = '''ply
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
'''
    file = open(ply_file, "w")
    file.write(header.format(len(points_str), "".join(points_str)))
    file.close()
    print(("\nSaved {0} points to:\n     {1}\n".format(len(points_str),
                                                            ply_file)))

def save_points(points, file_name):
    np.savetxt(file_name, points, fmt="%i", delimiter=' ')


if __name__ == '__main__':
    conf = load_config("./scan.ini")
    proc = Process(conf)
    ##proc.get_laser_line()
    ##img = cv2.imread('skandal.png', 0)
    ##cv2.imshow('img', img)
    ##cv2.waitKey(100)
    ##cv2.destroyAllWindows()
    proc.get_PLY()
