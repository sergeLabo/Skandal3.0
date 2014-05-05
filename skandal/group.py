#! /usr/bin/env PYTHON
# -*- coding: utf-8 -*-

# group.py

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
From:
http://stackoverflow.com/questions/7790611/average-duplicate-values-from-two-paired-lists-in-python-using-numpy
'''

import numpy as np

def group(table, flip=False):
    """
    TODO: clear and explain this function

    table is a np.array dimension 2 x n
    Group the values by key
    Returns:
        nparray of unique keys
        nparray of the per-key average
        nparray of the keycounts per key = thickness.
    """
    # Return always something
    x = np.array([0])
    y = np.array([0])
    thickness = np.array([0])

    # Create 2 numpy array
    if not flip:
        keys = table[:,0]
        values = table[:,1]
    else:
        keys = table[:,1]
        values = table[:,0]

    # Create array of keys index: retur example [0, 1, 2, 3, 4]
    keys_index = np.argsort(keys)

    # Create array of keys table, sorted with keys_index
    keys = keys[keys_index]

    # Create list of values table, sorted with keys_index
    values = values[keys_index]

    # The slicing points of the bins to sum over
    slices = np.concatenate(([0], np.where(keys[:-1]!=keys[1:])[0]+1))

    # First entry of each bin is a unique key
    unique_keys = keys[slices]

    # Sum over the slices specified by index
    per_key_sum = np.add.reduceat(values, slices)

    # Number of counts per key is the difference of our slice points.
    # Cap off with number of keys for last bin
    key_count = np.diff(np.append(slices, len(keys)))
    try:
        per_key_average = per_key_sum / key_count
    except:
        print("Zero division in group function !")
        per_key_average = np.zeros(len(keys_index))

    # x, y are separate array
    x = unique_keys
    y = per_key_average
    thickness = key_count

    return x, y, thickness
