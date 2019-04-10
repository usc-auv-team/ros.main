#!/usr/bin/env python
"""
Helper functions to allow cv_inference_dist to calculate the distance to an object
"""

def get_calibration_values(path):
    # will be updated later to fit more calibration values
    f = open(path + os.sep + "calibration.txt", "r")
    return float(f.read())

def distance_to_camera(knownWidth, focalLength, perWidth):
    # compute the distance from the marker to the camera
    return (knownWidth * focalLength) / perWidth

def calibrate(pixelWidth):
    print("calibrating")
    print("vvvvvvvvvvvvvvv")
    # info about buoy
    # known_height_mm = 189.44
    # known_width_mm = 279.89
    known_height_m = 0.18944
    known_width_m = 0.27989
    # known_height_in = 7.45826772
    # known_width_in = 11.0192913
    # for now, we're using meters and buoys
    KNOWN_DISTANCE = 0.5
    KNOWN_HEIGHT = known_height_m
    KNOWN_WIDTH = known_width_m
    focalLength = (pixelWidth * KNOWN_DISTANCE) / KNOWN_WIDTH
    f = open(path + os.sep + "calibration.txt", "w+")
    # f.write("buoy ")
    f.write(str(focalLength))
