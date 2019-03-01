"""Angle Calculation Function
    constants:
        horizontal FOV: 80 degrees
        vertical FOV: 63 degrees
    formula:
        ((x - (totalWidth/2))/totalWidth ) * FOV
"""
#FOV values for the BlueOrigin Low Light USB Camera
horizontalFOV = 80
verticalFOV = 64

"""Returns an approximate value for the angle of an object
from the center of the camera frame.
All arguments to the function should be pixel coordinates
where the top left corner is (0,0).
Width and height are pixel values for the width and height
of the frame, and xmin, xmax, ymin, and ymax are pixel
coordinates for the the center of the object in the frame.
"""
def get_angle(width, height, xmin, xmax, ymin, ymax):
    global horizontalFOV, verticalFOV
    x,y = get_center(xmin, xmax, ymin, ymax)
    horizontal_angle = ((x-width/2)/width) * horizontalFOV
    #don't need this for the moment, but might later
    vertical_angle = ((y-width/2)/width) * horizontalFOV

    return horizontal_angle, vertical_angle

def get_center(xmin, xmax, ymin, ymax):
    x = (xmin + xmax)/2
    y = (ymin + ymax)/2

    return x, y
