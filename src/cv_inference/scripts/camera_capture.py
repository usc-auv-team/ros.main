"""
Simply display the contents of the webcam with optional mirroring using OpenCV
via the new Pythonic cv2 interface.  Press <esc> to quit.
"""

import cv2
import time


def show_webcam(mirror=False):
    cam = cv2.VideoCapture(0)
    timestr = time.strftime('%Y-%m-%d_%H:%M:%S_')
    outstr = timestr + 'output.MP4'
    print(outstr)
    fourcc = cv2.VideoWriter_fourcc(*'MP4V')
    out = cv2.VideoWriter(outstr, fourcc, cam.get(cv2.CAP_PROP_FPS),
                          (int(cam.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))))

    while True:
        ret_val, img = cam.read()
        if mirror:
            img = cv2.flip(img, 1)
        out.write(img)
        cv2.imshow('front_camera', img)
        if cv2.waitKey(1) == 27:
            break  # esc to quit
    out.release()
    cam.release()
    cv2.destroyAllWindows()


def main():
    show_webcam(mirror=True)


if __name__ == '__main__':
    main()
