import numpy as np
import cv2


class SlidingWindow(object):
    def __init__(self, size, init_pos):
        self.size = size
        self.pos = init_pos

    def slide(self, deltax, deltay):
        self.pos[0] += deltax
        self.pos[1] += deltay

    def set_size(self, new_size):
        self.size = new_size

    def crop_image(self, image, resize=None):
        cropped = image[self.pos[1]:self.pos[1]+self.size[1],self.pos[0]:self.pos[0]+self.size[0],:]
        if resize is not None:
            cropped = cv2.resize(cropped, resize)
        return cropped

    def centroid(self):
        x0 = float(self.pos[0] +  self.size[0] // 2)
        y0 = float(self.pos[1] +  self.size[1] // 2)
        return x0, y0






