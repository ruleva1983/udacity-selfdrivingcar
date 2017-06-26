import numpy as np
import cv2
from preprocessing import FeatExtractor


class SlidingWindow(object):
    """
    A class for the sliding window obeject.
    :param size: The horizontal and vertical sizes of the window as a tuple
    :param init_pos: The (x,y) coordinate pixels providing the initial position of the upper left corner of the window.
    """
    def __init__(self, size, init_pos):
        self.size = size
        self.pos = init_pos

    def slide(self, deltax, deltay):
        """
        It shifts the window on the image
        :param deltax: Shift in the horizontal direction
        :param deltay: Shift in the vertical direction
        """
        self.pos[0] += deltax
        self.pos[1] += deltay

    def set_size(self, new_size):
        """
        Sets a new size for the window
        :param new_size: The new size
        """
        self.size = new_size

    def crop_image(self, image, resize=None):
        """
        Crops the image in the place of the window and eventually resizes it.
        :param image:  The image to be cropped
        :param resize:  If specified it provides the new size of the image cropped.
        :return:
        """
        cropped = image[self.pos[1]:self.pos[1]+self.size[1],self.pos[0]:self.pos[0]+self.size[0],:]
        if resize is not None:
            cropped = cv2.resize(cropped, resize)
        return cropped

    def centroid(self):
        """
        It returns the center of the window
        :return: The center of the window
        """
        x0 = float(self.pos[0] +  self.size[0] // 2)
        y0 = float(self.pos[1] +  self.size[1] // 2)
        return y0, x0

    def draw_rectangle(self, image, color=(255,0,0), thickness=10):
        """
        Draws a rectangle of the window on an image, specifying color and tickness of the rectangle
        :param image: Input image
        :param color: Color in RGB space
        :param thickness: thickness of the line
        """
        return cv2.rectangle(image, (self.pos[1], self.pos[0]),
                      (self.pos[1] + self.size[1], self.pos[0] + self.size[0]),
                      color=color, thickness=thickness)


def extract_features(cropped, scaler):
    """
    An helper function that extracts all the features from an image and performs preprocessing
    :param cropped: The cropped image from where the features are extracted
    :param scaler: A scaler object to normalize the image
    :return: The feature vector to be used for classification
    """
    feat_HOG = FeatExtractor.extract_HOG(cv2.cvtColor(cropped, cv2.COLOR_RGB2YCrCb), orient=9, cell_per_block=2,
                                         pix_per_cell=8, visualize=False)
    feat_CH = FeatExtractor.extract_COLOR_Histograms(cropped, nbins=32)
    feat_SB = FeatExtractor.extract_spatial_bin(cropped, size=(32, 32))
    features = np.concatenate([feat_CH, feat_SB, feat_HOG])
    features = scaler.transform(features)
    return features



def detect_vehicles(image, model, scaler, limit_image=[[(350, 550), (0, 1280)], [(400, 650), (0, 1280)]],
                     window_sizes=[64, 96], win_steps=[10, 20], colors=[(255, 0, 0), (0, 255, 0)],
                     visualize=False):
    """
    It returns the windows (bounding boxes) where the vehicles have been detected
    :param image: The input image
    :param model: The classifier used for detecting vehicles
    :param scaler: The scaler used for normalization
    :param limit_image: Limit search ranges for different window sizes
    :param window_sizes: The window sizes to be used
    :param win_steps: The steps each window of a given size is shifted in the allowed area.
    :param colors: Colors of windows
    :param visualize: If true superimposes the input image with the bounding boxes.
    :return: A list of windows that contain vehicles, the original image with superimposed the window boxes.
    """
    windows_with_vehicle = []
    copy_image = np.copy(image)
    for i, win_size in enumerate(window_sizes):
        x0 = limit_image[i][1][0]
        y0 = limit_image[i][0][0]
        for x in range(x0, limit_image[i][1][1] - win_size, win_steps[i]):
            for y in range(y0, limit_image[i][0][1] - win_size, win_steps[i]):
                window = SlidingWindow(size=(win_size, win_size), init_pos=(x, y))
                cropped = window.crop_image(image, resize=(64, 64))
                features = extract_features(cropped, scaler)
                if model.predict(features)[0] == 1:
                    windows_with_vehicle.append(window)
                    if visualize:
                        cv2.rectangle(copy_image, (x, y), (x + win_size, y + win_size), color=colors[i], thickness=5)
    return windows_with_vehicle, copy_image


def draw_labeled_bboxes(img, labels):
    """
    Draws a unique box around an area where a vehicle has been found.
    :param img: The input image
    :param labels:  The labels coming from the scipy.ndimage.measurements.label funciton
    :return: The image with superimposed the unique boxes
    """
    for car_number in range(1, labels[1] + 1):
        nonzero = (labels[0] == car_number).nonzero()

        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        bbox = ((np.min(nonzerox), np.min(nonzeroy)), (np.max(nonzerox), np.max(nonzeroy)))
        cv2.rectangle(img, bbox[0], bbox[1], (0, 0, 255), 6)
    return img


def generate_heatmap(image, windows):
    """
    It generates the heatmap given the window boxes where a vehicle has been found.
    :param image: The input image
    :param windows: A list of window boxes
    :return: The heatmap image.
    """
    heatmap = np.zeros_like(image)
    for win in windows:
        heatmap[win.pos[1]:win.pos[1] + win.size[1] , win.pos[0]:win.pos[0] + win.size[0],0] += 1
    return heatmap