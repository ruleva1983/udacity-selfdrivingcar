import cv2
import numpy as np
import os
from moviepy.editor import VideoFileClip
from IPython.display import display, HTML
from preprocess import Masker, DistRemover, PersTransformer
from utils import superimpose_images

class Line(object):
    """
    A class that holds the a single road line. Usually instantiated
    without arguments. If wrong argument are passed, default instantiation
    is imposed.
    :param coeffs: The polynomial coefficients of the line
    :param degree: The degree of the polynomial fitted
    """

    yM_PIXEL = 0.042
    xM_PIXEL = 0.0053

    def __init__(self, coeffs=None, degree=None):
        self.coeffs = coeffs
        if coeffs is not None:
            try:
                assert (degree == len(coeffs) - 1)
            except AssertionError:
                self.polydegree = None
                self.coeffs = None
                print ("Line instantiated with degree and coefficient conflict. Imposing default instantiation!!")
        self.coeffs_meter = None

    def fit(self, x, y, degree=2):
        """
        Fits the line on a plane with a polynomial of given degree in both pixel and real space.
        The method is adapted to the image case where the independent variable is y and the
        dependent variable is x.
        :param x: The dependent variable.
        :param y: The independent variable.
        :param degree: The degree of the polynomial.
        """
        assert len(x) == len(y)
        self.polydegree = degree
        self.coeffs = np.polyfit(y, x, degree)
        self.coeffs_meter = np.polyfit(self.yM_PIXEL * y, self.xM_PIXEL * x, degree)

    def exists(self):
        """
        Checks if the line has been fitted once.
        :return: True if fit has been performed, False otherwise.
        """
        if self.coeffs is None:
            return False
        return True

    def evaluate_curvature(self, y):
        """
        Evaluates and returns the curvature radius of the line in meters at position pixel y.
        Raises assertion error if the line has not been previously fitted.
        :param y: The position in pixels.
        :return : The curvature radius in meters.
        """
        assert self.exists()
        yp = y * self.yM_PIXEL
        A, B = self.coeffs_meter[0], self.coeffs_meter[1]
        return ((1 + (2 * A * yp + B) ** 2) ** (1.5)) / np.absolute(2 * A)

    def get_line_points(self, x):
        """
        Evaluates the images of the input array through the line equation. Assertion exception
        thrown if the line has not been fitted.
        :param x: An array of independent variable points.
        :return: A tuple of input points and their images according to the line equation.
        """
        assert self.exists()
        y = np.zeros_like(x)
        for i in range(self.polydegree + 1):
            y += self.coeffs[i] * np.power(x, self.polydegree - i)
        return x, y

    @staticmethod
    def get_line_pixels(mask, x0, window_size=(100, 100)):
        """
        It detects which pixels in the mask belong to a line starting from
        the lower part of the image.
        :param mask: A mask image, one channel, either zero or one values.
        :param x0: The initial center of the sliding window.
        :param window_size: A tuple of windows sizes (horizontal, vertical)
        :return: A tuple containing: An array for x coordinates of the detected pixels;
                 An array with y coordinates of the detected pixels; The sliding windows
                 coordinates to be passed as arguments for cv2.rectangle function.
        """

        n_windows = int(mask.shape[0] / window_size[1])
        full_line_pixels_x, full_line_pixels_y = [], []
        win_pixels_x, win_pixels_y = [], []

        y_below = mask.shape[0]
        x_left = x0 - window_size[0] // 2
        windows = [[(x_left, y_below), (x_left + window_size[0], y_below - window_size[1])]]

        for x in range(x_left, x_left + window_size[0]):
            for y in range(y_below - window_size[1], y_below):
                if mask[y, x] == 1:
                    win_pixels_x.append(x)
                    win_pixels_y.append(y)

        full_line_pixels_x += win_pixels_x
        full_line_pixels_y += win_pixels_y

        for n in range(1, n_windows):
            if win_pixels_x:
                x_left = int(np.mean(win_pixels_x)) - window_size[0] // 2
            y_below -= window_size[1]

            windows.append([(x_left, y_below), (x_left + window_size[0], y_below - window_size[1])])

            win_pixels_x, win_pixels_y = [], []
            for x in range(x_left, x_left + window_size[0]):
                for y in range(y_below - window_size[1], y_below):
                    if mask[y, x] > 0:
                        win_pixels_x.append(x)
                        win_pixels_y.append(y)

            full_line_pixels_x += win_pixels_x
            full_line_pixels_y += win_pixels_y
        return np.array(full_line_pixels_x), np.array(full_line_pixels_y), windows


class RoadLane(object):
    """
    A class holding a Lane of the Road, i.e. a left and a right line
    that form the lane.
    """
    def __init__(self):
        self.rightLine = Line()
        self.leftLine = Line()

    def lane_curvature(self, y=720):
        """
        Evaluates the curvature of the lane in meters, by taking the mean curvature
        of the two lines evaluated at position y.
        :param y: pixel vertical position.
        :return : The average curvature.
        """
        return self.leftLine.evaluate_curvature(y), self.rightLine.evaluate_curvature(y)

    def relative_car_position(self, width=1280, y=720.):
        """
        Returns the distance in meters to the center of the line.
        If negative the car stands on the right side of the center.
        :param width: The number of horizontal pixels of the image
        :param y: The vertical position of the camera in the image (lowest part)
        """
        x_l = self.leftLine.get_line_points(y)[1]
        x_r = self.rightLine.get_line_points(y)[1]
        return ((x_r + x_l) / 2. - width / 2.) * self.rightLine.xM_PIXEL

    def update(self, mask):
        """
        If the lines have not been previously determined, it generates
        them using the full algorithm. Otherwise it updates the already
        existing lines with new data image.
        :param mask: The new image mask (one channel, only zeros and ones values)
        """
        if not (self.rightLine.exists() and self.leftLine.exists()):
            self._generateLane(mask)
        else:
            self._updateLane(mask)

    def _generateLane(self, mask, hist_level=None, degree=2):
        """
        Estimate the lines from an input mask image using a sliding
        windows algorithm. First it estimates the initial pixel position
        of the left and right lines using an histogram approach.
        Then uses these values to track the lines from the bottom to the top.
        Finalizes with fitting the two lines updating their coefficients.

        :param mask: The input mask image
        :param hist_level: The upper pixel limit for histogram search.
        :param degree: The degree of the polinomial to fit the lines
        """
        init_left, init_right, _ = RoadLane.get_initial_points(mask, hist_level)

        pixels_x, pixels_y, _ = Line.get_line_pixels(mask, init_left)
        self.leftLine.fit(pixels_x, pixels_y, degree)

        pixels_x, pixels_y, _ = Line.get_line_pixels(mask, init_right)
        self.rightLine.fit(pixels_x, pixels_y, degree)

    def _updateLane(self, mask):

        left = Tracker(self.leftLine)
        left.update_line(mask)

        right = Tracker(self.rightLine)
        right.update_line(mask)

    def draw_free_space(self, image, color=(0, 255, 0)):
        """
        Draws the free space between the lines on the input image.

        :param image: The input three channel image
        :param color: The color of the free space area in RGB color space.
        :return : The image with the superimposed lane area, the original image
        """
        xl, yl = self.leftLine.get_line_points(np.linspace(0, 720, 100))
        xr, yr = self.rightLine.get_line_points(np.linspace(0, 720, 100))
        pts_left = np.array([np.transpose(np.vstack([yl, xl]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([yr, xr])))])
        pts = np.hstack((pts_left, pts_right))

        drawn_image = np.copy(image)
        cv2.fillPoly(drawn_image, np.int_([pts]), color)
        return drawn_image, image

    def draw_lines(self, image, thick=10, color=(255, 0, 0)):
        """
        Draws the lines on an input image.
        :param image: The input image
        :param thick: The tickness of the line to be drawn
        :param color: The RGB color of the line
        :return : The image with superimposed lines and the original image
        """
        xl, yl = self.leftLine.get_line_points(np.linspace(0, 720, 100))
        xr, yr = self.rightLine.get_line_points(np.linspace(0, 720, 100))
        drawn_image = np.copy(image)
        for i in range(len(yl) - 1):
            p1 = (int(yl[i]), int(xl[i]))
            p2 = (int(yl[i + 1]), int(xl[i + 1]))
            cv2.line(drawn_image, p1, p2, color=color, thickness=thick)

        for i in range(len(yr) - 1):
            p1 = (int(yr[i]), int(xr[i]))
            p2 = (int(yr[i + 1]), int(xr[i + 1]))
            cv2.line(drawn_image, p1, p2, color=color, thickness=thick)
        return drawn_image, image

    @staticmethod
    def get_initial_points(mask, hist_level=None):
        """
        Estimates initial search points for left and right lines.

        :param mask: The mask image
        :param hist_level:
        :return: The left and the right initial points, as well as the histogram values
                 for visualization
        """
        if hist_level is None:
            histogram = np.sum(mask[mask.shape[0] // 2:, :], axis=0)
        else:
            histogram = np.sum(mask[hist_level:, :], axis=0)
        middle = len(histogram) // 2
        left = np.argmax(histogram[:middle])
        right = middle + np.argmax(histogram[middle:])
        return left, right, histogram


class Tracker(object):
    """
    This object holds an already fitted line, and updates it according to a new input mask
    using a lighter algorithm.

    :param line: A line object already fitted
    """
    def __init__(self, line):
        assert line.exists()
        self.line = line

    def update_line(self, mask):
        """
        The main method of the class. It performs the update of the line coefficients.

        :param mask: The input mask
        """
        area_mask = self._create_search_area(mask)
        line_mask = cv2.bitwise_and(area_mask, mask)
        points = np.squeeze(cv2.findNonZero(line_mask))
        pixels_x = points[:, 0]
        pixels_y = points[:, 1]
        self.line.fit(pixels_x, pixels_y)

    def _create_search_area(self, mask, width=200):
        """
        Creates an
        :param mask: The input mask
        :param width:
        :return:
        """
        lane = RoadLane()
        lane.leftLine.coeffs = np.copy(self.line.coeffs)
        lane.leftLine.polydegree = 2
        lane.leftLine.coeffs[-1] -= width // 2

        lane.rightLine.coeffs = np.copy(self.line.coeffs)
        lane.rightLine.polydegree = 2
        lane.rightLine.coeffs[-1] += width // 2

        area_mask = np.zeros_like(mask)
        area_mask, _= lane.draw_free_space(area_mask, color=(255, 255, 255))
        return area_mask


class Pipeline(object):
    def __init__(self, remover=None):
        self.lane = RoadLane()
        # We need to pass a remover otherwise it will always calibrate
        if remover is None:
            self.Remover = DistRemover()
        else:
            self.Remover = remover
        self.Transformer = PersTransformer()

    def run(self, img):
        img_undistorted = self.Remover.remove_distorsion(img)
        img_masked, _, _ = Masker.combined_mask(img_undistorted)
        img_warped = self.Transformer.transform(img_masked)
        Minv = self.Transformer.Minv
        self.lane.update(img_warped)
        img_out = superimpose_images(img, self.lane, Minv, self.lane.lane_curvature(720))
        return img_out


if __name__ == "__main__":


    import matplotlib.image as mpimg
    import matplotlib.pyplot as plt

    #example = 'data/test_images/test5.jpg'
    #img = mpimg.imread(example)
    pipeline = Pipeline()
    #plt.imshow(pipeline.run(img))
    #plt.show()

    #example = 'data/test_images/test6.jpg'
    #img = mpimg.imread(example)
    #pipeline = Pipeline()
    #plt.imshow(pipeline.run(img))
    #plt.show()


    #Remover = DistRemover()


    video = "data/videos/challenge_video.mp4"
    clip = VideoFileClip(video)
    clip_processed = clip.fl_image(pipeline.run)

    _, video_name = os.path.split(video)
    out_name = os.path.join("data/report_images/", video_name)
    clip_processed.write_videofile(out_name, audio=False)