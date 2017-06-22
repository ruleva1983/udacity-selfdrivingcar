import glob
import cv2
import numpy as np
import matplotlib.image as mpimg


class DistRemover(object):
    """
    Class used for camera calibration and distorsion removal
    :param paths: list of image paths to images containing chessboard pictures
    :param c_size: tuple with the number of corners in x and y directions
    """
    def __init__(self, paths=glob.glob('data/camera_cal/calibration*.jpg'), c_size=(9, 6)):
        print ("Calibrating camera...")
        self._calibrate(paths, c_size)
        print("Calibration completed...")

    def _calibrate(self, paths, c_size):
        """
        Used to calibrate the camera. Called once at instantiation with same parameter
        as the constructor.
        """
        object_points = np.zeros((c_size[0] * c_size[1], 3), np.float32)
        object_points[:, :2] = np.mgrid[:c_size[0], :c_size[1]].T.reshape(-1, 2)
        objpoints = []
        imgpoints = []
        for path in paths:
            image = mpimg.imread(path)
            gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, c_size, None)
            if ret == True:
                imgpoints.append(corners)
                objpoints.append(object_points)
            else:
                print('Not possible to extract points from image: %s.' % path)

        self.ret, self.mtx, self.dist, _, _ = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    def remove_distorsion(self, image):
        """
        Removes the distorsion in an input image
        :param image: The input image
        :return: The undistorted image
        """
        return cv2.undistort(np.copy(image), self.mtx, self.dist, None, self.mtx)


class PersTransformer(object):
    """
    Class used to define and apply perspective transformations on input images
    :param src_points: The source points forming a rectangle in the original image
    :param dst_points: The points in the transformed image we want to map the rectangle to.
    """
    def __init__(self, src_points=None, dst_points=None, offset=(100, 0)):
        x1, x2 = 207, 1105
        y1 = 470
        if src_points is None:
            src_points = ((x1, 720),(x2, 720),(717, y1),(567, y1))
        if dst_points is None:
            dst_points = ((x1 + offset[0], 720), (x2 - offset[0], 720),
                        (x2 - offset[0], offset[1]), (x1 + offset[0], offset[1]))
        self.src_points = src_points
        self.dst_points = dst_points
        self.M = cv2.getPerspectiveTransform(np.float32(self.src_points), np.float32(self.dst_points))
        self.Minv = cv2.getPerspectiveTransform(np.float32(self.dst_points), np.float32(self.src_points))

    def transform(self, image):
        """
        Applies the transformation to an input image
        :param image: Input image
        :return: The warped image
        """
        return cv2.warpPerspective(np.copy(image), self.M, (image.shape[1], image.shape[0]), flags=cv2.INTER_LINEAR)

    def get_points(self):
        """
        Returns the source points and the ...
        :return:
        """
        return self.src_points, self.dst_points


class Masker(object):
    def __init__(self):
        pass

    @staticmethod
    def _compute_sobel(image, axis, kernel=3):
        """
        Computes the Sobel gradient on input RGB image.

        :param image: The input iumage
        :param axis: The axis of gradient computation, (0 for x, 1 for y).
        :param kernel: The kernel size of the Sobel operator
        :return: The output gradient image
        """
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

        if axis == 0:
            sobel = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=kernel)
        else:
            sobel = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=kernel)
        return sobel

    @staticmethod
    def abs_sobel_mask(image, axis=0, kernel=3, thresh=(0, 255)):
        """
        Computes the absolute Sobel gradient on image and returns a binary mask of
        retained pixels according to threshold values.

        :param image: The input image
        :param axis: The axis determining the direction of the gradient, (0 for x, 1 for y).
        :param kernel: The kernel size of the Sobel operator
        :param thresh: The threshold values
        :return: The binary output mask
        """
        sobel = Masker._compute_sobel(image, axis, kernel=kernel)
        abs_sobel = np.absolute(sobel)
        scaled= np.uint8(255 * abs_sobel / np.max(abs_sobel))

        binary_output = np.zeros_like(scaled)
        binary_output[(scaled >= thresh[0]) & (scaled <= thresh[1])] = 1
        return binary_output

    @staticmethod
    def magnitude_mask(img, kernel=3, thresh=(0, 255)):
        sobel_x = Masker._compute_sobel(img, axis=0, kernel=kernel)
        sobel_y = Masker._compute_sobel(img, axis=1, kernel=kernel)

        sobel_mag = np.sqrt(sobel_x ** 2 + sobel_y ** 2)

        scaled_sobel = np.uint8(255 * sobel_mag / np.max(sobel_mag))

        binary_output = np.zeros_like(scaled_sobel)
        binary_output[(sobel_mag >= thresh[0]) & (sobel_mag <= thresh[1])] = 1
        return binary_output

    @staticmethod
    def direction_mask(img, kernel=3, thresh=(0, np.pi / 2)):
        sobel_x = Masker._compute_sobel(img, axis=0, kernel=kernel)
        sobel_y = Masker._compute_sobel(img, axis=1, kernel=kernel)

        sobel_dir = np.absolute(np.arctan(sobel_y / (sobel_x + 1.e-7)))

        binary_output = np.zeros_like(sobel_dir)
        binary_output[(sobel_dir >= thresh[0]) & (sobel_dir <= thresh[1])] = 1
        return binary_output

    @staticmethod
    def yellow_color_mask(img):
        """
        Returns a
        :param img:
        :return: A mask with zeros and ones
        """
        img_hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
        lower = np.array([15, 50, 100])
        upper = np.array([25, 200, 255])
        return cv2.inRange(img_hls, lower, upper) // 255

    @staticmethod
    def white_color_mask(img):
        img_hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
        lower = np.array([0, 200, 0])
        upper = np.array([255, 255, 255])
        return cv2.inRange(img_hls, lower, upper) // 255

    @staticmethod
    def saturation_mask(img, thresh=(100,255)):
        img_hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
        binary_output = np.zeros_like(img_hls[:, :, 2])
        binary_output[(img_hls[:, :, 2] >= thresh[0]) & (img_hls[:, :, 2] <= thresh[1])] = 1
        return binary_output


    @staticmethod
    def combined_mask(img):
        # Independent Masks
        yellow_mask = Masker.yellow_color_mask(img)
        white_mask = Masker.white_color_mask(img)
        sobel_x_mask = Masker.abs_sobel_mask(img, 0, thresh=(50, 255))

        # Final mask
        color_mask = cv2.bitwise_or(white_mask, yellow_mask)
        output_mask = cv2.bitwise_or(sobel_x_mask, color_mask)

        return output_mask, color_mask, sobel_x_mask

    @staticmethod
    def combined_mask2(img):
        # Independent Masks
        yellow_mask = Masker.yellow_color_mask(img)
        white_mask = Masker.white_color_mask(img)
        sobel_x_mask = Masker.abs_sobel_mask(img, 0, thresh=(50, 255))

        # Final mask
        color_mask = cv2.bitwise_or(white_mask, yellow_mask)
        output_mask = cv2.bitwise_or(sobel_x_mask, color_mask)

        return output_mask, color_mask, sobel_x_mask

