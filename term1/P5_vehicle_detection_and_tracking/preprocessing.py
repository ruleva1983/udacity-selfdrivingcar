import cv2
import numpy as np
from skimage.feature import hog

class FeatExtractor(object):
    """
    This class collects static methods for extracting features from images.
    """
    def __init__(self):
        pass

    @staticmethod
    def extract_HOG(image, orient, pix_per_cell, cell_per_block, visualize=True):
        """
        It extracts HOG features from 3 channel input images.
        :param image: The input image
        :param orient: Same parameter as in the skimage.hog documentation
        :param pix_per_cell: Same parameter as in the skimage.hog documentation
        :param cell_per_block: Same parameter as in the skimage.hog documentation
        :param visualize: If True it returns also an image containing visualization of the hog features
        """
        hog_image = np.zeros_like(image)
        features = []
        for i in range(3):
            if visualize:
                features_i, hog_image[:,:,i] = hog(image[:, :, i], orientations=orient, pixels_per_cell=(pix_per_cell, pix_per_cell),
                    cells_per_block=(cell_per_block, cell_per_block), transform_sqrt=False,
                    visualise=True, feature_vector=False)
            else:
                features_i = hog(image[:, :, i], orientations=orient, pixels_per_cell=(pix_per_cell, pix_per_cell),
                                    cells_per_block=(cell_per_block, cell_per_block), transform_sqrt=False,
                                    visualise=False, feature_vector=True)
            features.append(features_i)
        if visualize:
            return np.array(features), hog_image
        else:
            return np.concatenate(features)


    @staticmethod
    def extract_COLOR_Histograms(image, color_space='RGB', nbins=32, bins_range=(0, 256)):
        """
        Extracts histogram of color channels from image.
        :param image: The input image
        :param color_space: The color space of the input image
        :param nbins: Number of bins that divide the color channel intensity
        :param bins_range: The color channel intensity range
        :returns: The features for each channel concatenated
        """
        if color_space != 'RGB':
            if color_space == 'HSV':
                feature_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
            elif color_space == 'LUV':
                feature_image = cv2.cvtColor(image, cv2.COLOR_RGB2LUV)
            elif color_space == 'HLS':
                feature_image = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
            elif color_space == 'YUV':
                feature_image = cv2.cvtColor(image, cv2.COLOR_RGB2YUV)
            else:
                raise KeyError
        else:
            feature_image = np.copy(image)

        ch_1_hist = np.histogram(feature_image[:, :, 0], bins=nbins, range=bins_range)
        ch_2_hist = np.histogram(feature_image[:, :, 1], bins=nbins, range=bins_range)
        ch_3_hist = np.histogram(feature_image[:, :, 2], bins=nbins, range=bins_range)

        features = np.concatenate((ch_1_hist[0], ch_2_hist[0], ch_3_hist[0]))
        return features


    @staticmethod
    def extract_spatial_bin(image, color_space='RGB', size=(32, 32)):
        """
        Takes an input image, resizes it to a low resolution image
        and extract the color values for each pixel in a choosable
        color space.

        :param image: The input image in RGB color space
        :param color_space: The chosen color space
        :param size: The size of the new image
        :return: The feature vector
        """
        if color_space != 'RGB':
            if color_space == 'HSV':
                feature_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
            elif color_space == 'LUV':
                feature_image = cv2.cvtColor(image, cv2.COLOR_RGB2LUV)
            elif color_space == 'HLS':
                feature_image = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
            elif color_space == 'YUV':
                feature_image = cv2.cvtColor(image, cv2.COLOR_RGB2YUV)
            else:
                raise KeyError
        else:
            feature_image = np.copy(image)
        features = cv2.resize(feature_image, size).ravel()
        return features

