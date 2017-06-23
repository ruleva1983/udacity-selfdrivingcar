import cv2
import numpy as np
from skimage.feature import hog

class FeatExtractor(object):
    def __init__(self):
        pass

    @staticmethod
    def extract_HOG(image, orient, pix_per_cell, cell_per_block, feature_vec=True, channels="ALL", normalize=True):
        """
        Extracts HOG features from input image channels

        :param image:
        :param orient:
        :param pix_per_cell:
        :param cell_per_block:
        :param feature_vec:
        :param normalize:
        :return:
        """
        def HOG_single_channel(channel, orient, pix_per_cell, cell_per_block, feature_vec=True):
            return hog(channel, orientations=orient, pixels_per_cell=(pix_per_cell, pix_per_cell),
                        cells_per_block=(cell_per_block, cell_per_block), transform_sqrt=False,
                        visualise=False, feature_vector=feature_vec)

        if channels == "ALL":
            features = np.array([HOG_single_channel(image[:,:,i], orient, pix_per_cell, cell_per_block) for i in range(3)])
            features = np.concatenate(features)
        else:
            features = HOG_single_channel(image[:, :, channels], orient, pix_per_cell, cell_per_block)
        return np.array(features)

    @staticmethod
    def extract_COLOR_Histograms(image, color_space='RGB', nbins=32, bins_range=(0, 256), normalize=True):
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
        #if normalize:
        #    features = (features - np.min(features))/(np.max(features)-np.min(features))
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