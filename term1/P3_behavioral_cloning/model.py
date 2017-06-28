import os

import cv2
import numpy as np
import pandas as pd

from keras.models import Sequential
from keras.layers.convolutional import Convolution2D
from keras.layers import Dense, Dropout, Flatten, Lambda
from keras.optimizers import Adam

import matplotlib.image as mpimg



SIZE_INPUT = (66, 200, 3)
MAX_TRANS_RANGE = 25
STEERING_CORRECTION = 0.25
BATCH_SIZE = 64
NB_EPOCHS = 20
KEEP_PROB = 0.3
WEIGHTS = 'glorot_uniform'


def preprocess(img):
    """
    Resizes the image to appropriate size for network input
    :param img: Input image
    :return: Resized image
    """
    def resize(x):
        ratio = float(SIZE_INPUT[1]) / float(x.shape[1])
        resized_size = (int(x.shape[1] * ratio), int(x.shape[0] * ratio))
        x = cv2.resize(x, dsize=resized_size)
        crop_above = resized_size[1] - SIZE_INPUT[0]
        return x[crop_above:, :, :]

    img = resize(img)
    return img


def random_translation(img, angle):
    """
    Shifts the image horizontally and corrects the associated angle.
    :param img: Input image
    :param angle: Associated steering angle
    :return: The shifted image, the corrected angle
    """
    tr_x = np.random.uniform(-MAX_TRANS_RANGE, MAX_TRANS_RANGE)
    angle = angle + (tr_x / MAX_TRANS_RANGE) * STEERING_CORRECTION

    rows = img.shape[0]
    cols = img.shape[1]

    M = np.float32([[1,0,tr_x],[0,1,0]])
    img = cv2.warpAffine(img,M,(cols,rows))

    return img, angle

def augmentation(image, angle):
    """
    Performs augmentation on input image. Both random_translation and flipping can be applied.
    :param image: Input image
    :param angle: Associated angle
    :return: The augmented image and the updated angle
    """
    image, angle = random_translation(image, angle)

    if np.random.randint(2):
        image, angle = cv2.flip(image, 1), -angle
    return image, angle


def train_gen(data, batch_size):
    """
    A generator which provides batches for training.
    :param data: Info data about location of images
    :param batch_size: The size of the batch to return
    :yield: The images and correspondent angles
    """
    X = np.column_stack((np.copy(data['center']), np.copy(data['left']), np.copy(data['right'])))
    y = np.copy(data['steering'])
    while 1:
        x_out = []
        y_out = []

        for i in range(0, batch_size):

            idx = np.random.randint(len(y))

            idx_img = np.random.randint(3)

            x_i = mpimg.imread(X[idx][idx_img].strip())
            if idx_img == 0:
                y_i = y[idx]
            elif idx_img == 1:
                y_i = y[idx] + STEERING_CORRECTION
            else:
                y_i = y[idx] - STEERING_CORRECTION

            x_i = preprocess(x_i)
            x_i, y_i = augmentation(x_i, y_i)

            x_out.append(x_i)
            y_out.append(y_i)

        yield (np.array(x_out), np.array(y_out))


def val_data(data):
    """
    Provides data for validation (Non in generator format)
    :param data: Info data about location of images
    :return: The validation images + steering angles.
    """
    X = np.column_stack((np.copy(data['center']), np.copy(data['left']), np.copy(data['right'])))
    y = np.copy(data['steering'])
    X_out = []
    Y_out = []
    for i in range(len(y)):

        x_out = mpimg.imread(X[i][0].strip())
        y_out = np.array([y[i]])

        x_out = preprocess(x_out)

        Y_out.append(y_out)
        X_out.append(x_out)

    return np.array(X_out), np.array(Y_out)



def stack_model():
    """
    Defines the convolutional network model and compiles it
    :return: The keras model
    """
    model = Sequential()

    model.add(Lambda(lambda x: x/255.0 -0.5, input_shape=SIZE_INPUT, output_shape=SIZE_INPUT))

    model.add(Convolution2D(24, 5, 5, activation="relu", border_mode='valid', init = WEIGHTS, subsample = (2, 2)))
    model.add(Convolution2D(36, 5, 5, activation="relu", border_mode='valid', init = WEIGHTS, subsample = (2, 2)))
    model.add(Convolution2D(48, 5, 5, activation="relu", border_mode='valid', init = WEIGHTS, subsample = (2, 2)))
    model.add(Convolution2D(64, 3, 3, activation="relu", border_mode='valid', init = WEIGHTS, subsample = (1, 1)))
    model.add(Convolution2D(64, 3, 3, activation="relu", border_mode='valid', init = WEIGHTS, subsample = (1, 1)))
    model.add(Flatten())
    model.add(Dropout(KEEP_PROB))

    model.add(Dense(100, init = WEIGHTS, activation="relu"))
    model.add(Dropout(KEEP_PROB))

    model.add(Dense(50, init = WEIGHTS,  activation="relu"))
    model.add(Dropout(KEEP_PROB))

    model.add(Dense(10, init = WEIGHTS,  activation="relu"))
    model.add(Dropout(KEEP_PROB))

    model.add(Dense(1, init = WEIGHTS))

    model.compile(loss = 'mse', optimizer = Adam(lr = 0.001))
    return model

def train_model(model, n_epochs, data):
    """
    Trains the model
    :param model: The compiled keras model
    :param n_epochs: The number of epochs
    :param data: Data info for image extraction
    """

    gen_train = train_gen(data, BATCH_SIZE)
    X_valid, y_valid = val_data(data)

    model.fit_generator(generator=gen_train,
                        samples_per_epoch=len(data) // BATCH_SIZE,
                        validation_data=(X_valid, y_valid),
                        nb_val_samples=len(data),
                        nb_epoch=n_epochs,
                        verbose=1)



def get_training_data(log_file):
    """
    Loads the data info for image extraction
    :param log_file: The log file with data information
    :return: The data info
    """
    data = pd.read_csv(log_file)
    return data[["center", "left", "right", "steering"]]




def build_model(log, n_epochs):
    """
    Calls the training data, model stacking and model training functions
    :param log: The logfile of the data
    :param n_epochs: The number of epochs training
    :return: The model trained
    """
    data = get_training_data(log)
    model = stack_model()
    train_model(model, n_epochs, data)
    return model

def main():
    """
    Main function
    """

    log = "data/udacity/driving_log.csv"
    model = build_model(log, NB_EPOCHS)
    model.save(os.path.join("models/", 'final_model.h5'))

if __name__ == '__main__':
    main()