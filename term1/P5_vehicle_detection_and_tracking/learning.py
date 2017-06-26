from sklearn.svm import LinearSVC
from sklearn.model_selection import cross_val_score
from sklearn.model_selection import GridSearchCV
from sklearn.metrics import confusion_matrix

import numpy as np


class LinearSVM(object):
    """
    Defines a Linear SVM model with helper functions to call the
    standard training and validation methods.
    :param **kwargs: See documentation from sklearn.svm.LinearSVC
    """
    def __init__(self, **kwargs):
        self.model = LinearSVC(**kwargs)

    def fit(self, X, y):
        """
        Fits the model
        :param X: Input features
        :param y: Input labels
        """
        self.model.fit(X, y)

    def predict(self, X):
        """
        Predict new instances
        :param X: New instance features
        :return: The prediction vector
        """
        return self.model.predict(X)

    def score(self, X, y):
        """
        Returns the accuract of the model
        :param X: The input features
        :param y: The input labels
        :return: The accuracy
        """
        return self.model.score(X, y)

    def confusion_matrix(self, X, y):
        """
        Returns the confusion matrix of the predictions
        :param X:  The input features
        :param y: The expected labels
        :return: The confusion matrix of the predictions vs real labels
        """
        return confusion_matrix(y, self.predict(X))

    def cross_validation(self, X, y, cv=5):
        """
        Performs k-fold crossvalidation
        :param X: Training set features
        :param y: Training set labels
        :param cv: number of folds
        :return: The crossvalidation accuracy
        """
        return np.mean(cross_val_score(LinearSVC(), X, y, cv=cv))

    def grid_search(self, X, y, params, cv=5):
        """
        Performs gridsearch over a set of parameters
        :param X: Input features
        :param y: Input labels
        :param params: Dictionary with parameter variability
        :param cv: Number of cross validation folds
        :return: The results of the grid search.
        """
        clf = GridSearchCV(LinearSVC(), params, cv=cv)
        clf.fit(X, y)
        return clf.cv_results_



