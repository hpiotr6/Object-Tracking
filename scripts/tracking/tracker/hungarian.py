from scipy.optimize import linear_sum_assignment
import numpy as np


class HungarianAlgorithm():
    """
    A wrapper class to assign predictions to detections.

    Attributes
    ----------
    detections : Detection's list
    predictions : Point's list

    Methods
    --------
    create_matrix():
        Create cost matrix
    get_indices():
        Solve linear sum assigment problem and give cost matrix's indices.
    """
    def __init__(self, detections: list, predictions: list):
        self.__detections = detections
        self.__predictions = predictions

    @property
    def detections(self):
        return self.__detections

    @property
    def predictions(self):
        return self.__predictions

    def create_matrix(self) -> [int]:
        costs = []
        for detection in self.detections:
            row = []
            for prediction in self.predictions:
                p = np.array([prediction.x, prediction.y])
                d = np.array([detection.x, detection.y])
                row.append(np.linalg.norm(p-d))
            costs.append(row)
        return costs

    def get_indices(self) -> [[int], [int]]:
        matrix = self.create_matrix()
        costs = np.array(matrix)
        row_ind, col_ind = linear_sum_assignment(costs)
        return row_ind, col_ind
