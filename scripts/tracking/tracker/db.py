class Point():
    """
    A class to represent a point.

    Attributes
    ----------
    x : float
    y : float
    coords : tuple
    """

    def __init__(self, x: float, y: float):
        self.__x = x
        self.__y = y
        self.__coords = (x, y)

    @property
    def coords(self):
        return self.__coords

    @coords.setter
    def coords(self, value: tuple):
        self.__coords = value

    @property
    def x(self):
        return self.__x

    @property
    def y(self):
        return self.__y


class Detection(Point):
    """
    A class to represent a detection, subclasses Point.

    Attributes
    ----------
    x : float
    y : float
    """
    def __init__(self, x: float, y: float):
        super().__init__(x, y)


class Obstacle(Point):
    """
    A class to represent an obstacle, subclasses Point.

    Attributes
    ----------
    x : float
    y : float
    id : static int

    Methods:
    --------
    predict():
        Kalman prediction
    """
    __id = 0

    def __init__(self, x: float, y: float):
        super().__init__(x, y)
        Obstacle.__id += 1
        self.__id = Obstacle.__id

    def __del__(self):
        Obstacle.__id -= 1

    def predict(self) -> Point:
        pass

    @property
    def id(self):
        return self.__id


class DB():
    """
    A class to represent an database.

    Attributes
    ----------
    data: list

    Methods:
    --------
    add()
    delete()
    """
    def __init__(self):
        self.__data = []

    def add(self, object) -> None:
        self.__data.append(object)

    def delete(self) -> None:
        pass

    @property
    def data(self):
        return self.__data


class DetectionsDB():
    """
    A class to represent an detection database.

    Attributes
    ----------
    data: list

    Methods:
    --------
    """
    def __init__(self):
        self.__data = []

    @property
    def data(self):
        return self.__data

    @data.setter
    def data(self, detections: list):
        self.__data = []
        for x, y in detections:
            self.__data.append(Detection(x, y))


class ObstaclesDB(DB):
    """
    A class to represent a obstacle database, subclasses DB.
    """
    def __init__(self):
        super().__init__()

    def predict(self) -> dict:
        # predictions = {}
        # for data in self.data:
        #   predictions[data] = data.predict()
        # return predictions

        return {self.data[0]: Point(3, 4),
                self.data[1]: Point(8, 2)}

    def assign(self, matchings: tuple) -> None:
        for obstacle in self.data:
            for matching in matchings:
                detection_coords = matching[0][0]
                prediction_coords = matching[0][1]
                if obstacle.coords == prediction_coords:
                    obstacle.coords = detection_coords
