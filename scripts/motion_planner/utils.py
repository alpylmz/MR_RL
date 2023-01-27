import numpy as np

def get_distance(x1, y1, x2, y2):
    """
    Returns the distance between two points.
    """
    return np.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def ccw(A, B, C):
    """
    Returns true if the points are in counter clockwise order.
    """
    return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

def intersect(A, B, C, D):
    """
    Returns true if the line segments AB and CD intersect.
    """
    return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)