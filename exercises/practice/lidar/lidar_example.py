from math import sqrt, cos, pi
from z3 import *

def lawOfCosinesSideDeg(a: float, b: float, angle: float = 2/3*pi) -> float:
    """"""
    return sqrt(a ** 2 + b ** 2 - 2 * a * b * cos(angle))

def lidar(sideLength: float, distances: list) -> tuple:
    """Determines the position and orientation in a square from which the measurements were taken."""
    assert len(distances) == 3, f"Distances list is the wrong length {len(distances)}. Needs to be 3."
    x, y, theta = Reals("x y theta")
    sides = [lawOfCosinesSideDeg(distances[i], distances[(i + 1) % 3]) for i in range(3)]
    x0, y0, x1, y1, x2, y2 = Reals("x0 y0 x1 y1 x2 y2")
    s = Solver()
    equations = [
        0 <= x0,
        0 <= x1,
        0 <= x2,
        0 <= y0,
        0 <= y1,
        0 <= y2,
        x0 <= sideLength,
        x1 <= sideLength,
        x2 <= sideLength,
        y0 <= sideLength,
        y1 <= sideLength,
        y2 <= sideLength,
        Or(Or(x0 == 0, x0 == sideLength), Or(y0 == 0, y0 == sideLength)),
        Or(Or(x1 == 0, x1 == sideLength), Or(y1 == 0, y2 == sideLength)),
        Or(Or(x2 == 0, x2 == sideLength), Or(y2 == 0, y2 == sideLength)),
        sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2) == sides[0],
        sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2) == sides[1],
        sqrt((x0 - x2) ** 2 + (y0 - y2) ** 2) == sides[2],
        sqrt((x0 - x) ** 2 + (y0 - y) ** 2) == distances[0],
        sqrt((x1 - x) ** 2 + (y1 - y) ** 2) == distances[1],
        sqrt((x2 - x) ** 2 + (y2 - y) ** 2) == distances[2],
    ]
    s.add(equations)
    assert s.check() == sat
    m = s.model()
    return (m.eval(x), m.eval(y), m.eval(theta))
