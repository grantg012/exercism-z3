from math import sqrt, cos, pi
from z3 import *

def lawOfCosinesSideDeg(a: float, b: float, angle: float = 2/3*pi) -> float:
    """"""
    return sqrt(a ** 2 + b ** 2 - 2 * a * b * cos(angle))

def lidar(sideLength: float, distances: list) -> tuple:
    """Determines the position and orientation in a square from which the measurements were taken."""
    assert len(distances) == 3, f"Distances list is the wrong length {len(distances)}. Needs to be 3."
    x, y = Reals("x y")
    sides = [lawOfCosinesSideDeg(distances[i], distances[(i + 1) % 3]) for i in range(3)]
    distances = [RealVal(d) for d in distances]
    sides = [RealVal(s) for s in sides]
    sideLength = RealVal(sideLength)
    x0, y0, x1, y1, x2, y2 = Reals("x0 y0 x1 y1 x2 y2")
    # x1, y1, x2, y2 = Reals("x1 y1 x2 y2")
    # x0, y0 = RealVal(0), RealVal(sideLength)
    s = Solver()
    equations = [
        # Constraints for being on a square
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
        # Constraints for being on a circle
        # (x0 ** 2 + y0 ** 2) ** 0.5 == sideLength,
        # (x1 ** 2 + y1 ** 2) ** 0.5 == sideLength,
        # (x2 ** 2 + y2 ** 2) ** 0.5 == sideLength,
        # (x0 ** 2 + y0 ** 2) == sideLength ** 2,
        # (x1 ** 2 + y1 ** 2) == sideLength ** 2,
        # (x2 ** 2 + y2 ** 2) == sideLength ** 2,
        # Constraints to enforce distances between points
        # ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5 == sides[0],
        # ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5 == sides[1],
        # ((x0 - x2) ** 2 + (y0 - y2) ** 2) ** 0.5 == sides[2],
        ((x1 - x0) ** 2 + (y1 - y0) ** 2) == sides[0] ** 2,
        # ((x2 - x1) ** 2 + (y2 - y1) ** 2) == sides[1] ** 2,
        # ((x0 - x2) ** 2 + (y0 - y2) ** 2) == sides[2] ** 2,
        # Constraints to enforce distances between points and measurement spot
        # ((x0 - x) ** 2 + (y0 - y) ** 2) ** 0.5 == distances[0],
        # ((x1 - x) ** 2 + (y1 - y) ** 2) ** 0.5 == distances[1],
        # ((x2 - x) ** 2 + (y2 - y) ** 2) ** 0.5 == distances[2],
        # ((x0 - x) ** 2 + (y0 - y) ** 2) == distances[0] ** 2,
        # ((x1 - x) ** 2 + (y1 - y) ** 2) == distances[1] ** 2,
        # ((x2 - x) ** 2 + (y2 - y) ** 2) == distances[2] ** 2,
    ]
    s.add(equations)
    isSat = s.check()
    if(isSat):
        m = s.model()
        # return (m.eval(x), m.eval(y))
        return (m.eval(x0), m.eval(y0))
    else:
        return None
