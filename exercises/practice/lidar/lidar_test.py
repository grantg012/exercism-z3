import unittest
# from lidar import lidar
from lidar_example import lidar
import z3

def toFloat(x: z3.RatNumRef) -> float:
    """"""
    if(isinstance(x, float) or isinstance(x, int)):
        return x
    x = x.as_fraction()
    return float(x.numerator) / float(x.denominator)

class LidarTest(unittest.TestCase):
    def test1(self):
        # pos = lidar(10, [5.584846, 3.724622, 7.000767])
        pos = lidar(10, [10, 10, 10])
        print(pos)

def main():
    """"""
    unittest.main()

if(__name__ == "__main__"):
    main()
