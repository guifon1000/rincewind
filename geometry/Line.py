from geometry.Point import Point
from geometry.Vector import Vector
import sys
sys.path.append('../')
from geo_functions import cross, is_on_line
class Line(list):
    """
    defines a line 
    """
    def __init__(self, *largs):
        if ([type(largs[0][i]) for i in range(len(*largs))] == [Point,Vector]):
            pt = largs[0][0]
            vec = largs[0][1].unit()
            super(Line, self).__init__((pt, vec))
        elif ([type(largs[0][i]) for i in range(len(*largs))] == [Point,Point]):
            pt = largs[0][0]
            vec = Vector([largs[0][1][i] - largs[0][0][i] for i in range(3)]).unit()
            super(Line, self).__init__((pt, vec))


 
    def __eq__(self, other):
        vec1 = self[1].unit()
        vec2 = other[1].unit()
        ans = False
        if cross(vec1,vec2).norm < 1.e-12:
            if is_on_line(self[0], other) and is_on_line(other[0], self):
                ans = True
        else:
            ans = False
        return ans


    def parameter_point(self, par):
        pt = [self[0][i] + par * self[1][i] for i in range(3) ]
        return Point(pt)
