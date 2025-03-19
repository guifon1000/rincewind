import sys
from geometry.Polyline3D import Polyline3D

class Named3DSection(dict):
    def __init__(self, polylines, ordered_keys):
        if len(polylines) == len(ordered_keys):
            d = {}
            self.ordered_keys = ordered_keys
            whole_list = []
            for i,k in enumerate(self.ordered_keys):
                array = polylines[i]
                lst = []
                for p in array:
                    lst.append(p)
                    if not whole_list:
                        whole_list.append(p)
                    elif p!= whole_list[-1]:
                        whole_list.append(p)
                self[k] = Polyline3D(lst)
            self.whole_polyline = Polyline3D(whole_list)

        else:
            print(polylines)
            print('-----------')
            print(ordered_keys)
            sys.exit('error : len(polylines) != len(ordered_keys) ')

    @property
    def is_closed(self):
        return self.whole_polyline.is_closed
                
    @property
    def is_planar(self):
        return self.whole_polyline.is_planar

    def express_in_frame_plane(self, frame):
        from rincewind.geometry.Polyline2D import Polyline2D, NamedPolyline
        polylines2d = []
        ordo = []
        for k in self.ordered_keys:
            polyline2d = Polyline2D(self[k].express_in_frame_plane(frame))
            polylines2d.append(polyline2d)
            ordo.append(k)
        return NamedPolyline(polylines2d, ordered_keys=ordo)


