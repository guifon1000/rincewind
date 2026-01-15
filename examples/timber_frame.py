from rincewind.geometry.Polyline2D import Polyline2D, NamedPolyline

# concrete slab at z = 0
slab = Polyline2D([[0., 0.], [10., 0.], [10., 10.], [0., 10.], [0., 0.]])
slab.z = 0.


spacing = 0.6
seg = slab.segments

for s in seg:
    l = round(int(s.length / spacing))
    print (l)