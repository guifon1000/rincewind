from rincewind.analytic_functions.bezier_functions import piecewise_bezier_polyline
import matplotlib.pyplot as plt
ri = 0.005
lc = 0.5
lt = 0.25
r0 = 0.25
rt = 0.012


f = piecewise_bezier_polyline(0., 0. , [[0.,r0,1.],[1.-lc-lt,r0,1.], [1.-lt,r0,1.], [1.-lt,rt,1.],[1.,rt,1.]])


N = 1000
x = []
y = []
for i in range(N):
    s = float(i)/float(N-1)
    x.append(s)
    y.append(f(s))

plt.plot(x,y)
plt.axis('equal')
plt.show()

