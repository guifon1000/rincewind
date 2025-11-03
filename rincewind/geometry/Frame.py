
from .Vector import Vector
from .Point import Point


class Frame(list):
    """
    if (T,N,B) is a frame placed in M, the given list is :
    [ 
      [Mx, My, Mz],
      [Tx, Ty, Tz],
      [Nx, Ny, Nz],
      [Bx, By, Bz] ]
    """
    def __init__(self, *largs):
        super(Frame, self).__init__(*largs)

    def add_to_ax(self,ax, scale = 1.):
        ax.scatter(self[0][0], self[0][1], self[0][2], c='r')
        ax.quiver(self[0][0], self[0][1], self[0][2], scale * self[1][0], scale * self[1][1], scale * self[1][2], color = 'r')
        ax.quiver(self[0][0], self[0][1], self[0][2], scale * self[2][0], scale * self[2][1], scale * self[2][2], color = 'b')
        ax.quiver(self[0][0], self[0][1], self[0][2], scale * self[3][0], scale * self[3][1], scale * self[3][2], color = 'g')



