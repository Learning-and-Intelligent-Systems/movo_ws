#!/usr/bin/env python
from scipy.spatial import ConvexHull
import numpy as np

corners= lambda ( (x1,y1), (x2,y2) ): [ (x1, y1), (x1, y2) , (x2, y2), (x2, y1) ]


""" 
    Convex Hull with helper functions 
"""
class CHull(ConvexHull):
    def __init__(self, points):
        ConvexHull.__init__(self, points)
        self.dim =range(self.points.shape[1])
        self.n = range(len(self.points))

    def centrum(self):
        c = [np.mean(self.points[self.vertices,i]) for i in self.dim]
        return c

    def get_vertices(self):
        return self.points[self.vertices]
    
    def translate(self, (x,y)): #XXX
        delta = np.array([x,y])
        points = [delta + self.points[i] for i in self.n] 
        return CHull(points)
   
    def rotate(self, theta):
        #raise NotImplementedError()
        cos, sin = np.cos(theta), np.sin(theta)
        R = np.matrix( [[cos, -sin], [sin, cos] ])
        #center around middle
        c = np.array(self.centrum())
        V_offset = [np.matrix(v - c).transpose()for v in self.points]
        V_rotated = [(R*v).flatten()  for v in V_offset]
        points =np.vstack( [v+c for v in V_rotated])
        #print " offset : \n\t%s\nrotated:\n\t%s\npoints:\n\t%s" % (V_offset, V_rotated, points)
        #ConvexHull.__init__(self, points)
        return CHull(points)

    def min_pts(self):
        return self.points.min(axis=0)
    
    def max_pts(self):
        return self.points.max(axis=0)
    
    def bounding_box(self):
        return np.array(corners(( self.min_pts(), self.max_pts())))

    def shape(self):
        return self.width(), self.height()

    def width(self):
        x1, _ = self.min_pts()
        x2, _ = self.max_pts()
        return x2-x1
    
    def height(self):
        _,y1 = self.min_pts()
        _,y2 = self.max_pts()
        return y2-y1

    def plot(self,ax,color="b", hatch="", alpha=.3 ):
        patch = Polygon(self.get_vertices(), facecolor=color, hatch=hatch, alpha=alpha)
        ax.add_patch(patch)


    def __str__(self):
        bb = self.bounding_box()
        w, h = self.width(), self.height()
        n = len(self.get_vertices())
        return "Convex Hull with %d vertices. w,h = %s,%s. Bounding box at %s" % (n, w, h, bb)

    # minkowski sum of two CHull shapes
    def minkowski_sum(self, A, difference=True):
        sign = -1 if difference else 1
        origin = np.array(self.centrum())
        Bverts = self.get_vertices()
        Averts = A.get_vertices()
        verts = np.vstack([Averts + sign*Bverts[i] + origin for i in range(len(Bverts))])
        return CHull(verts)

def union(A, B):
    Averts = A.points
    Bverts = B.points
    verts = np.vstack([Averts, Bverts])
    return CHull(verts)


