""" PathSolver-v0.03.2
    Inputs:
        SOLVER: type solver "ByGravity" or "ByDirect" (str)
        START_POINT_LIST: start point for solver (Point3d: list)
        MESH: one mash for solver (mesh)
        -
        START_VEC_PROJ: point projection direction vector
                        at the start of the script (Vector3d: list)
                        Default value = (0,0,-1)
        START_VEC_DIR:  for "ByDirect". Starting direction for the
                        salt pan in the mode (Vector3d: list)
                        Default value = (1,0,0)
        -
        MOVE_DIST: moving a point relative to the normal to
                   the mesh surface (float)
                   Default value = 10
        N_STEP: number of steps (int)
                Default value = 300
        MIN_RAY_DIST: for "ByGravity". minimum distance between
                      points after which the execution will be stopped
                      Default value = 0.0
        MAX_RAY_DIST: for "ByDirect". Maximum distance between
                      points after which the execution will be stopped
                      Default value = 10.0
        RUN: on/off script. Default value = True
    Output:
        wayPoints: path points (Point3d: list)
        wayOutsidePoints: point at the edge of the surface (Point3d: list)
        wayNormal: normal from every point on the surface (Vector3d: list)
"""

__author__ = "AndriiPavlov"
__version__ = "2022.09.14"


import Rhino.Geometry as rg
import Rhino.UI as ui
import rhinoscriptsyntax as rs
import ghpythonlib.components as gh
import ghpythonlib.treehelpers as th
import math as ma
#import Rhino as r
#import copy
#import time
#import System
#import os

if not SOLVER:
    SOLVER = "ByGravity"

if not START_VEC_PROJ:
    START_VEC_PROJ = [rg.Vector3d(0,0,-1)]

if not START_VEC_DIR:
    START_VEC_DIR = [rg.Vector3d(1,0,0)]

if not MOVE_DIST:
    MOVE_DIST = 10

if not N_STEP:
    N_STEP = 300

if not MIN_RAY_DIST:
    MIN_RAY_DIST = 0.00

if not MAX_RAY_DIST:
    MAX_RAY_DIST = 10.0

if not RUN:
    RUN = True


def SameLengthList(list1, List2):
    """
    Збільшує List2 до розміру List1
    :param list1: - еталонний список
    :param List2: - той шо треба збільшити
    :return: - новій список
    """
    temp = []
    for i in range(len(list1)):
        temp.append(List2[-1])
    return temp

if len(START_POINT_LIST) != len(START_VEC_PROJ) and len(START_VEC_PROJ) == 1:
    START_VEC_PROJ = SameLengthList(START_POINT_LIST, START_VEC_PROJ)

if len(START_POINT_LIST) != len(START_VEC_DIR) and len(START_VEC_DIR) == 1:
    START_VEC_DIR = SameLengthList(START_POINT_LIST, START_VEC_DIR)


def pointDistans(p1, p2):
    """
    Distance between points
    :param p1: point #1
    :param p2: point #2
    :return: - distance
    """
    return ma.sqrt((p1.X - p2.X)**2 + (p1.Y - p2.Y)**2 + (p1.Z - p2.Z)**2)



class Solver():

    def __init__(self, startPoint, startVectorProj, startVectorDirection):
        self.startPoint = startPoint
        self.vecProj = startVectorProj
        self.vecDirection = startVectorDirection

        self.moveDist = MOVE_DIST
        self.mesh = MESH
        self.wayPointsList = []
        self.wayOutsidePoints = []
        self.wayNormal = []

    def GetWayPointList(self):
        return self.wayPointsList

    def GetWayOutsidePoints(self):
        return self.wayOutsidePoints

    def GetWayNormal(self):
        return self.wayNormal

    def SetVecProj(self, vec):
        self.vecProj = vec

    def SetVecDirection(self, vec):
        self.vecDirection = vec

    def StepOne(self, point):
        ray = rg.Intersect.Intersection.MeshRay(self.mesh, rg.Ray3d(point, self.vecProj))
        rayPoint = rg.Point3d(point.X + self.vecProj.X * ray,
                              point.Y + self.vecProj.Y * ray,
                              point.Z + self.vecProj.Z * ray
                              )
        if ray < 0:
            if self.wayPointsList:
                self.wayOutsidePoints.append(self.wayPointsList[-1])
            return None

        if len(self.wayPointsList) > 2:
            if pointDistans(rayPoint, self.wayPointsList[-1]) < MIN_RAY_DIST:
                return None

        self.wayPointsList.append(rayPoint)

        meshPoint = self.mesh.ClosestMeshPoint(rayPoint, 100000.0)
        normal = self.mesh.NormalAt(meshPoint)
        normal.Unitize()

        self.wayNormal.append(normal)

        ptMoveBNormal = rg.Point3d(rayPoint.X + normal.X * self.moveDist,
                                   rayPoint.Y + normal.Y * self.moveDist,
                                   rayPoint.Z + normal.Z * self.moveDist
                                   )
        return ptMoveBNormal

    def StepOneByDirect(self, point):
        ray = rg.Intersect.Intersection.MeshRay(self.mesh, rg.Ray3d(point, self.vecProj))
        rayPoint = rg.Point3d(point.X + self.vecProj.X * ray,
                              point.Y + self.vecProj.Y * ray,
                              point.Z + self.vecProj.Z * ray
                              )
        if ray < 0:
            if self.wayPointsList:
                self.wayOutsidePoints.append(self.wayPointsList[-1])
            return None

        # if len(self.wayPointsList) > 2:
        #     if pointDistans(rayPoint, self.wayPointsList[-1]) < MIN_RAY_DIST:
        #         return None

        if len(self.wayPointsList) > 2:
            if pointDistans(rayPoint, self.wayPointsList[-1]) >= MAX_RAY_DIST:
                return None

        self.wayPointsList.append(rayPoint)

        meshPoint = self.mesh.ClosestMeshPoint(rayPoint, 100000.0)
        normal = self.mesh.NormalAt(meshPoint)
        normal.Unitize()
        self.wayNormal.append(normal)

        normalVecRev = rg.Vector3d(normal.X, normal.Y, normal.Z)
        normalVecRev.Reverse()
        self.SetVecProj(rg.Vector3d(normalVecRev.X,
                                    normalVecRev.Y,
                                    normalVecRev.Z
                                    ))

        normAndDir = rg.Vector3d(normal.X + self.vecDirection.X,
                                 normal.Y + self.vecDirection.Y,
                                 normal.Z + self.vecDirection.Z
                                 )

        normAndDir.Unitize()

        ptMoveBNormal = rg.Point3d(rayPoint.X + normAndDir.X * self.moveDist,
                                   rayPoint.Y + normAndDir.Y * self.moveDist,
                                   rayPoint.Z + normAndDir.Z * self.moveDist
                                   )

        if len(self.wayPointsList) > 3:
            tempPt_1 = self.wayPointsList[-1]
            tempPt_2 = self.wayPointsList[-3]
            self.SetVecDirection(rg.Vector3d(tempPt_1.X - tempPt_2.X,
                                             tempPt_1.Y - tempPt_2.Y,
                                             tempPt_1.Z - tempPt_2.Z
                                             ))

        return ptMoveBNormal

    def SolverForMultiplePoints(self, startPt, nStep):
        tempPt = startPt
        for i in range(nStep):
            if tempPt:
                if SOLVER == "ByGravity":
                    tempPt = self.StepOne(tempPt)
                if SOLVER == "ByDirect":
                    tempPt = self.StepOneByDirect(tempPt)
            else:
                break



wayPoints = []
wayOutsidePoints = []
wayNormal = []

if RUN:
    for i in range(len(START_POINT_LIST)):
        solver = Solver(START_POINT_LIST[i], START_VEC_PROJ[i], START_VEC_DIR[i])
        solver.SolverForMultiplePoints(START_POINT_LIST[i], N_STEP)

        wayPoints.append(solver.GetWayPointList())
        wayOutsidePoints.append(solver.GetWayOutsidePoints())
        wayNormal.append(solver.GetWayNormal())


wayPoints = th.list_to_tree(wayPoints)
wayOutsidePoints = th.list_to_tree(wayOutsidePoints)
wayNormal = th.list_to_tree(wayNormal)
