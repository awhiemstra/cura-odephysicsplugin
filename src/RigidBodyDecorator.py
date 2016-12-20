import ode
import numpy

from UM.Scene.SceneNodeDecorator import SceneNodeDecorator
from UM.Scene.SceneNode import SceneNode
from UM.Math.Vector import Vector
from UM.Math.Quaternion import Quaternion
from UM.Math.Color import Color

from UM.Mesh.MeshBuilder import MeshBuilder

from .Helpers import ScaleFactor, fromODE, toODE

class RigidBodyDecorator(SceneNodeDecorator):
    def __init__(self, world, space):
        super().__init__()

        self._world = world
        self._space = space
        self._body = None
        self._geom = None
        self._update_from_ode = False

    def setNode(self, node):
        super().setNode(node)

        aabb = node.getBoundingBox()
        if not aabb.isValid():
            return

        if not self._body:
            self._body = ode.Body(self._world)
            mass = ode.Mass()
            mass.setBox(1000.0, toODE(aabb.width), toODE(aabb.height), toODE(aabb.depth))
            self._body.setMass(mass)


        if not self._geom:
            if node.getMeshData():
                mesh = node.getMeshData()

                self._trimesh = ode.TriMeshData()

                #hull = mesh.getConvexHull()

                vertices = mesh.getVertices() / ScaleFactor
                #indices = mesh.getIndices() if mesh.hasIndices() else numpy.fliplr(numpy.arange(len(vertices), dtype = numpy.int32).reshape((len(vertices) // 3, 3)))
                indices = mesh.getConvexHull().simplices

                self._trimesh.build(vertices, indices)

                self._geom = ode.GeomTriMesh(self._trimesh, self._space)

                mb = MeshBuilder()

                for i in range(self._geom.getTriangleCount()):
                    tri = self._geom.getTriangle(i)
                    mb.addFace(
                        v0 = fromODE(tri[0]),
                        v1 = fromODE(tri[1]),
                        v2 = fromODE(tri[2]),
                        color = Color(1.0, 0.0, 0.0, 1.0)
                    )

                node.setMeshData(mb.build())
            else:
                self._geom = ode.GeomBox(self._space, lengths = (toODE(aabb.width), toODE(aabb.height), toODE(aabb.depth)))

            self._geom.setBody(self._body)

        self._body.setPosition(toODE(node.getWorldPosition()))

        node.transformationChanged.connect(self._onTransformationChanged)

    def updateFromODE(self):
        self._update_from_ode = True

        self.getNode().setPosition(fromODE(self._body.getPosition()), SceneNode.TransformSpace.World)

        body_orientation = self._body.getQuaternion()
        self.getNode().setOrientation(Quaternion(body_orientation[3], body_orientation[0], body_orientation[1], body_orientation[2]))

        self._update_from_ode = False

    def _onTransformationChanged(self, node):
        if self._update_from_ode:
            return

        self._body.setPosition(toODE(self.getNode().getWorldPosition()))
