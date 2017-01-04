import ode
import numpy

from UM.Scene.SceneNodeDecorator import SceneNodeDecorator
from UM.Scene.SceneNode import SceneNode
from UM.Math.Vector import Vector
from UM.Math.Quaternion import Quaternion
from UM.Math.Color import Color
from UM.Math.Matrix import Matrix

from UM.Mesh.MeshBuilder import MeshBuilder

from . import Helpers

class RigidBodyDecorator(SceneNodeDecorator):
    def __init__(self, world, space):
        super().__init__()

        self._world = world
        self._space = space
        self._body = None
        self._geom = None
        self._update_from_ode = False

        self._fixed = False

    def setNode(self, node):
        super().setNode(node)

        aabb = node.getBoundingBox()
        if not aabb.isValid():
            return

        if not self._body:
            self._body = ode.Body(self._world)
            self._body.setMaxAngularSpeed(0)
            mass = ode.Mass()
            mass.setBox(5.0, Helpers.toODE(aabb.width), Helpers.toODE(aabb.height), Helpers.toODE(aabb.depth))
            self._body.setMass(mass)

        if not self._geom:
            if node.getMeshData():
                scale_matrix = Matrix()
                scale_matrix.setByScaleFactor(1.01)
                mesh = node.getMeshData().getTransformed(scale_matrix)

                self._trimesh = ode.TriMeshData()

                debug_builder = MeshBuilder()

                vertices = mesh.getVertices()
                indices = mesh.getConvexHull().simplices

                _fixWindingOrder(vertices, indices, debug_builder)

                self._trimesh.build(vertices / Helpers.ScaleFactor, indices)

                self._geom = ode.GeomTriMesh(self._trimesh, self._space)

                mb = MeshBuilder()

                for i in range(self._geom.getTriangleCount()):
                    tri = self._geom.getTriangle(i)

                    v0 = Helpers.fromODE(tri[0])
                    v1 = Helpers.fromODE(tri[1])
                    v2 = Helpers.fromODE(tri[2])

                    mb.addFace(v0 = v0, v1 = v1, v2 = v2, color = Color(1.0, 0.0, 0.0, 0.5))

                chn = SceneNode(node)
                chn.setMeshData(mb.build())

                def _renderConvexHull(renderer):
                    renderer.queueNode(chn, transparent = True)
                    return True
                chn.render = _renderConvexHull

                n = SceneNode(node)
                n.setMeshData(debug_builder.build())

                def _renderNormals(renderer):
                    renderer.queueNode(n, mode = 1, overlay = True)
                    return True
                n.render = _renderNormals
            else:
                self._geom = ode.GeomBox(self._space, lengths = (Helpers.toODE(aabb.width), Helpers.toODE(aabb.height), Helpers.toODE(aabb.depth)))

            self._geom.setBody(self._body)

        self._body.setPosition(Helpers.toODE(node.getWorldPosition()))

        node.transformationChanged.connect(self._onTransformationChanged)

    def getBody(self):
        return self._body

    def getGeom(self):
        return self._geom

    def updateFromODE(self):
        self._update_from_ode = True

        self.getNode().setPosition(Helpers.fromODE(self._body.getPosition()), SceneNode.TransformSpace.World)

        body_orientation = self._body.getQuaternion()
        self.getNode().setOrientation(Quaternion(body_orientation[3], body_orientation[0], body_orientation[1], body_orientation[2]))

        self._update_from_ode = False

    def setFixed(self, fixed):
        if fixed != self._fixed:
            self._fixed = fixed

            self._body.setGravityMode(not self._fixed)

            if self._fixed:
                self._body.setKinematic()
            else:
                self._body.setDynamic()

    def toODE(self, value):
        return Helpers.toODE(value)

    def _onTransformationChanged(self, node):
        if self._update_from_ode:
            return

        self._body.setPosition(Helpers.toODE(self.getNode().getWorldPosition()))

def _fixWindingOrder(vertices, indices, debug_builder):
    for i in range(len(indices)):
        v0 = vertices[indices[i][0]]
        v1 = vertices[indices[i][1]]
        v2 = vertices[indices[i][2]]

        normal = _calculateNormal(v0, v1, v2)

        center = (v0 + v1 + v2) / 3

        direction = center / numpy.linalg.norm(center)

        debug_builder.addLine(Vector(data = center), Vector(data = center + direction * 10), Color(0.0, 0.0, 1.0, 1.0))
        debug_builder.addLine(Vector(data = center), Vector(data = center + normal * 10), Color(0.0, 1.0, 0.0, 1.0))

        if normal.dot(direction) < 0.1:
            indices[i][0], indices[i][2] = indices[i][2], indices[i][0]
            new_normal = _calculateNormal(v2, v1, v0)
            debug_builder.addLine(Vector(data = center), Vector(data = center + new_normal * 10), Color(1.0, 0.0, 0.0, 1.0))

def _calculateNormal(v0, v1, v2):
    edge0 = v0 - v1
    edge1 = v0 - v2

    normal = numpy.cross(edge0, edge1)
    length = numpy.linalg.norm(normal)

    if length > 0:
        normal = normal / length

    return normal
