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
            mass.setBox(5.0, toODE(aabb.width), toODE(aabb.height), toODE(aabb.depth))
            self._body.setMass(mass)


        if not self._geom:
            if node.getMeshData():
                mesh = node.getMeshData()

                self._trimesh = ode.TriMeshData()

                debug_builder = MeshBuilder()

                #hull = mesh.getConvexHull()

                vertices = mesh.getVertices()
                #indices = mesh.getIndices() if mesh.hasIndices() else numpy.fliplr(numpy.arange(len(vertices), dtype = numpy.int32).reshape((len(vertices) // 3, 3)))
                indices = mesh.getConvexHull().simplices

                self._fixWindingOrder(vertices, indices, debug_builder)

                self._trimesh.build(vertices / ScaleFactor, indices)

                self._geom = ode.GeomTriMesh(self._trimesh, self._space)

                mb = MeshBuilder()

                for i in range(self._geom.getTriangleCount()):
                    tri = self._geom.getTriangle(i)

                    v0 = fromODE(tri[0])
                    v1 = fromODE(tri[1])
                    v2 = fromODE(tri[2])

                    #normal = calculateNormal(v0, v1, v2)

                    mb.addFace(v0 = v0, v1 = v1, v2 = v2, color = Color(1.0, 0.0, 0.0, 0.5))

                    #for n in range(3):
                        #mb.addLine(fromODE(tri[n]), fromODE(tri[n]) + (normal * 10), Color(0.0, 1.0, 0.0, 1.0))

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

    def _fixWindingOrder(self, vertices, indices, debug_builder):
        print(indices)
        for i in range(len(indices)):
            v0 = vertices[indices[i][0]]
            v1 = vertices[indices[i][1]]
            v2 = vertices[indices[i][2]]

            print("v0", v0)
            print("v1", v1)
            print("v2", v2)

            normal = calculateNormal(v0, v1, v2)

            #debug_builder.addLine(Vector(data = v0), Vector(data = v0 + normal * 10), Color(0.0, 1.0, 0.0, 1.0))
            #debug_builder.addLine(Vector(data = v1), Vector(data = v1 + normal * 10), Color(0.0, 1.0, 0.0, 1.0))
            #debug_builder.addLine(Vector(data = v2), Vector(data = v2 + normal * 10), Color(0.0, 1.0, 0.0, 1.0))

            print("normal", normal)

            center = (v0 + v1 + v2) / 3

            print("center", center)

            direction = center / numpy.linalg.norm(center)

            debug_builder.addLine(Vector(data = center), Vector(data = center + direction * 10), Color(0.0, 0.0, 1.0, 1.0))
            debug_builder.addLine(Vector(data = center), Vector(data = center + normal * 10), Color(0.0, 1.0, 0.0, 1.0))

            print("direction", direction)
            print("dot", normal.dot(direction))

            if normal.dot(direction) < 0.5:
                print("flipping winding order of index entry", i)
                indices[i][0], indices[i][2] = indices[i][2], indices[i][0]
                new_normal = calculateNormal(v2, v1, v0)
                debug_builder.addLine(Vector(data = center), Vector(data = center + new_normal * 10), Color(1.0, 0.0, 0.0, 1.0))




        print(indices)

def calculateNormal(v0, v1, v2):
    edge0 = v0 - v1
    edge1 = v0 - v2

    normal = numpy.cross(edge0, edge1)
    normal = normal / numpy.linalg.norm(normal)

    return normal

