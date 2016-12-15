import ode

from UM.Scene.SceneNodeDecorator import SceneNodeDecorator
from UM.Scene.SceneNode import SceneNode
from UM.Math.Vector import Vector

class RigidBodyDecorator(SceneNodeDecorator):
    def __init__(self, world, space, width = 10, height = 10, depth = 10):
        super().__init__()

        self._world = world
        self._space = space

        self._body = ode.Body(self._world)
        mass = ode.Mass()
        mass.setBox(1000.0, width / 1000, height / 1000, depth / 1000)
        self._body.setMass(mass)

        self._geom = ode.GeomBox(self._space, lengths = (width / 1000, height  / 1000, depth / 1000))
        self._geom.setBody(self._body)

    def setNode(self, node):
        super().setNode(node)

        position = node.getWorldPosition()
        self._body.setPosition( (position.x / 1000, position.y / 1000, position.z / 1000) )

        #node.transformationChanged.connect(self._onTransformationChanged)

    def updateFromODE(self):
        body_position = self._body.getPosition()
        self.getNode().setPosition(Vector(body_position[0] * 1000, body_position[1] * 1000, body_position[2] * 1000), SceneNode.TransformSpace.World)

        body_orientation = self._body.getRotation()

    #def _onTransformationChanged(self):
        #pass
        #position = self.getNode().getWorldPosition()

        #self._body.setPosition( (position.x, position.y, position.z) )
