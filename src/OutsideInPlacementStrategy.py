from enum import IntEnum

from UM.Application import Application
from UM.Scene.Iterator.DepthFirstIterator import DepthFirstIterator

from UM.Math.Vector import Vector

from . import Helpers
from . import RigidBodyDecorator

class OutsideInPlacementStrategy:
    def __init__(self, physics_plugin):
        self._physics_plugin = physics_plugin

        self._x = None
        self._z = None

    def placeNodes(self, new_nodes):
        fixed_nodes = []
        for node in DepthFirstIterator(Application.getInstance().getController().getScene().getRoot()):
            if node.hasDecoration("setFixed"):
                fixed_nodes.append(node)
                node.callDecoration("setFixed", True)

        for node in new_nodes:
            node.addDecorator(RigidBodyDecorator.RigidBodyDecorator(self._physics_plugin.world, self._physics_plugin.space))

        if self._x is None or self._z is None:
            self._x = Application.getInstance().getGlobalContainerStack().getProperty("machine_width", "value") / 2
            self._z = Application.getInstance().getGlobalContainerStack().getProperty("machine_depth", "value") / 2

        for node in new_nodes:
            body = node.callDecoration("getBody")
            if not body:
                continue

            body.setPosition( Helpers.toODE( Vector(self._x, 0, self._z) ) )

            if self._x > 0 and self._z < 0:
                self._z = -self._z
            elif self._x > 0:
                self._x = -self._x
            elif self._z > 0:
                self._z = -self._z
            else:
                self._x = -self._x

        for i in range(1000):
            for node in new_nodes:
                body = node.callDecoration("getBody")
                direction = Helpers.fromODE(body.getPosition()).normalized() * -1
                body.addForce( Helpers.toODE( direction * 100000000 ) )

            self._physics_plugin.runSimulation()

        for i in range(100):
            self._physics_plugin.runSimulation()

        for node in fixed_nodes:
            node.callDecoration("setFixed", False)
