import math
import ode
import random

from PyQt5.QtCore import QTimer

from UM.Application import Application
from UM.Message import Message
from UM.Scene.Iterator.DepthFirstIterator import DepthFirstIterator
from UM.Math.Vector import Vector

from . import RigidBodyDecorator

class ObjectPacker:
    def __init__(self, physics_plugin):
        self._physics_plugin = physics_plugin

        self._running = False

        self._message = None

        self._planes = []

        self._current_iteration = 0
        self._max_iterations = 100
        self._initial_plane_distance = -500
        self._min_plane_distance = -100

        self._timer = QTimer()
        self._timer.setInterval(20)
        self._timer.timeout.connect(self._update)

    def pack(self):
        if self._running:
            return

        self._message = Message("Packing Objects...", lifetime = -1, dismissable = False, progress = 100)
        self._message.show()

        Application.getInstance().getController().setToolsEnabled(False)

        plane = ode.GeomPlane(self._physics_plugin.space, (1, 0, 0), self._initial_plane_distance)
        self._planes.append(plane)

        plane = ode.GeomPlane(self._physics_plugin.space, (-1, 0, 0), self._initial_plane_distance)
        self._planes.append(plane)

        plane = ode.GeomPlane(self._physics_plugin.space, (0, 0, 1), self._initial_plane_distance)
        self._planes.append(plane)

        plane = ode.GeomPlane(self._physics_plugin.space, (0, 0, -1), self._initial_plane_distance)
        self._planes.append(plane)

        for node in DepthFirstIterator(Application.getInstance().getController().getScene().getRoot()):
            if node.getDecorator(RigidBodyDecorator.RigidBodyDecorator):
                random_position = Vector( random.randrange(self._initial_plane_distance, -self._initial_plane_distance), 0, random.randrange(self._initial_plane_distance, -self._initial_plane_distance) )
                node.setPosition( random_position )

        self._timer.start()

    def _update(self):
        for plane in self._planes:
            params = plane.getParams()
            distance = max(self._min_plane_distance, self._initial_plane_distance - (self._initial_plane_distance * (self._current_iteration / self._max_iterations)))
            plane.setParams(params[0], distance)

        self._current_iteration += 1
        if self._current_iteration > self._max_iterations:
            self._timer.stop()
            self._message.hide()
            Application.getInstance().getController().setToolsEnabled(True)

            for plane in self._planes:
                self._physics_plugin.space.remove(plane)
            self._planes = []

            self._current_iteration = 0
        else:
            self._message.setProgress((self._current_iteration / self._max_iterations) * 100)
