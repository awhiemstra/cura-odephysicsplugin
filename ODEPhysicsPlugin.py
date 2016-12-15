import ode

from PyQt5.QtCore import QTimer

from UM.Application import Application
from UM.Extension import Extension
from UM.JobQueue import JobQueue
from UM.Math.Vector import Vector
from UM.Mesh.ReadMeshJob import ReadMeshJob
from UM.Scene.Iterator.DepthFirstIterator import DepthFirstIterator

from .RigidBodyDecorator import RigidBodyDecorator

class ODEPhysicsPlugin(Extension):
    def __init__(self):
        super().__init__()

        JobQueue.getInstance().jobFinished.connect(self._onJobFinished)

        self._world = ode.World()
        self._world.setGravity( (0, -9.81, 0) )
        self._world.setERP(0.8)
        self._world.setCFM(1e-5)

        self._space = ode.Space()

        self._contact_group = ode.JointGroup()

        self._build_plate = ode.GeomPlane(self._space, (0, 1, 0), 0)

        self._update_interval = 10

        self._timer = QTimer()
        self._timer.setInterval(20)
        self._timer.timeout.connect(self._onTimer)
        self._timer.start()

    def _onJobFinished(self, job):
        if not isinstance(job, ReadMeshJob):
            return

        nodes = job.getResult()
        if not nodes:
            return

        for node in nodes:
            aabb = node.getBoundingBox()
            print(aabb)
            node.translate(Vector(0, 100, 0))
            node.addDecorator(RigidBodyDecorator(self._world, self._space, aabb.width, aabb.height, aabb.depth))

    def _onCollision(self, args, geom1, geom2):
        contacts = ode.collide(geom1, geom2)

        world, contact_group = args
        for c in contacts:
            c.setBounce(0)
            c.setMu(float("inf"))
            j = ode.ContactJoint(world, contact_group, c)
            j.attach(geom1.getBody(), geom2.getBody())

    def _onTimer(self):
        for i in range(2):
            self._space.collide( (self._world, self._contact_group), self._onCollision )

            self._world.step((self._update_interval/1000) / 2)

            self._contact_group.empty()

        for node in DepthFirstIterator(Application.getInstance().getController().getScene().getRoot()):
            node.callDecoration("updateFromODE")
