import ode

from PyQt5.QtCore import QTimer

from UM.Application import Application
from UM.Extension import Extension
from UM.JobQueue import JobQueue
from UM.Math.Vector import Vector
from UM.Mesh.ReadMeshJob import ReadMeshJob
from UM.Scene.Iterator.DepthFirstIterator import DepthFirstIterator

from .RigidBodyDecorator import RigidBodyDecorator
from .Helpers import fromODE, toODE

class ODEPhysicsPlugin(Extension):
    def __init__(self):
        super().__init__()

        JobQueue.getInstance().jobFinished.connect(self._onJobFinished)

        self._world = ode.World()
        self._world.setGravity( (0, toODE(-98100), 0) )
        self._world.setERP(0.2)
        self._world.setCFM(1e-5)

        #self._world.setContactMaxCorrectingVel(toODE(1000))
        self._world.setContactSurfaceLayer(0.001)

        self._world.setLinearDamping(1)
        #self._world.setAngularDamping(0)

        self._world.setAutoDisableLinearThreshold(0.001)
        #self._world.setAutoDisableTime(0.1)

        self._space = ode.Space(space_type = 1)

        self._contact_group = ode.JointGroup()

        self._contact_bounce = 0.0
        #self._contact_friction = float("inf")
        self._contact_friction = 0

        self._initial_positioning = False

        self._build_plate = ode.GeomPlane(self._space, (0, 1, 0), 0)

        self._update_interval = 20
        self._steps_per_update = 10

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

        fixed_nodes = []
        for node in DepthFirstIterator(Application.getInstance().getController().getScene().getRoot()):
            if node.hasDecoration("setFixed"):
                fixed_nodes.append(node)
                node.callDecoration("setFixed", True)

        for node in nodes:
            node.addDecorator(RigidBodyDecorator(self._world, self._space))

        #self._world.setLinearDamping(0)
        #self._contact_bounce = 1
        #self._world.setContactMaxCorrectingVel(toODE(1000))
        self._initial_positioning = True

        for i in range(1000):
            self._space.collide( (self._world, self._contact_group), self._onCollision )
            self._world.quickStep( (self._update_interval / 1000) / self._steps_per_update )
            self._contact_group.empty()

            #for node in nodes:
                #geom = node.callDecoration("getGeom")
                #if geom and hasattr(geom, "collision_count"):

                    #force = node.getWorldPosition().normalized() * (geom.collision_count * 10000000)
                    #node.callDecoration("getBody").addForce( (force.x, force.y, force.z) )

        #self._world.setLinearDamping(1)
        #self._world.setContactMaxCorrectingVel(float("inf"))
        #self._contact_bounce = 0

        self._initial_positioning = False

        #for node in fixed_nodes:
            #node.callDecoration("setFixed", False)

    def _onCollision(self, args, geom1, geom2):
        contacts = ode.collide(geom1, geom2)

        world, contact_group = args
        for c in contacts:
            c.setBounce(self._contact_bounce)
            c.setMu(self._contact_friction)
            j = ode.ContactJoint(world, contact_group, c)
            j.attach(geom1.getBody(), geom2.getBody())

            #if self._initial_positioning:
                #for geom in (geom1, geom2):
                    #if geom.getBody() and not geom.getBody().isKinematic():
                        #if not hasattr(geom, "collision_count"):
                            #geom.collision_count = 0
                        #geom.collision_count += 1

    def _onTimer(self):
        dt = self._update_interval / 1000

        for i in range(self._steps_per_update):
            self._space.collide( (self._world, self._contact_group), self._onCollision )

            self._world.quickStep(dt / self._steps_per_update)

            self._contact_group.empty()

        for node in DepthFirstIterator(Application.getInstance().getController().getScene().getRoot()):
            node.callDecoration("updateFromODE")
