project(cura-odephysicsplugin NONE)

cmake_minimum_required(VERSION 2.8.12)

install(FILES
    src/__init__.py
    src/ODEPhysicsPlugin.py
    src/RigidBodyDecorator.py
    src/Helpers.py
    src/OutsideInPlacementStrategy.py
    src/TranslateTool.py
    src/TranslateToolHandle.py
    src/TranslateTool.qml
    src/ObjectPacker.py
    DESTINATION lib/cura/plugins/ode-physicsplugin
)
