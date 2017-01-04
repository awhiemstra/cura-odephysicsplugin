# Copyright (c) 2016 Ultimaker B.V.
# Cura is released under the terms of the AGPLv3 or higher.

from UM.Application import Application

from . import ODEPhysicsPlugin
from . import TranslateTool

from UM.i18n import i18nCatalog
catalog = i18nCatalog("cura")

def getMetaData():
    return {
        "plugin": {
            "name": catalog.i18nc("@label", "Platform Phyisics Plugin"),
            "author": "Ultimaker",
            "version": "1.0",
            "description": catalog.i18nc("@info:whatsthis", "Performs object collision and other platform physics related tasks."),
            "api": 3
        },
        "tool": {
            "name": "Move",
            "description": "Move Model",
            "icon": "translate",
            "tool_panel": "TranslateTool.qml",
            "weight": -1
        }
    }

def register(app):
    # Hack that allows us to replace the existing translate tool
    try:
        Application.getInstance().getController()._tools["TranslateTool"] = TranslateTool.TranslateTool()
    except KeyError:
        pass

    return { "extension": ODEPhysicsPlugin.ODEPhysicsPlugin() }

