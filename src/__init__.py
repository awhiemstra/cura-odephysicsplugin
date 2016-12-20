# Copyright (c) 2016 Ultimaker B.V.
# Cura is released under the terms of the AGPLv3 or higher.

from . import ODEPhysicsPlugin

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
        }
    }

def register(app):
    return { "extension": ODEPhysicsPlugin.ODEPhysicsPlugin() }

