# -*- coding: utf-8 -*-
"""
Inverse model.

Content:
********
.. autosummary::
    :toctree: _autosummary

    softrobots.inverse.actuators
    softrobots.inverse.effectors

"""

__all__=["actuators","effectors"]

from Sofa import msg_error, msg_info

isAvailable = False

### This is searching for the private part of the plugin. In case this part is not installed
### then a message is printed.
try:
    from __softrobotsinverse__ import isAvailable
    msg_info("SoftRobots", "Loading 'softrobots.inverse' python module.")
    isAvailable = True
except:
    msg_error("SoftRobots", """Missing SoftRobots.Inverse.
This scene is using the SoftRobots.Inverse plugin which does not seem available on your system.
More infos at: https://project.inria.fr/softrobot/install-get-started-2/download/""")
