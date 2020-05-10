from __future__ import print_function

import argparse
import collections
import datetime
import glob
import logging
import math
import os
import random
import re
import sys
import weakref

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError(
        'cannot import numpy, make sure numpy package is installed')

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==============================================================================
# -- add PythonAPI for release mode --------------------------------------------
# ==============================================================================
try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass

import carla
from carla import ColorConverter as cc

class GenConfig(object):
    def __init__(self):
        self.width = 320
        self.height = 240
        self.verbose = True
        self.host = '127.0.0.1'
        self.port = 2000
        self.filter = 'vehicle.*'
        self.town = 'Town01'
        
class SensorsConfig(object):
    def __init__(self):
        self.lidar_location = carla.Location(0,0,2)
        self.camera_transforms = [
            carla.Transform(carla.Location(x=2.5, y=0, z=0.5)),
            carla.Transform(carla.Location(x=0, y=1.2,z=1),  carla.Rotation(yaw=90)),
            carla.Transform(carla.Location(x=-2.5,y=0, z=1),  carla.Rotation(yaw=180)),
            carla.Transform(carla.Location(x=0, y=-1.2, z=1),  carla.Rotation(yaw=-90))
            ]        

class WeatherConfig(object):
    def __init__(self):
        self.cloudiness = 10.0
        self.precipitation = 10.0
        self.precipitation_deposits = 10.0
        self.wind_intensity = 10.0
        self.fog_density = 10.0
        self.fog_distance = 100.0
        self.wetness = 10.0
        self.sun_azimuth_angle = 40.0
        self.sun_altitude_angle = 90.0

        
class SpawnerConfig(object):
    def __init__(self):
        self.host = '127.0.0.1'
        self.port = 2000
        self.number_of_vehicles = 10
        self.number_of_walkers = 10
        self.safe = True
        self.filterv = 'vehicle.*'
        self.filterw = 'walker.pedestrian.*'
        self.tm_port = 8000
        self.sync = True