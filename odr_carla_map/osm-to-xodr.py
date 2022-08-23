import glob
import os
import sys
import time

try:
    sys.path.append('~/CARLA_0.9.11/PythonAPI/carla/dist/carla-0.9.11-py3.7-linux-x86_64.egg')
except IndexError:
    pass

import carla

def process(inputname, outputname):
    # Read the .osm data
    f = open(inputname, 'r')
    osm_data = f.read()
    f.close()

    # Define the desired settings. In this case, default values.
    settings = carla.Osm2OdrSettings()
    # Convert to .xodr
    xodr_data = carla.Osm2Odr.convert(osm_data, settings)

    # save opendrive file
    f = open(outputname, 'w')
    f.write(xodr_data)
    f.close()

if __name__ == '__main__':

    try:
        process(sys.argv[1], sys.argv[2])
    except KeyboardInterrupt:
        pass
    except:
    	print("Usage: python<3>", sys.argv[0], "/path/to/input/file.osm", "/path/to/output/file.xodr")
    finally:
        print('\ndone.')
