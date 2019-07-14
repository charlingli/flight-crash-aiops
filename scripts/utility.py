import datetime
import numpy as np
import os
import sys
import time
import yaml

dir_path = os.path.dirname(os.path.realpath(__file__))
settings_file = open(dir_path + '/settings.yml', 'r')
settings_conf = yaml.load(settings_file, Loader=yaml.CLoader)
settings_file.close()

def getFlightTime(startTime):
    simulationLength = settings_conf['simulation']['length']
    timeNow = time.time() - startTime
    timeDelta = datetime.timedelta(seconds=timeNow % simulationLength)
    return getTimeInSeconds(timeDelta)

def getTimeInSeconds(inputTime):
    timeSplit = np.array(list(map(int, str(inputTime).split(".")[0].split(":"))))
    seconds = timeSplit[0] * 3600 + timeSplit[1] * 60 + timeSplit[2]
    return seconds

def getFailureVector():
    return settings_conf['model']['failure']

# def getFailureProbability():
    # Uses a superposition of two Gaussian functions (one near the start, one near the end), to get the probability of a 

def getTopology():
    return settings_conf['model'][settings_conf['model']['type']]
    
if __name__ == "__main__":
    # Debugging for when this is executed as a script
    getTimeInSeconds("0:01:55.005707")