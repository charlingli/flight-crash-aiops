import datetime
import os
import sys
import yaml

dir_path = os.path.dirname(os.path.realpath(__file__))
settings_file = open(dir_path + '/settings.yml', 'r')
settings_conf = yaml.load(settings_file, Loader=yaml.CLoader)
settings_file.close()

def getFlightTime():
    simulation_length = settings_conf['simulation']['length']
    time_now = datetime.datetime.now()
    return datetime.timedelta(seconds=time_now.second % simulation_length)

def getFailureVector():
    return settings_conf['model']['failure']

# def getFailureProbability():
    # Uses a superposition of two Gaussian functions (one near the start, one near the end), to get the probability of a 

def getTopology():
    return settings_conf['model'][settings_conf['model']['type']]