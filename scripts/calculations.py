import datetime
import json
import matplotlib.pyplot as plt
import numpy as np
import os
import sys
import time
import yaml

dir_path = os.path.dirname(os.path.realpath(__file__))
settings_file = open(os.path.join(dir_path, 'settings.yml'), 'r')
settings_conf = yaml.load(settings_file, Loader=yaml.CLoader)
settings_file.close()

simulation_length = settings_conf['simulation']['length']
simulation_multiplier = settings_conf['simulation']['multiplier']

# simulation_failure_type = settings_conf['failure'][np.random.randint(len(settings_conf['failure']))]
# simulation_failure_time = np.random.randint(np.floor(simulation_length / 5), np.floor(simulation_length * 4 / 5))

data_flight_time = np.arange(simulation_length)

## Utility functions used in metric calculations
data_coefficient_lift = lambda angle_of_attack: (angle_of_attack / 10) + (2.4 - 2 * angle_of_attack / 9.6) / (1 + np.exp(7.5 - angle_of_attack / 2))

test_data_altitude = np.linspace(0, 12, 120)
data_temperature = lambda altitude: 288.15 - 12.25 * altitude
data_density = lambda altitude: 1.225 * np.power(data_temperature(altitude) / (data_temperature(altitude) + 0.02 * altitude), 1 + 9.807 * 12.25 / (8.314 * 0.02))

data_coefficient_drag_induced = lambda angle_of_attack: np.power(data_coefficient_lift(angle_of_attack), 2) / (np.pi * settings_conf['c172']['aspectratio'])
data_coefficient_drag_parasite = settings_conf['c172']['parasitedrag']

## Define FBD model equations
def calculateLift(altitude, airspeed, angle_of_attack):
    return 0.5 * data_density(altitude) * np.power(airspeed, 2) * settings_conf['c172']['surfacearea'] * data_coefficient_lift(angle_of_attack)

def calculateDrag(altitude, airspeed, angle_of_attack):
    return 0.5 * data_density(altitude) * np.power(airspeed, 2) * settings_conf['c172']['surfacearea'] * (data_coefficient_drag_induced(angle_of_attack) + data_coefficient_drag_parasite)

def calculateWeight(altitude):
    return 9.807 / np.power((6378.137 + altitude) / 6378.137, 2) * settings_conf['c172']['mass']

def calculateGeneratedThrust(altitude, airspeed, throttle_setting, state, simulation_failure_time):
    if state['fuellevels'][-1] < 1: 
        return 0
    if state['engineloading'][-1] <= 0 and state['flighttime'][1] > simulation_failure_time:
        return 0
    return 0.5 * data_density(altitude) * settings_conf['c172']['propellerarea'] * (np.power(state['propellerrpm'][-1] / 81.95, 2) - np.power(airspeed, 2)) * throttle_setting * (state['enginerpm'][-1] / settings_conf['c172']['defaults']['enginerpm'])
    
## Define control equations as a function of time:
def getPitch():
    return settings_conf['c172']['defaults']['angleofattack']

def getThrottle():
    return settings_conf['c172']['defaults']['throttle']

## Define state functions
def getFuelLevels(state, simulation_failure_type, simulation_failure_time):
    if state['fuellevels'][-1] < 1: 
        return 0
    if simulation_failure_type == 'fuellevels' and state['flighttime'][-1] > simulation_failure_time:
        return state['fuellevels'][-1] - 0.5 * (state['enginerpm'][-1] / settings_conf['c172']['defaults']['enginerpm']) * np.random.poisson(1, 1) if state['fuellevels'][-1] > 1 else 0
    return state['fuellevels'][-1] - getThrottle() * settings_conf['c172']['fuelefficiency'] * (state['enginerpm'][-1] / settings_conf['c172']['defaults']['enginerpm']) * np.random.poisson(1, 1) 

def getEngineRPM(state, simulation_failure_type, simulation_failure_time):
    if state['fuellevels'][-1] == 0: 
        return state['enginerpm'][-1] - 100 * np.random.normal(1, 0.005, 1) if state['enginerpm'][-1] - 100 > 0 else 0
    if state['intakemassflowrate'][-1] == 0 and state['flighttime'][-1] > simulation_failure_time:
        return state['enginerpm'][-1] - 200 * np.random.normal(1, 0.005, 1) if state['enginerpm'][-1] - 200 > 0 else 0
    if state['enginetemperature'][-1] < 80 and state['flighttime'][-1] > simulation_failure_time:
        return state['enginerpm'][-1] * (state['enginetemperature'][-1] + 20) / (state['enginerpm'][-1] / 24) if state['enginetemperature'][-1] > 0 else 0
    if state['enginetemperature'][-1] > 180 and state['flighttime'][-1] > simulation_failure_time:
        return state['enginerpm'][-1] + 20 if state['enginerpm'][-1] < 2600 else 0
    if simulation_failure_type == 'enginerpm' and state['flighttime'][-1] > simulation_failure_time:
        return 0
    return (state['enginerpm'][-1] + (settings_conf['c172']['defaults']['enginerpm'] - state['enginerpm'][-1]) * 0.1 * (state['intakemassflowrate'][-1] / settings_conf['c172']['defaults']['intakemassflowrate'])) * np.random.normal(1, 0.005, 1)

def getIntakeMassFlowRate(state, simulation_failure_type, simulation_failure_time):
    if simulation_failure_type == 'intakemassflowrate' and state['flighttime'][-1] > simulation_failure_time:
        return 0
    else:
        return data_density(state['altitude'][-1]) * state['airspeed'][-1] * settings_conf['c172']['intakearea'] * np.random.normal(1, 0.005, 1)

def getEngineTemperature(state, simulation_failure_type, simulation_failure_time):
    if simulation_failure_type == 'enginetemperature' and state['flighttime'][-1] > simulation_failure_time:
        return (state['enginetemperature'][-1] - (state['flighttime'][-1] - simulation_failure_time) * 0.002) if state['enginetemperature'][-1] > 0 else 0
    else:
        return state['enginerpm'][-1] / 24 * 200 / (state['oillevels'][-1] + 100) * np.random.normal(1, 0.005, 1)

def getOilLevels(state, simulation_failure_type, simulation_failure_time):
    if simulation_failure_type == 'oillevels' and state['flighttime'][-1] > simulation_failure_time:
        return state['oillevels'][-1] - 1 if state['oillevels'][-1] > 0 else 0
    else:
        return 100 * np.random.normal(1, 0.005, 1)

def getPropellerRPM(state, simulation_failure_type, simulation_failure_time):
    if simulation_failure_type == 'propellerrpm' and state['flighttime'][-1] > simulation_failure_time:
        return 0
    else:
        return state['enginerpm'][-1] * 2 * np.random.normal(1, 0.005, 1)

def getAlternatorOutput(state, simulation_failure_type, simulation_failure_time):
    if simulation_failure_type == 'alternatoroutput' and state['flighttime'][-1] > simulation_failure_time:
        return np.sin(state['flighttime'][-1] * 2) * 24 * np.exp(-0.01 * (state['flighttime'][-1] - simulation_failure_time))
    else:
        return np.sin(state['flighttime'][-1] * 2) * 24 * (state['enginerpm'][-1] / settings_conf['c172']['defaults']['enginerpm']) * np.random.normal(1, 0.01, 1)

def getBatteryLevels(state, simulation_failure_type, simulation_failure_time):
    if simulation_failure_type == 'batterylevels' and state['flighttime'][-1] > simulation_failure_time:
        return state['batterylevels'][-1] - 240 if state['batterylevels'][-1] > 0 else 0
    else:
        return state['batterylevels'][-1] - (24 / np.sqrt(2)) + (np.max(state['alternatoroutput'][-25:-1]) / np.sqrt(2)) * np.random.normal(1, 0.005, 1) if state['batterylevels'][-1] > 0 else 0

def getCommunicationsDelay(state, simulation_failure_type, simulation_failure_time):
    if simulation_failure_type == 'communicationsdelay' and state['flighttime'][-1] > simulation_failure_time:
        return -1
    if state['communicationsdelay'][-1] >= 2:
        return -1
    potentialDelay = state['communicationsdelay'][-1] + np.random.uniform(-1, 1) * 0.02
    return potentialDelay if potentialDelay > 0 else state['communicationsdelay'][-1] + np.random.uniform(0, 1) * 0.02

def getSatelliteMonitoring(state, simulation_failure_type, simulation_failure_time):
    if simulation_failure_type == 'satellitemonitoring' and state['flighttime'][-1] > simulation_failure_time:
        return 0
    else:
        return 1 if np.random.normal(0, 0.1) > -0.25 else 0

def getRadarProximity(state, simulation_failure_type, simulation_failure_time):
    return state['altitude'][-1] if np.random.normal(0, 0.1) >= 0 else state['radarproximity'][-1] - np.random.normal(0, 0.1)

def getOpticalVisibility(state, simulation_failure_type, simulation_failure_time):
    potentialVisibility = state['opticalvisibility'][-1] + np.random.uniform(-1, 1)
    return potentialVisibility if potentialVisibility < 100 else 100

def getFuselageStress(state, simulation_failure_type, simulation_failure_time):
    if (simulation_failure_type is 'fuellevels' or simulation_failure_type is 'oillevels') and state['flighttime'][-1] == simulation_failure_time - 4:
        return state['fuselagestress'][-1] + 80
    if simulation_failure_type == 'intakemassflowrate' and state['flighttime'][-1] > simulation_failure_time - 8:
        return (state['lift'][-1] + state['drag'][-1] + state['weight'][-1] + state['thrust'][-1]) / settings_conf['c172']['surfacearea'] + 40
    if simulation_failure_type == 'batterylevels' and state['flighttime'][-1] == simulation_failure_time - 2:
        return state['fuselagestress'][-1] + 160
    if simulation_failure_type == 'fuselagestress' and state['flighttime'][-1] > simulation_failure_time:
        return 0
    return (state['lift'][-1] + state['drag'][-1] + state['weight'][-1] + state['thrust'][-1]) / settings_conf['c172']['surfacearea'] * np.random.normal(1, 0.001)

# def getWingStress(state, simulation_failure_type, simulation_failure_time):

# def getTailStress(state, simulation_failure_type, simulation_failure_time):

# def getControlSurfaceLoading(state, simulation_failure_type, simulation_failure_time):

def getEngineLoading(state, simulation_failure_type, simulation_failure_time):
    if simulation_failure_type == 'engineloading' and state['flighttime'][-1] > simulation_failure_time:
        return 0
    return state['thrust'][-1] / settings_conf['c172']['surfacearea'] * np.random.normal(1, 0.001)

## Define the timestepping function
def advanceState(state, failure_type, failure_time):
    state['flighttime'].append(int(state['flighttime'][-1]) + 1)
    if state['batterylevels'][-1] > 0 and state['altitude'][-1] > 0 and state['communicationsdelay'][-1] > -1 and state['fuselagestress'][-1] > 0:
        state['lift'].append(calculateLift(state['altitude'][-1], state['airspeed'][-1], getPitch()) if state['altitude'][-1] > 0 else 0)
        state['drag'].append(calculateDrag(state['altitude'][-1], state['airspeed'][-1], getPitch()) if state['altitude'][-1] > 0 else 0)
        state['thrust'].append(calculateGeneratedThrust(state['altitude'][-1], state['airspeed'][-1], getThrottle(), state, failure_time) if state['altitude'][-1] > 0 else 0)
        state['weight'].append(calculateWeight(state['altitude'][-1]) if state['altitude'][-1] > 0 else 0)
        
        state['accely'].append((state['thrust'][-1] * np.sin(getPitch() * np.pi / 180) - state['drag'][-1] * np.sin(getPitch() * np.pi / 180) + state['lift'][-1] * np.cos(getPitch() * np.pi / 180) - state['weight'][-1]) / settings_conf['c172']['mass'] if state['altitude'][-1] > 0 else 0)
        state['accelx'].append((state['thrust'][-1] * np.cos(getPitch() * np.pi / 180) - state['drag'][-1] * np.cos(getPitch() * np.pi / 180) - state['lift'][-1] * np.sin(getPitch() * np.pi / 180)) / settings_conf['c172']['mass'] if state['altitude'][-1] > 0 else 0)
        
        state['vely'].append(state['accely'][-1] * simulation_multiplier * np.random.normal(1, 0.005, 1) + state['vely'][-1] if state['altitude'][-1] > 0 else 0)
        state['velx'].append(state['accelx'][-1] * simulation_multiplier * np.random.normal(1, 0.005, 1) + state['velx'][-1] if state['altitude'][-1] > 0 else 0)

        state['airspeed'].append(np.sqrt(np.power(state['velx'][-1], 2) + np.power(state['vely'][-1], 2)))
        state['altitude'].append(state['altitude'][-1] + state['vely'][-1] / 1000 if state['altitude'][-1] > 0 else 0)

        state['fuellevels'].append(getFuelLevels(state, failure_type, failure_time))
        state['enginerpm'].append(getEngineRPM(state, failure_type, failure_time))
        state['intakemassflowrate'].append(getIntakeMassFlowRate(state, failure_type, failure_time))
        state['enginetemperature'].append(getEngineTemperature(state, failure_type, failure_time))
        state['oillevels'].append(getOilLevels(state, failure_type, failure_time))
        state['propellerrpm'].append(getPropellerRPM(state, failure_type, failure_time))

        state['alternatoroutput'].append(getAlternatorOutput(state, failure_type, failure_time))
        state['batterylevels'].append(getBatteryLevels(state, failure_type, failure_time))

        state['communicationsdelay'].append(getCommunicationsDelay(state, failure_type, failure_time))
        state['satellitemonitoring'].append(getSatelliteMonitoring(state, failure_type, failure_time))
        state['radarproximity'].append(getRadarProximity(state, failure_type, failure_time))
        state['opticalvisibility'].append(getOpticalVisibility(state, failure_type, failure_time))

        state['fuselagestress'].append(getFuselageStress(state, failure_type, failure_time))
        state['engineloading'].append(getEngineLoading(state, failure_type, failure_time))
    else:
        for key in state:
            if key is not 'flighttime':
                state[key].append(0)
    return state

def generateLiveData(state, failure_type, failure_time):
    return advanceState(state, failure_type, failure_time)