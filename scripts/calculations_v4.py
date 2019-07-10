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

simulation_failure_type = settings_conf['failure'][np.random.randint(len(settings_conf['failure']))]
simulation_failure_time = np.random.randint(np.floor(simulation_length / 5), np.floor(simulation_length * 4 / 5))

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

def calculateGeneratedThrust(altitude, airspeed, throttle_setting):
    if state['fuellevels'][-1] < 1: 
        return 0
    else:
        return 0.5 * data_density(altitude) * settings_conf['c172']['propellerarea'] * (np.power(settings_conf['c172']['propellervelocity'], 2) - np.power(airspeed, 2)) * throttle_setting * (state['enginerpm'][-1] / settings_conf['c172']['defaults']['enginerpm'])
    
## Define control equations as a function of time:
def getPitch():
    return settings_conf['c172']['defaults']['angleofattack']

def getThrottle():
    return settings_conf['c172']['defaults']['throttle']

## Define state functions
def getFuelLevels():
    if state['fuellevels'][-1] < 1: 
        return 0
    if simulation_failure_type == 'fuellevels' and state['flighttime'][-1] > simulation_failure_time:
        return state['fuellevels'][-1] - 0.4 * (state['enginerpm'][-1] / settings_conf['c172']['defaults']['enginerpm']) * np.random.poisson(1, 1) if state['fuellevels'][-1] > 1 else 0
    else:
        return state['fuellevels'][-1] - getThrottle() * settings_conf['c172']['fuelefficiency'] * (state['enginerpm'][-1] / settings_conf['c172']['defaults']['enginerpm']) * np.random.poisson(1, 1) 

def getEngineRPM():
    if state['fuellevels'][-1] == 0: 
        return state['enginerpm'][-1] - 100 * np.random.normal(1, 0.005, 1) if state['enginerpm'][-1] - 100 > 0 else 0
    if state['intakemassflowrate'][-1] == 0:
        return state['enginerpm'][-1] - 200 * np.random.normal(1, 0.005, 1) if state['enginerpm'][-1] - 200 > 0 else 0
    if state['enginetemperature'][-1] < 80 and state['flighttime'][-1] > simulation_failure_time:
        return state['enginerpm'][-1] * (state['enginetemperature'][-1] + 20) / (state['enginerpm'][-1] / 24) if state['enginetemperature'][-1] > 0 else 0
    if simulation_failure_type == 'enginerpm' and state['flighttime'][-1] > simulation_failure_time:
        return 0
    else:
        return state['enginerpm'][-1] + (settings_conf['c172']['defaults']['enginerpm'] - state['enginerpm'][-1]) * 0.1 * (state['intakemassflowrate'][-1] / settings_conf['c172']['defaults']['intakemassflowrate']) * np.random.normal(1, 0.005, 1)

def getIntakeMassFlowRate():
    if simulation_failure_type == 'intakemassflowrate' and state['flighttime'][-1] > simulation_failure_time:
        return 0
    else:
        return data_density(state['altitude'][-1]) * state['airspeed'][-1] * settings_conf['c172']['intakearea'] * np.random.normal(1, 0.005, 1)

def getEngineTemperature():
    if simulation_failure_type == 'enginetemperature' and state['flighttime'][-1] > simulation_failure_time:
        return (state['enginetemperature'][-1] - (state['flighttime'][-1] - simulation_failure_time) * 0.002) if state['enginetemperature'][-1] > 0 else 0
    else:
        return state['enginerpm'][-1] / 24

## Define a state matrix
state = {
    "flighttime": [0],
    "failuretype": [simulation_failure_type],
    "failuretime": [simulation_failure_time],
    "lift": [0],
    "drag": [0],
    "weight": [settings_conf['c172']['mass'] * 9.807],
    "thrust": [0],
    "accelx": [0],
    "accely": [0],
    "velx": [40 * np.random.normal(1, 0.005, 1)],
    "vely": [0],
    "airspeed": [40 * np.random.normal(1, 0.005, 1)],
    "altitude": [2 * np.random.normal(1, 0.005, 1)],
    "fuellevels": [100],
    "enginerpm": [0],
    "intakemassflowrate": [0],
    "enginetemperature": [0]
}

## Define the timestepping function
def advanceState():
    state['flighttime'].append(int(state['flighttime'][-1]) + 1)

    state['lift'].append(calculateLift(state['altitude'][-1], state['airspeed'][-1], getPitch()) if state['altitude'][-1] > 0 else 0)
    state['drag'].append(calculateDrag(state['altitude'][-1], state['airspeed'][-1], getPitch()) if state['altitude'][-1] > 0 else 0)
    state['thrust'].append(calculateGeneratedThrust(state['altitude'][-1], state['airspeed'][-1], getThrottle(),) if state['altitude'][-1] > 0 else 0)
    state['weight'].append(calculateWeight(state['altitude'][-1]) if state['altitude'][-1] > 0 else 0)
    
    state['accely'].append((state['thrust'][-1] * np.sin(getPitch() * np.pi / 180) - state['drag'][-1] * np.sin(getPitch() * np.pi / 180) + state['lift'][-1] * np.cos(getPitch() * np.pi / 180) - state['weight'][-1]) / settings_conf['c172']['mass'] if state['altitude'][-1] > 0 else 0)
    state['accelx'].append((state['thrust'][-1] * np.cos(getPitch() * np.pi / 180) - state['drag'][-1] * np.cos(getPitch() * np.pi / 180) - state['lift'][-1] * np.sin(getPitch() * np.pi / 180)) / settings_conf['c172']['mass'] if state['altitude'][-1] > 0 else 0)
    
    state['vely'].append(state['accely'][-1] * simulation_multiplier * np.random.normal(1, 0.005, 1) + state['vely'][-1] if state['altitude'][-1] > 0 else 0)
    state['velx'].append(state['accelx'][-1] * simulation_multiplier * np.random.normal(1, 0.005, 1) + state['velx'][-1] if state['altitude'][-1] > 0 else 0)

    state['airspeed'].append(np.sqrt(np.power(state['velx'][-1], 2) + np.power(state['vely'][-1], 2)))
    state['altitude'].append(state['altitude'][-1] + state['vely'][-1] / 1000 if state['altitude'][-1] > 0 else 0)

    state['fuellevels'].append(getFuelLevels())
    state['enginerpm'].append(getEngineRPM())
    state['intakemassflowrate'].append(getIntakeMassFlowRate())
    state['enginetemperature'].append(getEngineTemperature())

def resetState():
    global state
    state = {
        "flighttime": [0],
        "failuretype": [simulation_failure_type],
        "failuretime": [simulation_failure_time],
        "lift": [0],
        "drag": [0],
        "weight": [settings_conf['c172']['mass'] * 9.807],
        "thrust": [0],
        "accelx": [0],
        "accely": [0],
        "velx": [40 * np.random.normal(1, 0.005, 1)],
        "vely": [0],
        "airspeed": [40 * np.random.normal(1, 0.005, 1)],
        "altitude": [2 * np.random.normal(1, 0.005, 1)],
        "fuellevels": [100],
        "enginerpm": [0],
        "intakemassflowrate": [0],
        "enginetemperature": [0]
    }

def generateLiveData(i):
    while True:
        startTime = time.time()

        live_file = os.path.join(dir_path, 'live', str(i) + '.txt')
        static_file = os.path.join(dir_path, 'static', str(i) + '_' + str(startTime) + '.txt')

        json_file = open(static_file, 'w+')
        json_file.writelines('[')
        json_file.close()

        while state['flighttime'][-1] < simulation_length:
            dump = {};
            for key in state:
                dump[key] = json.dumps(np.array(state[key][-1]).tolist())

            advanceState()

            json_file = open(live_file, 'w+')
            json.dump(dump, json_file, indent=2)
            json_file.close()

            json_file = open(static_file, 'a+')
            json.dump(dump, json_file, indent=2)
            if (state['flighttime'][-1] < simulation_length):
                json_file.writelines(',')
            json_file.close()

            time.sleep(1.0 - ((time.time() - startTime) % 1.0))

        json_file = open(static_file, 'a+')
        json_file.writelines(']')
        json_file.close()

        resetState()
