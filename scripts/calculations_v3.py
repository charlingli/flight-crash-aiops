import datetime
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
# simulation_file = dir_path + /data

simulation_length = settings_conf['simulation']['length']
simulation_multiplier = settings_conf['simulation']['multiplier']

simulation_failure_type = settings_conf['failure'][np.random.randint(len(settings_conf['failure']))]
simulation_failure_time = np.random.randint(np.floor(simulation_length / 5), np.floor(simulation_length * 4 / 5))

print(simulation_failure_type + ' failure at ' + str(simulation_failure_time))

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

## Define a state matrix
state = {
        "flighttime": [0],
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
        "enginerpm": [0]
    }

## Define state functions
def getFuelLevels():
    if state['fuellevels'][-1] < 1: 
        return 0
    if simulation_failure_type == 'fuellevels' and state['flighttime'][-1] > simulation_failure_time:
        return state['fuellevels'][-1] - 0.4 * (state['enginerpm'][-1] / settings_conf['c172']['defaults']['enginerpm']) * np.random.poisson(1, 1) if state['fuellevels'][-1] > 1 else 0
    else:
        return state['fuellevels'][-1] - getThrottle() * settings_conf['c172']['fuelefficiency'] * (state['enginerpm'][-1] / settings_conf['c172']['defaults']['enginerpm']) * np.random.poisson(1, 1) 

## Define state functions
def getEngineRPM():
    if state['fuellevels'][-1] < 1: 
        return state['enginerpm'][-1] - 100 * np.random.normal(1, 0.005, 1) if state['enginerpm'][-1] - 100 > 0 else 0
    if simulation_failure_type == 'enginerpm' and state['flighttime'][-1] > simulation_failure_time:
        return 0
    else:
        return state['enginerpm'][-1] + (settings_conf['c172']['defaults']['enginerpm'] - state['enginerpm'][-1]) * 0.1 * np.random.normal(1, 0.005, 1)

## Define the incremental time-dependent equations of state
def advanceState():
    state['flighttime'].append(state['flighttime'][-1] + 1)

    state['lift'].append(calculateLift(state['altitude'][-1], state['airspeed'][-1], getPitch()) if state['altitude'][-1] > 0 else 0)
    state['drag'].append(calculateDrag(state['altitude'][-1], state['airspeed'][-1], getPitch()) if state['altitude'][-1] > 0 else 0)
    state['thrust'].append(calculateGeneratedThrust(state['altitude'][-1], state['airspeed'][-1], getThrottle()) if state['altitude'][-1] > 0 else 0)
    state['weight'].append(calculateWeight(state['altitude'][-1]) if state['altitude'][-1] > 0 else 0)
    
    state['accely'].append((state['thrust'][-1] * np.sin(getPitch() * np.pi / 180) - state['drag'][-1] * np.sin(getPitch() * np.pi / 180) + state['lift'][-1] * np.cos(getPitch() * np.pi / 180) - state['weight'][-1]) / settings_conf['c172']['mass'] if state['altitude'][-1] > 0 else 0)
    state['accelx'].append((state['thrust'][-1] * np.cos(getPitch() * np.pi / 180) - state['drag'][-1] * np.cos(getPitch() * np.pi / 180) - state['lift'][-1] * np.sin(getPitch() * np.pi / 180)) / settings_conf['c172']['mass'] if state['altitude'][-1] > 0 else 0)
    
    state['vely'].append(state['accely'][-1] * simulation_multiplier * np.random.normal(1, 0.005, 1) + state['vely'][-1] if state['altitude'][-1] > 0 else 0)
    state['velx'].append(state['accelx'][-1] * simulation_multiplier * np.random.normal(1, 0.005, 1) + state['velx'][-1] if state['altitude'][-1] > 0 else 0)

    state['airspeed'].append(np.sqrt(np.power(state['velx'][-1], 2) + np.power(state['vely'][-1], 2)))
    state['altitude'].append(state['altitude'][-1] + state['vely'][-1] / 1000 if state['altitude'][-1] > 0 else 0)

    state['fuellevels'].append(getFuelLevels())
    state['enginerpm'].append(getEngineRPM())

    print('Force balance, x: ', state['thrust'][-1] * np.cos(getPitch() * np.pi / 180) - state['drag'][-1] * np.cos(getPitch() * np.pi / 180) - state['lift'][-1] * np.sin(getPitch() * np.pi / 180))
    print('Force balance, y: ', state['thrust'][-1] * np.sin(getPitch() * np.pi / 180) - state['drag'][-1] * np.sin(getPitch() * np.pi / 180) + state['lift'][-1] * np.cos(getPitch() * np.pi / 180) - state['weight'][-1])
    print('Acceleration x, y, combined: ', state['accelx'][-1], state['accely'][-1], np.sqrt(np.power(state['accelx'][-1], 2) + np.power(state['accely'][-1], 2)))
    print('Velocity x, y, combined: ', state['velx'][-1], state['vely'][-1], np.sqrt(np.power(state['velx'][-1], 2) + np.power(state['vely'][-1], 2)))
    print('Altitude: ', state['altitude'][-1])
    print('Fuel, EngineRPM: ', state['fuellevels'][-1], state['enginerpm'][-1])

    print('\n')
    # print(state['drag'][-1] * np.sin(getPitch() * np.pi / 180))
    # print(state['lift'][-1] * np.cos(getPitch() * np.pi / 180))
    # print(state['lift'][-1], state['drag'][-1], state['thrust'][-1], state['weight'][-1])
    # state['accelx'].append(state['accelx'][-1] + calculateAccelerationX(state['altitude'][-1], state['airspeed'][-1], getPitch(), getThrottle()))
    # state['accely'].append(state['accely'][-1] + calculateAccelerationY(state['altitude'][-1], state['airspeed'][-1], getPitch(), getThrottle()))
    # potential_airspeed = np.sqrt(np.power(state['accelx'][-1], 2) + np.power(state['accely'][-1], 2)) + state['airspeed'][-1]
    # state['airspeed'].append(potential_airspeed)


# test_drag = []
# test_thrust = []
# test_excess = []
# for i in np.linspace(0, 80, 81):
#     test_drag.append(calculateDrag(0, i, 0))
#     test_thrust.append(calculateGeneratedThrust(0, i, 1))
#     test_excess.append(calculateExcessThrust(0, i, 0, 1))

# plt.plot(np.linspace(0, 80, 81), test_drag, label='Drag')
# plt.plot(np.linspace(0, 80, 81), test_thrust, label='Thrust')
# plt.plot(np.linspace(0, 80, 81), test_excess, label='Excess')
# plt.xlabel(r'Time (s)')
# plt.ylabel(r'Airspeed')
# plt.legend()
# plt.title(r'')
# plt.show()

for i in data_flight_time:
    # print(state['airspeed'][-1])
    # print(state['altitude'][-1])
    # print(state['accelx'][-1])
    # print(state['accely'][-1])
    # print(calculateLift(state['altitude'][-1], state['airspeed'][-1], getPitch(i)))
    # print(calculateWeight(state['altitude'][-1]))
    # print(calculateDrag(state['altitude'][-1], state['airspeed'][-1], getPitch(i)))
    # print(calculateExcessThrust(state['altitude'][-1], state['airspeed'][-1], getPitch(i), getThrottle(i), i))
    # print(calculateDrag(state['altitude'][-1], state['airspeed'][-1], getPitch(i)))
    advanceState()
    # input("Press Enter to continue...")

# print(state)
plt.subplot(4, 1, 1)
plt.plot(data_flight_time, state['airspeed'][:-1], label='Airspeed')
plt.ylabel(r'Airspeed (m/s)')
plt.xlabel(r'Time (s)')
plt.subplot(4, 1, 2)
plt.plot(data_flight_time, state['altitude'][:-1], label='Altitude')
plt.ylabel(r'Altitude (km)')
plt.xlabel(r'Time (s)')
plt.subplot(4, 1, 3)
plt.plot(data_flight_time, state['fuellevels'][:-1], label='Fuel Levels')
plt.ylabel(r'Fuel Levels (%)')
plt.xlabel(r'Time (s)')
plt.subplot(4, 1, 4)
plt.plot(data_flight_time, state['enginerpm'][:-1], label='Engine RPM')
plt.ylabel(r'Engine RPM (rpm)')
plt.xlabel(r'Time (s)')
plt.title(r'')
plt.show()

print(simulation_failure_type + ' failure at ' + str(simulation_failure_time))