import datetime
import matplotlib.pyplot as plt
import numpy as np
import os
import sys
import time
import yaml

dir_path = os.path.dirname(os.path.realpath(__file__))
settings_file = open(dir_path + '/settings.yml', 'r')
settings_conf = yaml.load(settings_file, Loader=yaml.CLoader)
settings_file.close()

simulation_length = settings_conf['simulation']['length']
simulation_multiplier = settings_conf['simulation']['multiplier']

simulation_failure_type = settings_conf['failure'][np.random.randint(len(settings_conf['failure']))]
simulation_failure_time = np.random.randint(np.floor(simulation_length / 10), np.floor(simulation_length * 9 / 10))

## Different sigmoid mixing functions to indicate different stages of flight
data_takeoff_time = np.floor(simulation_length / 10)
data_landing_time = np.floor(simulation_length * 8 / 10)
data_sigmoid_takeoff = lambda flight_time: 1 / (1 + np.exp(0.1 * flight_time - (data_takeoff_time * 0.1)))
data_sigmoid_landing = lambda flight_time: 1 / (1 + np.exp(-0.1 * flight_time + (data_landing_time * 0.1)))
data_sigmoid_cruise = lambda flight_time: -data_sigmoid_takeoff(flight_time) - data_sigmoid_landing(flight_time) + 1

data_flight_time = np.linspace(0, simulation_length, simulation_length + 1)
# plt.plot(data_flight_time, data_sigmoid_takeoff(data_flight_time), label=r'Takeoff sigmoid')
# plt.plot(data_flight_time, data_sigmoid_cruise(data_flight_time), label=r'Cruise sigmoid')
# plt.plot(data_flight_time, data_sigmoid_landing(data_flight_time), label=r'Landing sigmoid')
# plt.xlabel(r'Flight Time (s)')
# plt.ylabel(r'Sigmoid Functions')
# plt.title(r'Sigmoid Functions')
# plt.legend()
# plt.show()

## Utility functions used in metric calculations
# test_data_aoa = np.linspace(-5, 20, 250)
data_coefficient_lift = lambda angle_of_attack: (0.5 + angle_of_attack / 10) + (3.2 - 2 * angle_of_attack / 9.6) / (1 + np.exp(7.5 - angle_of_attack / 2))

# plt.plot(test_data_aoa, data_coefficient_lift(test_data_aoa), label='Angle Of Attack')
# plt.xlabel(r'Angle Of Attack (°)')
# plt.ylabel(r'Coefficient of Lift')
# plt.title(r'C_L')
# plt.show()

test_data_altitude = np.linspace(0, 12, 120)
data_temperature = lambda altitude: 288.15 - 12.25 * altitude
data_density = lambda altitude: 1.225 * np.power(data_temperature(altitude) / (data_temperature(altitude) + 0.02 * altitude), 1 + 9.807 * 12.25 / (8.314 * 0.02))

# plt.plot(test_data_altitude, data_temperature(test_data_altitude), label='Temperature with Altitude')
# plt.xlabel(r'Altitude (m)')
# plt.ylabel(r'Temperature (°C)')
# plt.title(r'Temperature with Altitude')
# plt.show()

# plt.plot(test_data_altitude, data_density(test_data_altitude), label='Density with Altitude')
# plt.xlabel(r'Altitude (m)')
# plt.ylabel(r'Density (kg/m^3)')
# plt.title(r'Density with Altitude')
# plt.show()

data_coefficient_drag_induced = lambda angle_of_attack: np.power(data_coefficient_lift(angle_of_attack), 2) / (np.pi * settings_conf['c172']['aspectratio'])
data_coefficient_drag_parasite = settings_conf['c172']['parasitedrag']

# plt.plot(test_data_aoa, data_coefficient_drag_induced(test_data_aoa) + data_coefficient_drag_parasite, label=r'C_D with Angle of Attack')
# plt.xlabel(r'Angle of Attack (m)')
# plt.ylabel(r'C_D')
# plt.title(r'C_D with Angle of Attack')
# plt.show()

## Define FBD model equations
def calculateLift(altitude, airspeed, angle_of_attack):
    return 0.5 * data_density(altitude) * np.power(airspeed, 2) * settings_conf['c172']['surfacearea'] * data_coefficient_lift(angle_of_attack)

def calculateDrag(altitude, airspeed, angle_of_attack):
    return 0.5 * data_density(altitude) * np.power(airspeed, 2) * settings_conf['c172']['surfacearea'] * (data_coefficient_drag_induced(angle_of_attack) + data_coefficient_drag_parasite)

def calculateWeight(altitude):
    return 9.807 / np.power((6378.137 + altitude) / 6378.137, 2) * settings_conf['c172']['mass']

def calculateRequiredThrust(altitude, airspeed, angle_of_attack):
    return calculateDrag(altitude, airspeed, angle_of_attack) + calculateWeight(altitude) * np.sin(angle_of_attack * np.pi / 180)

def calculateGeneratedThrust(altitude, airspeed, throttle_setting, flight_time):
    if getFuelLevels(flight_time, state['fuellevels'][flight_time]) < 1: 
        return 0
    else:
        return 0.5 * data_density(altitude) * settings_conf['c172']['propellerarea'] * (np.power(settings_conf['c172']['propellervelocity'], 2) - np.power(airspeed, 2)) * throttle_setting
    
def calculateExcessThrust(altitude, airspeed, angle_of_attack, throttle_setting, flight_time):
    return calculateGeneratedThrust(altitude, airspeed, throttle_setting, flight_time) - calculateRequiredThrust(altitude, airspeed, angle_of_attack)

## Define kinematics equations
def calculateAccelerationX(altitude, airspeed, angle_of_attack, throttle_setting, flight_time):
    return calculateExcessThrust(altitude, airspeed, angle_of_attack, throttle_setting, flight_time) / settings_conf['c172']['mass'] * np.cos(angle_of_attack * np.pi / 180)

def calculateAccelerationY(altitude, airspeed, angle_of_attack, throttle_setting, flight_time):
    return calculateExcessThrust(altitude, airspeed, angle_of_attack, throttle_setting, flight_time) / settings_conf['c172']['mass'] * np.sin(angle_of_attack * np.pi / 180)

## Define control equations as a function of time:
def getPitch(flight_time):
    pitch = settings_conf['c172']['takeoff']['angleofattack'] * data_sigmoid_takeoff(flight_time) + settings_conf['c172']['cruise']['angleofattack'] * data_sigmoid_cruise(flight_time) + settings_conf['c172']['landing']['angleofattack'] * data_sigmoid_landing(flight_time);
    return pitch

# data_pitch = []
# for i in data_flight_time:
#     data_pitch.append(getPitch(i))

# plt.plot(data_flight_time, data_pitch, label=r'Pitch')
# plt.xlabel(r'Flight Time (s)')
# plt.ylabel(r'Pitch (°)')
# plt.title(r'Pitch')
# plt.show()

def getThrottle(flight_time):
    throttle = settings_conf['c172']['takeoff']['throttle'] * data_sigmoid_takeoff(flight_time) + settings_conf['c172']['cruise']['throttle'] * data_sigmoid_cruise(flight_time) + settings_conf['c172']['landing']['throttle'] * data_sigmoid_landing(flight_time);
    return throttle

## Define a state matrix
state = {
        "airspeed": [0],
        "altitude": [0],
        "fuellevels": [100],
        "intakemassflowrate": [0]
    };

## Define the time-dependent equations of state
def getAirspeed(flight_time, previous_value):
    if (flight_time <= data_takeoff_time):
        airspeed = calculateExcessThrust(0, previous_value, getPitch(flight_time), getThrottle(flight_time), flight_time) / settings_conf['c172']['mass'] * simulation_multiplier + previous_value;
    elif (flight_time > data_takeoff_time) and (flight_time < data_landing_time):
        airspeed = calculateExcessThrust(0, previous_value, getPitch(flight_time), getThrottle(flight_time), flight_time) / settings_conf['c172']['mass'] * simulation_multiplier + previous_value;
    else:
        airspeed = calculateExcessThrust(0, previous_value, getPitch(flight_time), getThrottle(flight_time), flight_time) / settings_conf['c172']['mass'] * simulation_multiplier + previous_value;
    state['airspeed'].append(airspeed * np.random.normal(1, 0.001, 1))
    return state['airspeed'][-1];

# airspeed_range = np.linspace(0, 30, 30)
# data_generated_thrust = []
# data_required_thrust = []
# data_excess_thrust = []
# for i in airspeed_range:
#     data_generated_thrust.append(calculateGeneratedThrust(0, i, settings_conf['c172']['takeoff']['throttle']))
#     data_required_thrust.append(calculateRequiredThrust(0, i, settings_conf['c172']['takeoff']['angleofattack']))
#     data_excess_thrust.append(data_generated_thrust[-1] - data_required_thrust[-1])
# plt.plot(airspeed_range, data_generated_thrust, label=r'Generated Thrust during Takeoff')
# plt.plot(airspeed_range, data_required_thrust, label=r'Required Thrust during Takeoff')
# plt.plot(airspeed_range, data_excess_thrust, label=r'Excess Thrust during Takeoff')
# plt.ylim(0, max(data_generated_thrust) * 1.2)
# plt.xlabel(r'Airspeed (m/s)')
# plt.ylabel(r'Thrust (N)')
# plt.title(r'Thrust during Takeoff')
# plt.legend()
# plt.show()

# plt.plot(data_flight_time, data_airspeed, label=r'Airspeed')
# plt.xlabel(r'Flight Time (s)')
# plt.ylabel(r'Airspeed (m/s)')
# plt.title(r'Airspeed')
# plt.show()

def getAltitude(flight_time, previous_value):
    altitude = calculateAccelerationY(previous_value, getAirspeed(flight_time, previous_value), getPitch(flight_time), getThrottle(flight_time)) + previous_value
    state['altitude'].append(altitude * np.random.normal(1, 0.002, 1))
    return state['altitude'][-1];

# data_altitude = [0]
# for i in data_flight_time[1:]:
#     data_altitude.append(getAltitude(i, data_altitude[-1]))

# plt.plot(data_flight_time, data_altitude, label=r'Altitude')
# plt.xlabel(r'Flight Time (s)')
# plt.ylabel(r'Altitude (m)')
# plt.title(r'Altitude')
# plt.show()

## Define metrics equations
def getFuelLevels(flight_time, previous_value):
    if simulation_failure_type == 'fuellevels' and flight_time > simulation_failure_time:
        fuel_function = previous_value - 0.4 if previous_value > 1 else 0
    else:
        fuel_function = previous_value - getThrottle(flight_time) * settings_conf['c172']['fuelefficiency'] * np.random.poisson(1, 1)
    state['fuellevels'].append(fuel_function)
    return state['fuellevels'][-1]

fuel_levels = [100]
for i in data_flight_time[1:]:
    fuel_levels.append(getFuelLevels(i, fuel_levels[-1]))

# plt.plot(data_flight_time, fuel_levels, label=r'Fuel Levels')
# plt.xlabel(r'Flight Time (s)')
# plt.ylabel(r'Fuel Levels (%)')
# plt.title(r'Fuel Level')
# plt.show()

def getIntakeMassFlowRate(flight_time, previous_value):
    if simulation_failure_type == 'intakemassflowrate' and flight_time > simulation_failure_time:
        intake_function = previous_value - 0.4 if previous_value > 1 else 0
    else:
        intake_function = previous_value - getThrottle(flight_time) * settings_conf['c172']['fuelefficiency'] * np.random.poisson(1, 1)
    state['intakemassflowrate'].append(intake_function)
    return state['intakemassflowrate'][-1]

fuel_levels = [100]
for i in data_flight_time[1:]:
    fuel_levels.append(getFuelLevels(i, fuel_levels[-1]))

plt.plot(data_flight_time, fuel_levels, label=r'Fuel Levels')
plt.xlabel(r'Flight Time (s)')
plt.ylabel(r'Fuel Levels (%)')
plt.title(r'Fuel Level')
plt.show()

print(simulation_failure_type + ' failure at ' + str(simulation_failure_time))

# print(calculateLift(2, 27.778, 5.768))
# print(calculateWeight(2))
# print(calculateGeneratedThrust(2, 27.778, 1))
# print(calculateRequiredThrust(2, 27.778, 5.768))
# print(calculateExcessThrust(2, 22.224, 3.844, 0.169))
# print(calculateAccelerationX(2, 33.336, 10.77, 0.792))
# print(calculateAccelerationY(2, 33.336, 10.77, 0.792))
# print(calculateVelocityX(2, 27.778, 5.768, 0.362))
# print(calculateVelocityY(2, 27.778, 5.768, 0.362))

# print(simulation_failure_type + ' failure at ' + str(simulation_failure_time))

# def getLiveState():