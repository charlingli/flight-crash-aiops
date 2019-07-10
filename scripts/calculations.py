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

simulationLength = settings_conf['simulation']['length']
data_sigmoid_takeoff = lambda flightTime: 1 / (1 + np.exp(0.1 * flightTime - (simulationLength / 5 * 0.1)))
data_sigmoid_landing = lambda flightTime: 1 / (1 + np.exp(-0.1 * flightTime + (simulationLength * 4 / 5 * 0.1)))
data_sigmoid_cruise = lambda flightTime: -data_sigmoid_takeoff(flightTime) - data_sigmoid_landing(flightTime) + 1

def generateFuelData(flightTime, data_noise):
    fuel_function_takeoff = 100 - (simulationLength / 6000) * flightTime
    fuel_function_cruise = (100 - (simulationLength / 6000) * (simulationLength / 5)) - (simulationLength / 12000) * (flightTime - simulationLength / 5)
    fuel_function_landing = ((100 - (simulationLength / 6000) * (simulationLength / 5)) - (simulationLength / 12000) * (flightTime - simulationLength / 5)) + (simulationLength / 16000) * (flightTime - simulationLength * 4 / 5)
    fuel_function_combined = (fuel_function_takeoff * data_sigmoid_takeoff(flightTime) + fuel_function_cruise * data_sigmoid_cruise(flightTime) + fuel_function_landing * data_sigmoid_landing(flightTime)) * data_noise
    return fuel_function_combined
    
def generateEngineRPMData(flightTime, data_noise):
    enginerpm_function_takeoff = 2200
    enginerpm_function_cruise = 1800
    enginerpm_function_landing = 1200
    enginerpm_function_combined = (enginerpm_function_takeoff * data_sigmoid_takeoff(flightTime) + enginerpm_function_cruise * data_sigmoid_cruise(flightTime) + enginerpm_function_landing * data_sigmoid_landing(flightTime)) * data_noise
    return enginerpm_function_combined
    
def generateEngineTempData(flightTime, data_noise):
    enginetemp_function_takeoff = np.log(simulationLength / 20 * flightTime + 1) * (100 / 7)
    enginetemp_function_cruise = (np.log(simulationLength / 20 * (simulationLength / 5) + 1) * (100 / 7)) + simulationLength / 16000 * (flightTime - simulationLength / 5)
    enginetemp_function_landing = ((np.log(simulationLength / 20 * (simulationLength / 5) + 1) * (100 / 7)) + simulationLength / 16000 * (simulationLength * 4 / 5 - simulationLength / 5)) - simulationLength / 14000 * (flightTime - simulationLength * 4 / 5)
    enginetemp_function_combined = (enginetemp_function_takeoff * data_sigmoid_takeoff(flightTime) + enginetemp_function_cruise * data_sigmoid_cruise(flightTime) + enginetemp_function_landing * data_sigmoid_landing(flightTime)) * data_noise
    return enginetemp_function_combined
    
def generateIntakeData(flightTime, data_noise):
    intake_function_takeoff = - np.power(flightTime - simulationLength / 12, 2) / np.power(simulationLength / 8, 2) + 8
    intake_function_cruise = 6
    intake_function_landing = 7
    intake_function_combined = (intake_function_takeoff * data_sigmoid_takeoff(flightTime) + intake_function_cruise * data_sigmoid_cruise(flightTime) + intake_function_landing * data_sigmoid_landing(flightTime)) * data_noise
    return intake_function_combined
    
def generateBatteryData(flightTime, data_noise):
    battery_function_takeoff = np.abs(np.log(flightTime + 1)) * (100 / (np.log(simulationLength / 5 + 1)))
    battery_function_cruise = 100
    battery_function_landing = 100
    battery_function_combined = (battery_function_takeoff * data_sigmoid_takeoff(flightTime) + battery_function_cruise * data_sigmoid_cruise(flightTime) + battery_function_landing * data_sigmoid_landing(flightTime)) * data_noise
    return battery_function_combined
    
def generateAlternatorData(flightTime, data_noise):
    alternator_function_combined = np.sin(flightTime) * 24 * data_noise
    alternator_rms_function_combined = np.sqrt(24) * data_noise
    return [alternator_function_combined, alternator_rms_function_combined]

def generateRadioDelayData(flightTime, data_noise):
    radiodelay_function_takeoff = 10
    radiodelay_function_cruise = 10 + np.sin((flightTime - simulationLength / 5) / (simulationLength / (1.5 * np.pi)))
    radiodelay_function_landing = 10
    radiodelay_function_combined = (radiodelay_function_takeoff * data_sigmoid_takeoff(flightTime) + radiodelay_function_cruise * data_sigmoid_cruise(flightTime) + radiodelay_function_landing * data_sigmoid_landing(flightTime)) * data_noise
    return radiodelay_function_combined

def generateSatelliteOnlineData(flightTime, data_noise):
    satelliteonline_function_takeoff = 1
    satelliteonline_function_cruise = np.piecewise(data_noise, [data_noise <= 1.01, data_noise > 1.01], [1, 0])
    satelliteonline_function_landing = 1
    satelliteonline_function_combined = (satelliteonline_function_takeoff * data_sigmoid_takeoff(flightTime) + satelliteonline_function_cruise * data_sigmoid_cruise(flightTime) + satelliteonline_function_landing * data_sigmoid_landing(flightTime))
    return satelliteonline_function_combined

def generateRadarProximityData(flightTime, data_noise):
    radarproximity_function_takeoff = np.power(flightTime, 2) / (np.power(simulationLength / 8, 2))
    radarproximity_function_cruise = np.power(simulationLength / 5, 2) / (np.power(simulationLength / 8, 2)) + 0.25
    radarproximity_function_landing = np.power(flightTime - simulationLength, 2) / (np.power(simulationLength / 8, 2))
    radarproximity_function_combined = (radarproximity_function_takeoff * data_sigmoid_takeoff(flightTime) + radarproximity_function_cruise * data_sigmoid_cruise(flightTime) + radarproximity_function_landing * data_sigmoid_landing(flightTime)) * data_noise
    return radarproximity_function_combined

def generateLiveData(flightTime):
    data_noise = np.random.normal(1, 0.005, 1)
    fuelData = generateFuelData(flightTime, data_noise)
    enginerpmData = generateEngineRPMData(flightTime, data_noise)
    enginetempData = generateEngineTempData(flightTime, data_noise)
    intakeData = generateIntakeData(flightTime, data_noise)
    batteryData = generateBatteryData(flightTime, data_noise)
    alternatorData = generateAlternatorData(flightTime, data_noise)[0]
    alternatorrmsData = generateAlternatorData(flightTime, data_noise)[1]
    radiodelayData = generateRadioDelayData(flightTime, data_noise)
    satelliteonlineData = generateSatelliteOnlineData(flightTime, data_noise)
    radarproximityData = generateRadarProximityData(flightTime, data_noise)
    data_payload = {
        'Time' : str(flightTime), 
        'Data' : 
        {
          'Propulsion : Fuel' : str(fuelData),
          'Propulsion : Engine RPM' : str(enginerpmData)
        }
      }
    return data_payload
    
def generateStaticData(nSamples):
    dataTime = list(range(simulationLength))

    with open('staticData.txt', 'w+') as f:
        f.write('{"Flight Data" : {')
        for i in range(0, nSamples):
            fuelData = []
            enginerpmData = []
            enginetempData = []
            intakeData = []
            batteryData = []
            alternatorData = []
            alternatorrmsData = []
            radiodelayData = []
            satelliteonlineData = []
            radarproximityData = []
            f.write('"Flight Number" : ' + str(i) + ',')
            
            for j in dataTime:
                data_noise = np.random.normal(1, 0.005, 1)
                fuelData.append(generateFuelData(j, data_noise))
                enginerpmData.append(generateEngineRPMData(j, data_noise))
                enginetempData.append(generateEngineTempData(j, data_noise))
                intakeData.append(generateIntakeData(j, data_noise))
                batteryData.append(generateBatteryData(j, data_noise))
                alternatorData.append(generateAlternatorData(j, data_noise)[0])
                alternatorrmsData.append(generateAlternatorData(j, data_noise)[1])
                radiodelayData.append(generateRadioDelayData(j, data_noise))
                satelliteonlineData.append(generateSatelliteOnlineData(j, data_noise))
                radarproximityData.append(generateRadarProximityData(j, data_noise))
                f.write('"Time" : ' + str(j) + ', "Data" : {"Propulsion : Fuel" : ' + str(int(fuelData[j])) + ', "Propulsion : Engine RPM" : ' + str(int(enginerpmData[j])) + ', "Propulsion : Engine Temperature" : ' + str(int(enginetempData[j])) + ', "Propulsion : Intake Mass Flow Rate" : ' + str(int(intakeData[j])) + '}')
                f.write('"Time" : ' + str(j) + ', "Data" : {"CPU : Utilisation" : ' + str(int(fuelData[j])) + ', "Memory : Allocation" : ' + str(int(enginerpmData[j])) + ', "Disk : Free Space" : ' + str(int(enginetempData[j])) + ', "Disk : IOPS" : ' + str(int(intakeData[j])) + '}')
    f.close()
                
                
if __name__ == "__main__":
    # Debugging for when this is executed as a script

    dataTime = list(range(simulationLength))

    fuelData = []
    enginerpmData = []
    enginetempData = []
    intakeData = []
    batteryData = []
    alternatorData = []
    alternatorrmsData = []
    radiodelayData = []
    satelliteonlineData = []
    radarproximityData = []
    
    for i in dataTime:
        data_noise = np.random.normal(1, 0.005, 1)
        fuelData.append(generateFuelData(i, data_noise))
        enginerpmData.append(generateEngineRPMData(i, data_noise))
        enginetempData.append(generateEngineTempData(i, data_noise))
        intakeData.append(generateIntakeData(i, data_noise))
        batteryData.append(generateBatteryData(i, data_noise))
        alternatorData.append(generateAlternatorData(i, data_noise)[0])
        alternatorrmsData.append(generateAlternatorData(i, data_noise)[1])
        radiodelayData.append(generateRadioDelayData(i, data_noise))
        satelliteonlineData.append(generateSatelliteOnlineData(i, data_noise))
        radarproximityData.append(generateRadarProximityData(i, data_noise))

    plt.plot(dataTime, fuelData, label='Fuel Level')
    plt.ylim((0, max(fuelData)))
    plt.xlabel('Flight Time (s)')
    plt.ylabel('Fuel Levels (%)')
    plt.title('Fuel Level over Flight Duration')
    plt.savefig('demo/img_fueldata.png')
    plt.close()
    
    plt.plot(dataTime, enginerpmData, label='Engine RPM')
    plt.ylim((0, max(enginerpmData)))
    plt.xlabel('Flight Time (s)')
    plt.ylabel('Engine RPM (RPM)')
    plt.title('Engine Speed over Flight Duration')
    plt.savefig('demo/img_enginerpmdata.png')
    plt.close()
    
    plt.plot(dataTime, enginetempData, label='Engine Temp')
    plt.ylim((0, max(enginetempData)))
    plt.xlabel('Flight Time (s)')
    plt.ylabel('Engine Temp (°C)')
    plt.title('Engine Temperature over Flight Duration')
    plt.savefig('demo/img_enginetempdata.png')
    plt.close()
    
    plt.plot(dataTime, intakeData, label='Intake Mass Flow Rate')
    plt.ylim((0, max(intakeData)))
    plt.xlabel('Flight Time (s)')
    plt.ylabel('Intake Mass Flow Rate (L/s)')
    plt.title('Intake Mass Flow Rate over Flight Duration')
    plt.savefig('demo/img_intakedata.png')
    plt.close()
    
    plt.plot(dataTime, batteryData, label='Battery Level')
    plt.ylim((0, max(batteryData)))
    plt.xlabel('Flight Time (s)')
    plt.ylabel('Battery Level (%)')
    plt.title('Battery Level over Flight Duration')
    plt.savefig('demo/img_batterydata.png')
    plt.close()
    
    plt.plot(dataTime, alternatorData, label='Alternator Output')
    plt.xlabel('Flight Time (s)')
    plt.ylabel('Alternator Output (V)')
    plt.title('Alternator Output over Flight Duration')
    plt.savefig('demo/img_alternatordata.png')
    plt.close()
    
    plt.plot(dataTime, alternatorrmsData, label='Alternator RMS Output')
    plt.ylim((0, max(alternatorrmsData) + 1))
    plt.xlabel('Flight Time (s)')
    plt.ylabel('Alternator Output (V)')
    plt.title('Alternator Output over Flight Duration')
    plt.savefig('demo/img_alternatorrmsdata.png')
    plt.close()
    
    plt.plot(dataTime, radiodelayData, label='Radio Delay')
    plt.ylim((0, max(radiodelayData)))
    plt.xlabel('Flight Time (s)')
    plt.ylabel('Radio Delay (ms)')
    plt.title('Radio Delay over Flight Duration')
    plt.savefig('demo/img_radiodelaydata.png')
    plt.close()
    
    plt.plot(dataTime, satelliteonlineData, label='Satellite Online')
    plt.ylim((0, 1.1))
    plt.xlabel('Flight Time (s)')
    plt.ylabel('Satellite Online')
    plt.title('Satellite Online over Flight Duration')
    plt.savefig('demo/img_satelliteonlinedata.png')
    plt.close()
    
    plt.plot(dataTime, radarproximityData, label='Radar Proximity')
    plt.ylim((0, max(radarproximityData) + 1))
    plt.xlabel('Flight Time (s)')
    plt.ylabel('Radar Proximity (km)')
    plt.title('Radar Proximity over Flight Duration')
    plt.savefig('demo/img_radarproximitydata.png')
    plt.close()
    
    generateStaticData(2)