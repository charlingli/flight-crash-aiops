from flask_restful import Resource

import concurrent.futures
import json
import numpy as np
import os
import sys
import threading
import time
import yaml

import scripts.calculations as calculations

dir_path = os.path.dirname(os.path.realpath(__file__))
settings_file = open(os.path.join(dir_path, 'settings.yml'), 'r')
settings_conf = yaml.load(settings_file, Loader=yaml.CLoader)
settings_file.close()

simulation_length = settings_conf['simulation']['length']

def initialiseFlight(i):
    start_time = time.time()

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
        "enginerpm": [0],
        "intakemassflowrate": [0],
        "enginetemperature": [0],
        "oillevels": [100],
        "propellerrpm": [0],
        "alternatoroutput": [0],
        "batterylevels": [4000],
        "communicationsdelay": [0],
        "satellitemonitoring": [1],
        "radarproximity": [2],
        "opticalvisibility": [100],
        "fuselagestress": [1],
        "engineloading": [1]
    }
    
    simulation_failure_type = settings_conf['failure'][np.random.randint(len(settings_conf['failure']))]
    simulation_failure_time = np.random.randint(np.floor(simulation_length / 5), np.floor(simulation_length * 4 / 5))

    print('INFO\tcalculations:generateLiveData\tStarting new flight ' + str(i) + ' at time ' + str(start_time) + ' with failure mode: ' + simulation_failure_type + ' at time ' + str(simulation_failure_time))
    while True:
        state = calculations.generateLiveData(state, simulation_failure_type, simulation_failure_time)

        if state['flighttime'][-1] >= simulation_length:
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
                "enginerpm": [0],
                "intakemassflowrate": [0],
                "enginetemperature": [0],
                "oillevels": [100],
                "propellerrpm": [0],
                "alternatoroutput": [0],
                "batterylevels": [4000],
                "communicationsdelay": [0],
                "satellitemonitoring": [1],
                "radarproximity": [2],
                "opticalvisibility": [100],
                "fuselagestress": [1],
                "engineloading": [1]
            }
            start_time = time.time()
            simulation_failure_type = settings_conf['failure'][np.random.randint(len(settings_conf['failure']))]
            simulation_failure_time = np.random.randint(np.floor(simulation_length / 5), np.floor(simulation_length * 4 / 5))

        dump = {}
        for key in state:
            dump[key] = json.dumps(np.array(state[key][-1]).tolist())


        with open(str(i) + '.txt', 'w+') as f:
            json.dump(dump, f)
        
        if state['flighttime'][-1] % 10 == 0:
            print('Flight ' + str(i) + ' launched ' + str(state['flighttime'][-1]) + ' seconds ago has failuretype ' + simulation_failure_type + ' at ' + str(simulation_failure_time) + ' seconds.')
        time.sleep(settings_conf['simulation']['resolution'] - ((time.time() - start_time) % settings_conf['simulation']['resolution']))

class Flight(Resource):
    def get(self, id):
      file_path = str(id) + '.txt'
      with open(file_path, 'r') as f:
          data = json.load(f)
      return data, 200

t0 = threading.Thread(target=initialiseFlight,args=(0,))
t1 = threading.Thread(target=initialiseFlight,args=(1,))
t2 = threading.Thread(target=initialiseFlight,args=(2,))
t3 = threading.Thread(target=initialiseFlight,args=(3,))
t4 = threading.Thread(target=initialiseFlight,args=(4,))
t5 = threading.Thread(target=initialiseFlight,args=(5,))
t6 = threading.Thread(target=initialiseFlight,args=(6,))
t7 = threading.Thread(target=initialiseFlight,args=(7,))
t8 = threading.Thread(target=initialiseFlight,args=(8,))
t9 = threading.Thread(target=initialiseFlight,args=(9,))

t0.start()
t1.start()
t2.start()
t3.start()
t4.start()
t5.start()
t6.start()
t7.start()
t8.start()
t9.start()