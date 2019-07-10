from flask_restful import Resource

import concurrent.futures
import json
import numpy as np
import os
import sys
import time
import yaml

import scripts.calculations as calculations

dir_path = os.path.dirname(os.path.realpath(__file__))
settings_file = open(os.path.join(dir_path, 'settings.yml'), 'r')
settings_conf = yaml.load(settings_file, Loader=yaml.CLoader)
settings_file.close()

simulation_length = settings_conf['simulation']['length']

def initialiseFlight(i):
  print('Thread ' + str(i) + ' started')
  calculations.generateLiveData(i)

class Flight(Resource):
  def get(self, id):
    file_path = os.path.join(dir_path, 'live', str(id) + '.txt')
    with open(file_path, 'r') as f:
      data = json.load(f)
    return data, 200

with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
  executor.map(initialiseFlight, range(3))