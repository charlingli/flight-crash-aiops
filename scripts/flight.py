from flask_restful import Resource

import json
import os
import sys
import threading
import time
import yaml

import scripts.utility as utility
import scripts.calculations_v3 as calculations

def initialiseFlight():
    startTime = time.time()
    while True:
      data = [ {
          "Flight Data" : {
            "Flight Number" : 0,
            "Flight Metrics" : calculations.generateLiveData(utility.getFlightTime(startTime))
          }
      } ]
      with open('liveData.txt', 'w+') as f:
        json.dump(data, f)
      print('Data generated')
      time.sleep(5.0 - ((time.time() - startTime) % 5.0))

class Flight(Resource):
  def get(self, id):
    with open('liveData.txt', 'r') as f:
      data = json.load(f)
    return data, 200

t = threading.Thread(target=initialiseFlight)
t.start()