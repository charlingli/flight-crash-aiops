from flask_restful import Resource
import os
import sys
import threading
import time
import scripts.utility as utility
import scripts.calculations as calculations

data = [{'Flight Data': {'Flight Number': 0, 'Flight Metrics': {'"Time" : 100,\n"Data" : {\n\t"Propulsion : Fuel" : [89.41096743],\n\t"Propulsion : Engine RPM" : [2141.06811649]\n}\n'}}}]

def getData():
    startTime = time.time()
    while True:
        data = [ {
            "Flight Data" : {
              "Flight Number" : 0,
              "Flight Metrics" : {
                calculations.generateLiveData(utility.getFlightTime(startTime))
              }
            }
        } ]
        print('Data generated')
        time.sleep(5.0 - ((time.time() - startTime) % 5.0))

class Flight(Resource):
  def get(self, id):
    print(data)
    return data, 200

t = threading.Thread(target=getData)
t.start()