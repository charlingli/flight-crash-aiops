from flask_restful import Resource
import os
import sys
import threading
import time
import scripts.utility as utility
import scripts.calculations as calculations

data = []

def generateLiveData():
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
        print(data)
        time.sleep(5.0 - ((time.time() - startTime) % 5.0))

class Flight(Resource):
  def get(self, id):
    print(data)
    return data, 200

t = threading.Thread(target=generateLiveData)
t.start()