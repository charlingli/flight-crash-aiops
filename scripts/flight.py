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

if __name__ == "__main__":
    t = threading.Thread(target=generateLiveData)
    t.start()

class Flight(Resource):
  def get(self, id):
    for metric in data:
        return metric, 200