from flask import Flask
from flask_restful import Api

import resources.flight as Flight

app = Flask(__name__)
api = Api(app)

api.add_resource(Flight.Flight, "/flight/all/<int:id>")
api.add_resource(Flight.Flight, "/flight/latest/<int:id>")

if __name__ == "__main__":
  app.run()