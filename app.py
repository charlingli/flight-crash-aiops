from flask import Flask
from flask_restful import Api

from scripts.flight import Flight

app = Flask(__name__)
api = Api(app)

api.add_resource(Flight, "/flight/all/<int:id>")
api.add_resource(Flight, "/flight/latest/<int:id>")

if __name__ == "__main__":
  app.run()