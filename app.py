from flask import Flask
from flask_restful import Api
import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), 'resources'))
from main import Main

app = Flask(__name__)
api = Api(app)

api.add_resource(Main, "/main/all/<int:id>")
api.add_resource(Main, "/main/latest/<int:id>")

if __name__ == "__main__":
  app.run()