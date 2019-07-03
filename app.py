from flask import Flask
from flask_restful import Api

import resources.main as Main

app = Flask(__name__)
api = Api(app)

api.add_resource(Main.Main, "/main/all/<int:id>")
api.add_resource(Main.Main, "/main/latest/<int:id>")

if __name__ == "__main__":
  app.run()