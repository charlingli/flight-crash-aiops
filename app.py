from flask import Flask
from flask_restful import Api
from resources.main import Main

app = Flask(__name__)
api = Api(app)

api.add_resource(Main, "/main/all/<int:id>")
api.add_resource(Main, "/main/latest/<int:id>")

if __name__ == "__main__":
  app.run()