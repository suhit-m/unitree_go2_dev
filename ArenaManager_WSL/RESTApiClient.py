# Unchanged from legacy code

import httplib2
import json

class RESTApiClient():
    def __init__(self, url):
        self.url = url
    
    def restGETjson(self):
        try:
            return json.loads(httplib2.Http().request(self.url, method="GET")[1])
        except Exception as e:
            print("unable to perform get request: "+str(e))

    def restPOSTjson(self, json_data):
        try:
            return httplib2.Http().request(
                self.url,
                method='POST',
                headers={'Content-Type': 'application/json; charset=UTF-8'},
                body=json.dumps(json_data)
            )
        except Exception as e:
            print("unable to perform post request: "+str(e))

    def restPUTjson(self, json_data):
        try:
            return httplib2.Http().request(
                self.url,
                method='PUT',
                headers={'Content-Type': 'application/json; charset=UTF-8'},
                body=json.dumps(json_data)
            )
        except Exception as e:
            print("unable to perform put request: "+str(e))

    def restDELjson(self, json_data):
        try:
            return httplib2.Http().request(
                self.url,
                method='DELETE',
                headers={'Content-Type': 'application/json; charset=UTF-8'},
                body=json.dumps(json_data)
            )
        except Exception as e:
            print("unable to perform del request: "+str(e))
