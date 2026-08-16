import json
import RESTApiClient

objects_server_url = "http://192.168.1.194:12345/OptiTrackRestServer"
object_client = RESTApiClient.RESTApiClient(objects_server_url)

# ObjectManager in charge of adding and deleting virtual objects in arena
class ObjectManager:
    def __init__(self, config_file=None):
        self.objects = dict()
        if config_file != None:
            self.loadConfig(config_file)
    
    def getObjectsBounding(self, x, y): # given a point (x,y), return the objects where (x,y) are in bounds
        objects = []
        for name in self.objects:
            values = self.objects[name].split(',')
            xobj = float(values[1]); yobj = float(values[2])
            wobj = 0.1; lobj = 0.1
            xlower = xobj-wobj; xupper = xobj+wobj
            ylower = yobj-lobj; yupper = yobj+lobj
            if x>=xlower and x<=xupper and y>=ylower and y<=yupper:
                objects.append(name)
        return objects

    def addObject(self, name, values):
        self.objects[name] = values
        object_client.restPUTjson({name:values})
    
    def undo(self):
        if len(self.objects) != 0:
            last_name = list(self.objects)[-1]
            self.deleteObject(last_name)

    def updateObjectPosition(self, name, x, y):
        if name in self.objects:
            values = self.objects[name].split(',')
            values[1] = str(x); values[2] = str(y)
            values = ",".join(values)
            object_client.restPUTjson({name:values})
            self.objects[name] = values
    
    def loadConfig(self, config_file):
        config_file_content = open(config_file,"r").read()
        try:
            objects = json.loads(config_file_content)
            for name in objects.keys():
                self.addObject(name, objects[name])
        except Exception as e:
            print("failed loading json file: "+str(e))
    
    def getValidObjectName(self, object_type):
        num = 4
        name = object_type+str(num)
        while name in self.objects:
            num += 1
            name = object_type+str(num)
        return name

    def getObjectsString(self):
        return json.dumps(self.objects)

    def deleteByType(self, object_type):
        target_objects = []
        for name in self.objects:
            if object_type in name:
                target_objects.append(name)
        object_client.restDELjson(target_objects)
        [self.objects.pop(name) for name in target_objects]

    def deleteObject(self, name):
        self.objects.pop(name)
        object_client.restDELjson([name])

    def deleteAll(self):
        object_client.restDELjson(list(self.objects.keys()))
        self.objects = dict()
    
    def __del__(self):
        self.deleteAll()