import keyboard, paramiko, requests

class KeyboardControl:
    def __init__(self):
        self.verbose = True

        self.deepracer_name = "deepracer"
        self.deepracer_password = "deepracer1234"
        self.deepracer_ip = "192.168.1.70"
        self.port = ":1234"

        self.ssh = paramiko.SSHClient()
        self.last_command = ""

        self.command_start_server = '''
            source /opt/aws/deepracer/setup.sh
            python ~/deepracer-utils/put_best_cal.py
            cd ~/deepracer-utils/tools
            python ManualControlServer.py
            '''

        self.start_manual_control_server()
        print("Manual Control Server Started")

        keyboard.hook(self.dump)
        keyboard.wait("esc")
        keyboard.unhook_all()
        self.stop()
        self.kill_manual_control_server()
        exit()
    
    def start_manual_control_server(self):
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            client.connect(self.deepracer_ip,22,self.deepracer_name,self.deepracer_password)
            client.exec_command(self.command_start_server)
        except Exception as e:
            print("unable to start manual control server: "+str(e))


    def kill_manual_control_server(self):
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            client.connect(self.deepracer_ip,22,self.deepracer_name,self.deepracer_password)
            client.exec_command(self.command_start_server)
        except Exception as e:
            print("unable to quit manual control server: "+str(e))

    def forward(self):
        requests.get("http://"+self.deepracer_ip+self.port+"/forward")

    def left(self):
        requests.get("http://"+self.deepracer_ip+self.port+"/left")

    def backward(self):
        requests.get("http://"+self.deepracer_ip+self.port+"/backward")

    def right(self):
        requests.get("http://"+self.deepracer_ip+self.port+"/right")

    def reset_throttle(self):
        requests.get("http://"+self.deepracer_ip+self.port+"/reset_throttle")

    def reset_angle(self):
        requests.get("http://"+self.deepracer_ip+self.port+"/reset_angle")

    def stop(self):
        requests.get("http://"+self.deepracer_ip+self.port+"/stop")

    def dump(self, x):
        f = keyboard.KeyboardEvent('down',72,'up')
        d = keyboard.KeyboardEvent('down',80,'down')
        l = keyboard.KeyboardEvent('down',75,'left')
        r = keyboard.KeyboardEvent('down',77,'right')
        if x.event_type == 'down' and x.name==f.name:
            self.forward()
        elif x.event_type == 'down' and x.name==d.name:
            self.backward()
        elif x.event_type == 'down' and x.name==l.name:
            self.left()
        elif x.event_type == 'down' and x.name==r.name:
            self.right()
        elif x.event_type == 'up' and x.name==f.name:
            self.reset_throttle()
        elif x.event_type == 'up' and x.name==d.name:
            self.reset_throttle()
        elif x.event_type == 'up' and x.name==l.name:
            self.reset_angle()
        elif x.event_type == 'up' and x.name==r.name:
            self.reset_angle()

if __name__=="__main__":
    kc = KeyboardControl()