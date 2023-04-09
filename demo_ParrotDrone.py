import socket
import time
import threading
import demo_artificial_potential_field
import olympe
import os
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged

class ParrotDrone:
    def __init__(self):    
    # def start_sensors():
      print("start sensors")
      self.sensor_readings = []
      self.avg_sensor_readings = [2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0]
      self.sensor_readings_last_received_timestamp = None
      
      sensors_thread = threading.Thread(target=self.TOFSensors)
      main_thread = threading.Thread(target=self.main_drone)
      sensors_thread.daemon = True
      sensors_thread.start()
      main_thread.start()
      

    def TOFSensors(self):
        UDP_IP = ""   # listen on all available interfaces
        UDP_PORT = 12345      # choose an available port number
        print("!DEBUG starting tof sensor thread")
        
        # 2390 is the sensor's local port
        # 192.168.0.152 is the sensor's address
        
        # create a UDP socket object
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print("!DEBUG creating tof sensor socket complete")
        
        # bind the socket to a specific IP address and port number
        sock.bind((UDP_IP, UDP_PORT))
      
        try:
          while True:
              # time.sleep(1)  
              # receive data and the address of the sender
              # print("!DEBUG attempting to recieve message")
              # sock.setblocking(False)
              data, addr = sock.recvfrom(1024)  # buffer size is 1024 bytes
          
              # print received data and sender's address
              print(f"!DEBUG Received message: {data.decode()} from {addr}")
              
              raw_sensor_readings = data.decode('utf-8')
              self.sensor_readings = list(map(int, raw_sensor_readings.split(";")))
              self.sensor_readings_last_received_timestamp = time.time()
        
        finally:
          print("!DEBUG closing the socket")
          sock.close()
        
    def main_drone(self):
    
      start_time = time.time()
      RUNTIME = 30
      DRONE_IP = os.environ.get("DRONE_IP", "192.168.42.1")
    
      apf = demo_artificial_potential_field.APF()
      self.drone = olympe.Drone(DRONE_IP) 
      self.drone.connect()
      print("RANDOMDEBUGGG_!!!!!!!!!!!!!")
      self.drone(TakeOff()).wait().success()
      while (time.time() - start_time) < RUNTIME:
        if(len(self.sensor_readings) > 0 and (time.time() - self.sensor_readings_last_received_timestamp) < 2):
          self.avg_sensor_readings = self.get_avg_sensor_readings()
          #fx_net, fy_net = apf.obs_avoid_APF(self.avg_sensor_readings, 100)
          fx_net, fy_net = apf.obs_avoid_APF(self.sensor_readings, 100)
          self.move_drone(fx_net, fy_net)
          #print(fx_net)
          #print(fy_net)
          print(self.sensor_readings)
          #print(" at " + self.sensor_readings_last_received_timestamp)
      #assert self.drone(Landing()).wait().success()
      self.drone.disconnect()
          
    def get_avg_sensor_readings(self):
        avgs = [0,0,0,0,0,0,0]
        for i in range(7):
          avg = (self.sensor_readings[i] + self.avg_sensor_readings[i]) / 2
          avgs[i] = avg
        return avgs
    
    
    def move_drone(self, fx, fy):
    
        #if(fx > 0.5):
        #  fx = 0.5
        #if(fx < -0.5):
        #  fx = -0.5
          
        #if(fy > 0.5):
        #  fy = 0.5
        #if(fy < -0.5):
        #  fy = -0.5
          
        if(fx > 0.1):
          fx = 0.3
        elif(fx < -0.1):
          fx = -0.3
        else:
          fx = 0.0
         
        if(fy > 0.1):
          fy = 0.3
        elif(fy < -0.1):
          fy = -0.3
        else:
          fy = 0.0
        
      
        if (fx<0):
          print("!WARNING flying backward with magnitude: ")
        else:
          print("!WARNING flying forward with magnitude: ")
        print(fx)
        if (fy>0):
          print("!WARNING flying left with magnitude: ")
        else:
          print("!WARNING flying right with magnitude: ")
        print(fy)
        
        assert self.drone(
            moveBy(fx, -fy, 0, 0)
            #>> FlyingStateChanged(state="hovering", _timeout=5)
        ).wait().success()
        time.sleep(1.0)
        
      
        
      



