import socket
import time
import threading
import artificial_potential_field
import olympe
import os
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged

class ParrotDrone(threading.Thread):
    def __init__(self, dist_to_target, run_duration, connected_drone):
      self.drone = connected_drone    
      self.dist_to_target = dist_to_target
      self.run_duration = run_duration
      self.sensor_readings = []
      self.avg_sensor_readings = [2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0]
      self.sensor_readings_last_received_timestamp = None

      threading.Thread.__init__(self)
    
    def run(self):
      print("Starting " + self.name) 
      #initialize variables
      start_time = time.time()
    
      #initialize APF algo
      self.apf = artificial_potential_field.APF()
      
      #initialize and connect drone
      #DRONE_IP = os.environ.get("DRONE_IP", "192.168.42.1")
      #self.drone = olympe.Drone(DRONE_IP) 
      #self.drone.connect()
      #print("main_drone has connected")   
      
      #initialize and connect sensors
      UDP_IP = ""   # listen on all available interfaces
      UDP_PORT = 12345      # choose an available port number
      # 2390 is the sensor's local port
      # 192.168.0.152 is the sensor's address
      # create a UDP socket object
      self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      # bind the socket to a specific IP address and port number
      self.sock.bind((UDP_IP, UDP_PORT))
        
      sensors_thread = threading.Thread(target=self.TOFSensors)
      movedrone_thread = threading.Thread(target=self.apply_apf_and_move, args=(self.dist_to_target,))
      sensors_thread.daemon = True
      sensors_thread.start()
      #self.drone(TakeOff()).wait().success()
      movedrone_thread.daemon = True
      movedrone_thread.start()
      
      while True:
        if (time.time() - start_time) > self.run_duration:
          self.drone.disconnect()
          print("socket is now closing")
          self.sock.close()
          return
        

    def TOFSensors(self):
        try:
          while True:
              # receive data and the address of the sender
              print("!DEBUG attempting to recieve message")
              # sock.setblocking(False)
              data, addr = self.sock.recvfrom(1024)  # buffer size is 1024 bytes
          
              # print received data and sender's address
              print(f"Received message: {data.decode()} from {addr}")
              
              raw_sensor_readings = data.decode('utf-8')
              self.sensor_readings = list(map(int, raw_sensor_readings.split(";")))
              self.sensor_readings_last_received_timestamp = time.time()
        except KeyboardInterrupt:
          print("Thread cancelled by keyboard interrupt.")
          return
        except Exception:
          return
          
        
    def apply_apf_and_move(self, dist_to_target):
      if(len(self.sensor_readings) > 0 and (time.time() - self.sensor_readings_last_received_timestamp) < 2):
        #self.avg_sensor_readings = self.get_avg_sensor_readings()
        #fx_net, fy_net = apf.obs_avoid_APF(self.avg_sensor_readings, 100)
        print(self.sensor_readings)
        fx_net, fy_net = self.apf.obs_avoid_APF(self.sensor_readings, dist_to_target)
        self.move_drone(fx_net, fy_net)
        time.sleep(1) 
        #print(fx_net)
        #print(fy_net)
        #print(" at " + self.sensor_readings_last_received_timestamp)
      #assert self.drone(Landing()).wait().success()
          
    def get_avg_sensor_readings(self):
        avgs = [0,0,0,0,0,0,0]
        for i in range(7):
          avg = (self.sensor_readings[i] + self.avg_sensor_readings[i]) / 2
          avgs[i] = avg
        return avgs
    
    
    def move_drone(connected_drone, fx, fy):
    
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
        
        #assert self.drone(
        #    moveBy(fx, -fy, 0, 0)
            #>> FlyingStateChanged(state="hovering", _timeout=5)
        #).wait().success()
        #time.sleep(1.0)
        
      
        
      



