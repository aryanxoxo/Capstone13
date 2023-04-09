import ParrotDrone
import time

tof = ParrotDrone.ParrotDrone(dist_to_target=50, run_duration=20)
tof.start()

time.sleep(1)
print("!DEBUG: THIS IS NOT BLCOKGINNNNNNNNNNNNNNNNNN")
#time.sleep(2)
tof.join()

print("!DEBUG: The main code has now ended")


# while True:
  #print(tof.sensor_readings)
