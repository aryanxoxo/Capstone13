import sys
import os
sys.path.append('/home/HwHiAiUser/Documents/Drone/gesture-controlled-drone-main/src/')
#sys.path.append('/home/HwHiAiUser/Ascend/ascend-toolkit/5.0.2.alpha003/arm64-linux/acllib/include/')
os.environ['LD_LIBRARY_PATH'] = '/home/HwHiAiUser/code/gsdk3/out/olympe-linux/final/usr/lib:' + os.environ.get('LD_LIBRARY_PATH', '')
from djitellopy import Tello
import openpose_tf
from atlas_utils.presenteragent import presenter_channel
from atlas_utils.acl_image import AclImage
import cv2
from utils.params import params
import time
from queue import Queue
from threading import Thread
# from atlas_utils.presenteragent import presenter_channel  


#Olympe
import csv
import numpy as np
import math
import queue
import shlex
import subprocess
import tempfile
import traceback

import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing
from olympe.messages.ardrone3.Piloting import moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.messages.ardrone3.PilotingSettings import MaxTilt
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.arsdkng.pdraw import VideoFrame
#Olympe

#Parrot
import ParrotDrone_CK
#Parrot

from utils.shared_variable import Shared
import threading
# from threading import Thread
from utils.runlive import PresenterServer
import openpose_tf

olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})

DRONE_IP = "192.168.42.1"

class StreamAndFindPerson(threading.Thread):

    def __init__(self, connected_drone):
        # Create the olympe.Drone object from its IP address
        self.drone = connected_drone
        self.tempd = tempfile.mkdtemp(prefix="olympe_streaming_test_")
        print("Olympe streaming example output dir: {}".format(self.tempd))
        self.h264_frame_stats = []
        self.h264_stats_file = open(
            os.path.join(self.tempd, 'h264_stats.csv'), 'w+')
        self.h264_stats_writer = csv.DictWriter(
            self.h264_stats_file, ['fps', 'bitrate'])
        self.h264_stats_writer.writeheader()
        self.frame_queue = queue.Queue()
        self.flush_queue_lock = threading.Lock()
        
        
        super().__init__()
        super().start()

    def start(self):
        # Connect the the drone
        #self.drone.connect()

        # You can record the video stream from the drone if you plan to do some
        # post processing.
        self.drone.set_streaming_output_files(
            h264_data_file=os.path.join(self.tempd, 'h264_data.264'),
            h264_meta_file=os.path.join(self.tempd, 'h264_metadata.json'),
            # Here, we don't record the (huge) raw YUV video stream
            # raw_data_file=os.path.join(self.tempd,'raw_data.bin'),
            # raw_meta_file=os.path.join(self.tempd,'raw_metadata.json'),
        )

        # Setup your callback functions to do some live video processing
        self.drone.set_streaming_callbacks(
            raw_cb=self.yuv_frame_cb,
            h264_cb=self.h264_frame_cb,
            start_cb=self.start_cb,
            end_cb=self.end_cb,
            flush_raw_cb=self.flush_cb,
        )
        # Start video streaming
        self.drone.start_video_streaming()

    def stop(self):
        # Properly stop the video stream and disconnect
        self.drone.stop_video_streaming()
        self.drone.disconnect()
        self.h264_stats_file.close()

    def yuv_frame_cb(self, yuv_frame):
        """
        This function will be called by Olympe for each decoded YUV frame.
            :type yuv_frame: olympe.VideoFrame
        """
        
        

        
        yuv_frame.ref()
        self.frame_queue.put_nowait(yuv_frame)

    def flush_cb(self):
        with self.flush_queue_lock:
            while not self.frame_queue.empty():
                self.frame_queue.get_nowait().unref()
        return True

    def start_cb(self):
        pass

    def end_cb(self):
        pass

    def h264_frame_cb(self, h264_frame):
        """
        This function will be called by Olympe for each new h264 frame.
            :type yuv_frame: olympe.VideoFrame
        """

        # Get a ctypes pointer and size for this h264 frame
        frame_pointer, frame_size = h264_frame.as_ctypes_pointer()

        # For this example we will just compute some basic video stream stats
        # (bitrate and FPS) but we could choose to resend it over an another
        # interface or to decode it with our preferred hardware decoder..

        # Compute some stats and dump them in a csv file
        info = h264_frame.info()
        frame_ts = info["ntp_raw_timestamp"]
        if not bool(info["h264"]["is_sync"]):
            if len(self.h264_frame_stats) > 0:
                while True:
                    start_ts, _ = self.h264_frame_stats[0]
                    if (start_ts + 1e6) < frame_ts:
                        self.h264_frame_stats.pop(0)
                    else:
                        break
            self.h264_frame_stats.append((frame_ts, frame_size))
            h264_fps = len(self.h264_frame_stats)
            h264_bitrate = (
                8 * sum(map(lambda t: t[1], self.h264_frame_stats)))
            self.h264_stats_writer.writerow(
                {'fps': h264_fps, 'bitrate': h264_bitrate})

    def show_yuv_frame(self, window_name, yuv_frame):
        # the VideoFrame.info() dictionary contains some useful information
        # such as the video resolution
        info = yuv_frame.info()
        height, width = info["yuv"]["height"], info["yuv"]["width"]

        # yuv_frame.vmeta() returns a dictionary that contains additional
        # metadata from the drone (GPS coordinates, battery percentage, ...)

        # convert pdraw YUV flag to OpenCV YUV flag
        cv2_cvt_color_flag = {
            olympe.PDRAW_YUV_FORMAT_I420: cv2.COLOR_YUV2BGR_I420,
            olympe.PDRAW_YUV_FORMAT_NV12: cv2.COLOR_YUV2BGR_NV12,
        }[info["yuv"]["format"]]

        # yuv_frame.as_ndarray() is a 2D numpy array with the proper "shape"
        # i.e (3 * height / 2, width) because it's a YUV I420 or NV12 frame

        # Use OpenCV to convert the yuv frame to RGB
        cv2frame = cv2.cvtColor(yuv_frame.as_ndarray(), cv2_cvt_color_flag)
        # Use OpenCV to show this frame
        cv2.imshow(window_name, cv2frame)
        cv2.waitKey(1)  # please OpenCV for 1 ms...

    def convert_yuv_frame(self, yuv_frame):
        info = yuv_frame.info()
        height, width = info["yuv"]["height"], info["yuv"]["width"]

        cv2_cvt_color_flag = {
            olympe.PDRAW_YUV_FORMAT_I420: cv2.COLOR_YUV2BGR_I420,
            olympe.PDRAW_YUV_FORMAT_NV12: cv2.COLOR_YUV2BGR_NV12,
        }[info["yuv"]["format"]]

        cv2frame = cv2.cvtColor(yuv_frame.as_ndarray(), cv2_cvt_color_flag)
        return cv2frame


    def run(self):
        window_name = "Olympe Streaming Example"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        main_thread = next(
            filter(lambda t: t.name == "MainThread", threading.enumerate())
        )
        while main_thread.is_alive():
            with self.flush_queue_lock:
                try:
                    yuv_frame = self.frame_queue.get(timeout=0.01)
                except queue.Empty:
                    continue
                try:
                    self.show_yuv_frame(window_name, yuv_frame)
                except Exception:
                    # We have to continue popping frame from the queue even if
                    # we fail to show one frame
                    traceback.print_exc()
                finally:
                    # Don't forget to unref the yuv frame. We don't want to
                    # starve the video buffer pool
                    yuv_frame.unref()
        cv2.destroyWindow(window_name)

    def get_cv2_frame(self):
        main_thread = next(
            filter(lambda t: t.name == "MainThread", threading.enumerate())
        )
        while main_thread.is_alive():
            with self.flush_queue_lock:
                try:
                    yuv_frame = self.frame_queue.get(timeout=0.01)
                except queue.Empty:
                    continue
                try:
                    cv2_frame = self.convert_yuv_frame(yuv_frame)
                except Exception:
                    # We have to continue popping frame from the queue even if
                    # we fail to show one frame
                    traceback.print_exc()
                finally:
                    # Don't forget to unref the yuv frame. We don't want to
                    # starve the video buffer pool
                    yuv_frame.unref()
                    # Not sure if this it the right place to return without messing up threading
                    return cv2_frame

    def postprocessing(self):
        # Convert the raw .264 file into an .mp4 file
        h264_filepath = os.path.join(self.tempd, 'h264_data.264')
        mp4_filepath = os.path.join(self.tempd, 'h264_data.mp4')
        subprocess.run(
            shlex.split('ffmpeg -i {} -c:v copy -y {}'.format(
                h264_filepath, mp4_filepath)),
            check=True
        )


def takeoff(drone):
    assert drone(TakeOff()).wait().success()

def land(drone):
    assert drone(Landing()).wait().success()

def show(chan, frame):
    _, jpeg_image = cv2.imencode('.jpg', frame)
    jpeg_image = AclImage(jpeg_image, frame.shape[0], frame.shape[1], jpeg_image.size)
    chan.send_detection_data(frame.shape[0], frame.shape[1], jpeg_image, [])
    
def init_presenter_server():
    SRC_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    PRESENTER_SERVER_CONF = os.path.join(SRC_PATH, "uav_presenter_server.conf")
    chan = presenter_channel.open_channel(PRESENTER_SERVER_CONF)
    if chan is None:
        raise Exception("Open presenter channel failed")
    return chan
    
def send_to_presenter_server(chan, frame_org, result_img):
    _, jpeg_image = cv2.imencode('.jpg', result_img)
    jpeg_image = AclImage(jpeg_image, frame_org.shape[0], frame_org.shape[1], jpeg_image.size)
    chan.send_detection_data(frame_org.shape[0], frame_org.shape[1], jpeg_image, [])

def find_closest_person(results):
    if results is not None:
      # find the entry with the biggest 'dist' attribute. this is the person who is closest to you
      return max(results, key=lambda x: x['dist'])
    


if __name__ == "__main__":
    RUNTIME = 20
    #uav_presenter_conf = params["presenter_server_conf"]
    #print(uav_presenter_conf)
    #chan = init_presenter_server()
    DRONE_IP = os.environ.get("DRONE_IP", "192.168.42.1")
    connected_drone = olympe.Drone(DRONE_IP) 
    connected_drone.connect()
    print("main_drone has connected") 
    streaming_example = StreamAndFindPerson(connected_drone)
    streaming_example.start()
    #frame= cv2.cvtColor(yuv_frame.as_ndarray(), cv2_cvt_color_flag)
    #print(frame)
    drone = ParrotDrone_CK.ParrotDrone(dist_to_target=50, run_duration=RUNTIME+10, connected_drone=connected_drone)
    drone.start()

    openpose_tf.init(openpose_tf.MODEL_PATH)
    error= 0
    start_time = time.time()
    
    print("!DEBUG: The ML code is starting")
    while (time.time() - start_time) < RUNTIME:
        frame = streaming_example.get_cv2_frame()
        #print(type(frame))
        #print(frame.dtype)
        results = openpose_tf.get_poses_in_img(frame)
        #process the results here
        person_to_track = find_closest_person(results)
        print(person_to_track)
        #has to be scaling_dist - the person's "distance". because the "distance"
        #represents the distance from the person's nose to their neck.
        #i.e. if this is bigger, the subject is closer. if this is smaller, 
        #the subject is farther. scaling_dist = 150 is chosen arbitrarily.
        scaling_dist = 150
        dist_to_target = scaling_dist - person_to_track['dist']
        #send_to_presenter_server(chan,frame,results)
        #if len(results) == 0:
        #     continue
        #print(results[0]["area"])

    
    drone.join()
    print("!DEBUG: The main code has now ended")