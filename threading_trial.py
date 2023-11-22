import threading
import time
import random

kalman_event = threading.Event()
control_event = threading.Event()
image_event = threading.Event()
local_nav_event = threading.Event()

position = 0

def kalman_filter():
    global position
    while True:
        image_event.wait()
        position += 1
        print("kalman filter \n")
        image_event.clear()
        kalman_event.set()
        time.sleep(2)

def control_alg():
    global position
    while True:
        local_nav_event.wait() 
        print("control \n")
        local_nav_event.clear()
        control_event.set()
        time.sleep(2)

def image():
    global position
    while True:
        control_event.wait()
        position = random.randint(1, 5)
        print("image processing \n")
        control_event.clear() 
        image_event.set()  
        time.sleep(2)

def local_nav():
    while True:
        kalman_event.wait()  
        print("local navigation \n")
        kalman_event.clear()  
        local_nav_event.set() 
        time.sleep(2)
        

kalman = threading.Thread(name='kalman_filter', target=kalman_filter)
control = threading.Thread(name='control_alg', target=control_alg)
image_thread = threading.Thread(name='image', target=image)
local_nav = threading.Thread(name='loc_nav', target=local_nav)

image_thread.start()
control_event.set()
local_nav.start()
kalman.start()
control.start()

time.sleep(10)  

image_thread.join()
local_nav.join()
kalman.join()
control.join()
