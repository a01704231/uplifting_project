from roboflowoak import RoboflowOak
import numpy as np
import paho.mqtt.client as mqtt
import time
import base64
import cv2
from io import BytesIO
from PIL import Image

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))

def find_fruit():
    global image
    result, frame, _, _ = rf.detect()
    predictions = result["predictions"]
    conf = 0
    detection = None
    piece_not_identified = False
    for p in predictions:
        if float(p.confidence) > conf:
            detection = str([p.class_name, p.x, p.y, p.width, p.height])
            conf = p.confidence
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    detect = len(predictions) > 0
    if conf < 85:
        detect = False
        detection = None
        piece_not_identified = True
    else:
        piece_not_identified = False
    return detect, detection, piece_not_identified

def pub_image():
    img = Image.fromarray(np.uint8(image))
    buffered = BytesIO()
    img.save(buffered, format="PNG")
    image_data = base64.b64encode(buffered.getvalue()).decode("utf-8")
    client.publish("image", image_data)

def pub_1(d, dd, n):
    client.publish("detect", d)
    client.publish("detection", dd)
    client.publish("piece", n)

global image
image = None
timeLast = 0
client = mqtt.Client()
client.on_connect = on_connect
client.connect("192.168.1.128", 1883, 60)
client.loop_start()
print("connected")
rf = RoboflowOak(model="cyberphisics-oa3xp", confidence=0.7, overlap=0.5, version="2", api_key="YvZQ7vpQ1mRu7mifMmIS", rgb=True, depth=False, device=None, blocking=True)
while True:
    d, dd, n = find_fruit()
    pub_image()
    pub_1(d, dd, n)
