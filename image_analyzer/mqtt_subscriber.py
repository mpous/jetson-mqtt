import paho.mqtt.client as mqtt
import base64
import cv2
import numpy as np
from io import BytesIO


# MQTT settings
MQTT_BROKER = "192.168.1.156"
MQTT_PORT = 1883
MQTT_SUB_TOPIC = "balena/site/area/line/cell/camera/raw"
MQTT_PUB_TOPIC = "balena/site/area/line/cell/camera/inference"

# Initialize OpenCV's face detector (assuming you're using the default OpenCV haarcascade_frontalface_default.xml)
#face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    # Subscribe to the topic
    client.subscribe(MQTT_SUB_TOPIC)

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    #print(f"Received message: {msg.topic} {str(msg.payload)}")
    print(f"Received message from topic: {msg.topic} ")
    
    # Decode the base64 string to an image
    img_data = base64.b64decode(msg.payload)
    nparr = np.fromstring(img_data, np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    
    # Process the image with OpenCV for face detection
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #faces = face_cascade.detectMultiScale(gray, 1.1, 4)
    
    #for (x, y, w, h) in faces:
    #    cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)
    
    # For demonstration purposes, let's save the image with detected faces
    #cv2.imwrite('detected_faces.jpg', img)
    
    # Encode the processed image back to a base64 string
    _, buffer = cv2.imencode('.jpg', img)
    io_buf = BytesIO(buffer)
    base64_image = base64.b64encode(io_buf.getvalue()).decode('utf-8')

    # Publish the base64 string of the processed image
    client.publish(MQTT_PUB_TOPIC, base64_image)
    print("Processed image published.")
    
    # Publish a response to another topic
    #response_message = "Response to received message"
    #client.publish(MQTT_PUB_TOPIC, response_message)

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(MQTT_BROKER, MQTT_PORT, 60)

# Blocking call that processes network traffic, dispatches callbacks, and handles reconnecting.
client.loop_forever()

