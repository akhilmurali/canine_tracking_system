import paho.mqtt.client as mqtt
import os
import socket
import ssl

#Event Handlers
# def on_connect_aws_broker(client, userdata, flags, rc):    
#     print("Connection returned result: " + str(rc) )

# def on_message_aws_broker(client, userdata, msg):
#     print("topic: "+msg.topic+"     payload: "+str(msg.payload))

def on_connect_local_broker(client, userdata, flags, rc):
    # rc is the error code returned when connecting to the broker
    print("Connected!", str(rc))
    # Once the client has connected to the broker, subscribe to the topic
    client.subscribe(local_mqtt_topic)

def on_message_local_broker(client, userdata, msg):
    print("Topic: ", msg.topic + "\nMessage: " + str(msg.payload))
    #hostendpoint: a37mwryi7jvbcj-ats.iot.ap-south-1.amazonaws.com
    #Publish to AWS IoT Core broker when message is received.
    print("Received message from the broker")
    # aws_client.publish("topic/water_level_breach", msg, qos=1)        
    # print("Sent message to aws broker")
    
local_mqtt_username = "pine_mos"
local_mqtt_password = "1234"
local_mqtt_topic = "topic/cts_local_proxy_topic"
local_mqtt_broker_ip = "192.168.1.8"

client = mqtt.Client()
client.username_pw_set(local_mqtt_username, local_mqtt_password)
# aws_client = mqtt.Client()
# aws_client.on_connect = on_connect_aws_broker
# aws_client.on_message = on_message_aws_broker

# #AWS related configuration
# awshost = "a1d4f7y1gy75pm.iot.ap-south-1.amazonaws.com"
# #Secure communication, so 8883
# awsport = 8883
# clientId = "MyIOTThing"
# thingName = "MyIOTThing"
# #path to the certificate authority file
# caPath = "/home/pi/Desktop/vws_gateway/Certificates/AmazonRootCA1-new.pem"
# #path to certificate
# certPath = "/home/pi/Desktop/vws_gateway/Certificates/0f39adf6de-certificate.pem.crt.txt"
# #path to private key
# keyPath = "/home/pi/Desktop/vws_gateway/Certificates/0f39adf6de-private.pem.key"
# #set the TLS parameters
# aws_client.tls_set(caPath, certfile=certPath, keyfile=keyPath, cert_reqs=ssl.CERT_REQUIRED, tls_version=ssl.PROTOCOL_TLSv1_2, ciphers=None)
# #establish a connection with the AWS endpoint
# aws_client.connect(awshost, awsport, keepalive=60)
# aws_client.loop_forever()
# #AWS related configuration

#Bind hanlders to client
client.on_connect = on_connect_local_broker
client.on_message = on_message_local_broker

client.connect(local_mqtt_broker_ip, 1883)
client.loop_forever()