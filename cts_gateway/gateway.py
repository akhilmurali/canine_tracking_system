import paho.mqtt.client as mqtt
import ssl
import _thread

run_flag = True
#Event Handlers
def on_connect_aws_broker(local_client, userdata, flags, rc):    
    print("Connection returned result: " + str(rc) )

def on_message_aws_broker(local_client, userdata, msg):
    print("topic: "+msg.topic+"     payload: "+str(msg.payload))

def on_connect_local_broker(local_client, userdata, flags, rc):
    # rc is the error code returned when connecting to the broker
    print("Connected!", str(rc))
    # Once the local_client has connected to the broker, subscribe to the topic
    local_client.subscribe(local_mqtt_topic)

def on_message_local_broker(local_client, userdata, msg):
    print("Local Topic: ", msg.topic + "\nMessage Payload: " + str(msg.payload))
    #hostendpoint: a37mwryi7jvbcj-ats.iot.ap-south-1.amazonaws.com
    #Publish to AWS IoT Core broker when message is received.
    aws_client.publish("topic/cts_gateway_message_handle", str(msg.payload), qos=1)        
    print("Sent message to aws broker")

local_mqtt_username = "pine_mos"
local_mqtt_password = "1234"
local_mqtt_topic = "topic/cts_local_proxy_topic"
local_mqtt_broker_ip = "192.168.1.8"

local_client = mqtt.Client()
local_client.username_pw_set(local_mqtt_username, local_mqtt_password)
#Bind hanlders to local_client
local_client.on_connect = on_connect_local_broker
local_client.on_message = on_message_local_broker
local_client.connect(local_mqtt_broker_ip, 1883)

aws_client = mqtt.Client()
awshost = "a37mwryi7jvbcj-ats.iot.ap-south-1.amazonaws.com"
awsport = 8883
clientId = "cts-gateway-device"
thingName = "cts-gateway-device"
caPath = "/home/pine/source/repos/cts_gateway/certificates/AmazonRootCA1.pem"
#path to certificate
certPath = "/home/pine/source/repos/cts_gateway/certificates/1abf3aafde-certificate.pem.crt"
#path to private key
keyPath = "/home/pine/source/repos/cts_gateway/certificates/1abf3aafde-private.pem.key"
#set the TLS parameters
aws_client.tls_set(caPath, certfile=certPath, keyfile=keyPath, cert_reqs=ssl.CERT_REQUIRED, tls_version=ssl.PROTOCOL_TLSv1_2, ciphers=None)
aws_client.on_connect = on_connect_aws_broker
aws_client.on_message = on_message_aws_broker
#establish a connection with the AWS endpoint
aws_client.connect(awshost, awsport, keepalive=60)

#Attach Loop related functions:
while (run_flag):
    aws_client.loop(0.01)
    local_client.loop(0.01)