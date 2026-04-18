from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
import json
import time

# =========================
# CHANGE THESE PARAMETERS
# =========================
broker_endpoint = "<your-endpoint>"
root_ca_path = "AmazonRootCA1.pem"

certificate_formatter = "generated_devices/vehicle{}/certificate.pem.crt"
key_formatter = "generated_devices/vehicle{}/private.pem.key"

topic = "vehicle/emission/data"


class MQTTClientWrapper:
    def __init__(self, device_id):
        self.device_id = str(device_id)
        self.client = AWSIoTMQTTClient(f"vehicle{self.device_id}")

        cert_path = certificate_formatter.format(device_id)
        key_path = key_formatter.format(device_id)

        self.client.configureEndpoint(broker_endpoint, 8883)
        self.client.configureCredentials(root_ca_path, key_path, cert_path)
        self.client.configureOfflinePublishQueueing(-1)
        self.client.configureDrainingFrequency(2)
        self.client.configureConnectDisconnectTimeout(10)
        self.client.configureMQTTOperationTimeout(5)

        self.client.onMessage = self.customOnMessage

    def connect(self):
        self.client.connect()
        print(f"vehicle{self.device_id} connected")

    def subscribe(self, topic_name):
        self.client.subscribe(topic_name, 1, self.message_callback)
        print(f"vehicle{self.device_id} subscribed to {topic_name}")

    def publish(self, topic_name, payload):
        payload_json = json.dumps(payload)
        self.client.publish(topic_name, payload_json, 1)
        print(f"vehicle{self.device_id} published to {topic_name}: {payload_json}")

    def customOnMessage(self, message):
        print(
            f"[onMessage] vehicle{self.device_id} received payload {message.payload} from topic {message.topic}"
        )

    def message_callback(self, client, userdata, message):
        print(
            f"[callback] vehicle{self.device_id} received payload {message.payload.decode()} from topic {message.topic}"
        )

    def disconnect(self):
        self.client.disconnect()
        print(f"vehicle{self.device_id} disconnected")


# Create 3 devices
vehicle1 = MQTTClientWrapper(1)
vehicle2 = MQTTClientWrapper(2)
vehicle3 = MQTTClientWrapper(3)

# Connect all
vehicle1.connect()
vehicle2.connect()
vehicle3.connect()

# vehicle3 subscribes
vehicle3.subscribe(topic)

time.sleep(2)

print("\nCommands:")
print("1 -> vehicle1 publishes")
print("2 -> vehicle2 publishes")
print("b -> both vehicle1 and vehicle2 publish")
print("q -> quit")

while True:
    cmd = input("Enter command: ").strip().lower()

    if cmd == "1":
        vehicle1.publish(topic, {
            "from_device": "vehicle1",
            "message": "hello from publisher vehicle1",
            "speed": 42
        })

    elif cmd == "2":
        vehicle2.publish(topic, {
            "from_device": "vehicle2",
            "message": "hello from publisher vehicle2",
            "speed": 55
        })

    elif cmd == "b":
        vehicle1.publish(topic, {
            "from_device": "vehicle1",
            "message": "batch message from vehicle1",
            "speed": 40
        })
        vehicle2.publish(topic, {
            "from_device": "vehicle2",
            "message": "batch message from vehicle2",
            "speed": 58
        })

    elif cmd == "q":
        break

    else:
        print("Unknown command")

    time.sleep(1)

vehicle1.disconnect()
vehicle2.disconnect()
vehicle3.disconnect()
