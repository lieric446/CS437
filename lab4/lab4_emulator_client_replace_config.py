from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
import time
import json
import pandas as pd


# =========================
# CHANGE THESE PARAMETERS
# =========================

# If you created vehicle1, vehicle2, vehicle3
device_st = 1
device_end = 6   # range(1, 6) => 1, 2, 3, 4, 5

# CSV files
data_path = "vehicle_data/vehicle{}.csv"

# Device certs
certificate_formatter = "generated_devices/vehicle{}/certificate.pem.crt"
key_formatter = "generated_devices/vehicle{}/private.pem.key"

# Root CA file
root_ca_path = "AmazonRootCA1.pem"

# Your AWS IoT endpoint
broker_endpoint = "<your-endpoint>"

# Topic
publish_topic = "vehicle/emission/data"


class MQTTClient:
    def __init__(self, device_id, cert, key):
        self.device_id = str(device_id)
        self.client = AWSIoTMQTTClient(f"vehicle{self.device_id}")

        self.client.configureEndpoint(broker_endpoint, 8883)
        self.client.configureCredentials(root_ca_path, key, cert)
        self.client.configureOfflinePublishQueueing(-1)
        self.client.configureDrainingFrequency(2)
        self.client.configureConnectDisconnectTimeout(10)
        self.client.configureMQTTOperationTimeout(5)
        self.client.onMessage = self.customOnMessage

        # Load this device's CSV once
        self.df = pd.read_csv(data_path.format(self.device_id))
        self.current_row = 0

    def customOnMessage(self, message):
        print(
            f"client {self.device_id} received payload {message.payload} from topic {message.topic}"
        )

    def customSubackCallback(self, mid, data):
        pass

    def customPubackCallback(self, mid):
        pass

    def publish_one_row(self, topic=publish_topic):
        if self.current_row >= len(self.df):
            print(f"vehicle{self.device_id}: no more rows left to publish")
            return

        row = self.df.iloc[self.current_row]
        payload = json.dumps(row.to_dict())

        print(f"Publishing from vehicle{self.device_id}: {payload} to {topic}")
        self.client.publishAsync(topic, payload, 0, ackCallback=self.customPubackCallback)

        self.current_row += 1


print("Initializing MQTT clients...")
clients = []

for device_id in range(device_st, device_end):
    cert_path = certificate_formatter.format(device_id)
    key_path = key_formatter.format(device_id)

    client = MQTTClient(device_id, cert_path, key_path)
    client.client.connect()
    clients.append(client)
    print(f"Connected vehicle{device_id}")

print("\nReady.")
print("Press 's' to send ONE message from each device")
print("Press 'd' to disconnect and quit")

while True:
    x = input("Enter command: ").strip().lower()

    if x == "s":
        for c in clients:
            c.publish_one_row()

    elif x == "d":
        for c in clients:
            c.client.disconnect()
        print("All devices disconnected")
        break

    else:
        print("Wrong key pressed. Use 's' or 'd'.")

    time.sleep(1)
