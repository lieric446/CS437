import json
import time
import traceback

from awsiot import greengrasscoreipc
from awsiot.greengrasscoreipc.model import (
    QOS,
    SubscribeToTopicRequest,
    PublishToTopicRequest,
    SubscriptionResponseMessage,
    JsonMessage,
    PublishMessage
)

from process_emission import process_message

INPUT_TOPIC = "vehicle/emission/data"
OUTPUT_TOPIC = "vehicle/emission/result"


class StreamHandler(greengrasscoreipc.client.SubscribeToTopicStreamHandler):
    def __init__(self, ipc_client):
        super().__init__()
        self.ipc_client = ipc_client

    def on_stream_event(self, event: SubscriptionResponseMessage) -> None:
        try:
            message = event.binary_message or event.json_message
            if message is None:
                return

            if event.json_message:
                payload = event.json_message.message
            else:
                payload = json.loads(event.binary_message.message.decode("utf-8"))

            print(f"Received payload: {payload}")

            result = process_message(payload)
            print(f"Publishing result: {result}")

            publish_request = PublishToTopicRequest()
            publish_request.topic = OUTPUT_TOPIC
            publish_request.publish_message = PublishMessage(
                json_message=JsonMessage(
                    message=result
                )
            )

            # operation = self.ipc_client.new_publish_to_topic()
            # operation.activate(publish_request)
            # operation.get_response().result(timeout=10)
            operation = self.ipc_client.new_publish_to_topic()
            operation.activate(publish_request)
            print(f"Published result successfully to {OUTPUT_TOPIC}")

        except Exception:
            traceback.print_exc()

    def on_stream_error(self, error: Exception) -> bool:
        print(f"Stream error: {error}")
        return True

    def on_stream_closed(self) -> None:
        print("Stream closed")


def main():
    ipc_client = greengrasscoreipc.connect()

    request = SubscribeToTopicRequest()
    request.topic = INPUT_TOPIC

    handler = StreamHandler(ipc_client)
    operation = ipc_client.new_subscribe_to_topic(handler)
    operation.activate(request)
    operation.get_response().result(timeout=10)

    print(f"Subscribed to {INPUT_TOPIC}")
    print(f"Will publish results to {OUTPUT_TOPIC}")

    try:
        while True:
            time.sleep(60)
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":
    main()
