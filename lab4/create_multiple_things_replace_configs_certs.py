import json
import os
from pathlib import Path

import boto3
from botocore.exceptions import ClientError

# ----------------------------
# Configuration
# ----------------------------
REGION = "<your-region>"
THING_GROUP_NAME = "<your-thing-group>"
POLICY_NAME = "<your-policy>"

# Keep this small to avoid unnecessary usage
DEVICE_COUNT = 5

# Device names: vehicle1, vehicle2, vehicle3...
THING_NAME_PREFIX = "vehicle"

# Where to save certificates locally
OUTPUT_DIR = Path("generated_devices")


def ensure_output_dir() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)


def write_text_file(path: Path, content: str) -> None:
    path.write_text(content, encoding="utf-8")


def thing_exists(iot_client, thing_name: str) -> bool:
    try:
        iot_client.describe_thing(thingName=thing_name)
        return True
    except ClientError as e:
        error_code = e.response["Error"]["Code"]
        if error_code == "ResourceNotFoundException":
            return False
        raise


def create_thing(iot_client, thing_name: str) -> None:
    if thing_exists(iot_client, thing_name):
        print(f"[SKIP] Thing already exists: {thing_name}")
        return

    iot_client.create_thing(thingName=thing_name)
    print(f"[OK] Created thing: {thing_name}")


def create_cert_and_keys(iot_client) -> dict:
    response = iot_client.create_keys_and_certificate(setAsActive=True)
    print(f"[OK] Created certificate: {response['certificateId']}")
    return response


def attach_policy(iot_client, certificate_arn: str) -> None:
    iot_client.attach_policy(
        policyName=POLICY_NAME,
        target=certificate_arn
    )
    print(f"[OK] Attached policy: {POLICY_NAME}")


def attach_thing_principal(iot_client, thing_name: str, certificate_arn: str) -> None:
    iot_client.attach_thing_principal(
        thingName=thing_name,
        principal=certificate_arn
    )
    print(f"[OK] Attached certificate to thing: {thing_name}")


def add_thing_to_group(iot_client, thing_name: str) -> None:
    iot_client.add_thing_to_thing_group(
        thingName=thing_name,
        thingGroupName=THING_GROUP_NAME
    )
    print(f"[OK] Added {thing_name} to group: {THING_GROUP_NAME}")


def save_device_files(thing_name: str, cert_response: dict) -> None:
    thing_dir = OUTPUT_DIR / thing_name
    thing_dir.mkdir(parents=True, exist_ok=True)

    # Save certificate material
    # <change>
    write_text_file(thing_dir / "certificate.pem.crt", cert_response["certificatePem"])
    write_text_file(thing_dir / "private.pem.key", cert_response["keyPair"]["PrivateKey"])
    write_text_file(thing_dir / "public.pem.key", cert_response["keyPair"]["PublicKey"])

    # Save metadata too
    metadata = {
        "thingName": thing_name,
        "certificateArn": cert_response["certificateArn"],
        "certificateId": cert_response["certificateId"],
    }
    write_text_file(thing_dir / "metadata.json", json.dumps(metadata, indent=2))
    print(f"[OK] Saved cert files for: {thing_name}")


def main() -> None:
    ensure_output_dir()

    session = boto3.Session(region_name=REGION)
    iot_client = session.client("iot")

    for i in range(3, DEVICE_COUNT + 1):
        thing_name = f"{THING_NAME_PREFIX}{i}"
        print(f"\n=== Processing {thing_name} ===")

        try:
            create_thing(iot_client, thing_name)

            cert_response = create_cert_and_keys(iot_client)
            certificate_arn = cert_response["certificateArn"]

            attach_policy(iot_client, certificate_arn)
            attach_thing_principal(iot_client, thing_name, certificate_arn)
            add_thing_to_group(iot_client, thing_name)
            save_device_files(thing_name, cert_response)

            print(f"[DONE] {thing_name} fully configured")

        except ClientError as e:
            print(f"[ERROR] Failed for {thing_name}: {e}")
        except Exception as e:
            print(f"[ERROR] Unexpected failure for {thing_name}: {e}")


if __name__ == "__main__":
    main()
