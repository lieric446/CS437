from dotenv import load_dotenv
import os
import boto3
from pathlib import Path


load_dotenv()

REGION = os.getenv('AWS_REGION')
THING_GROUP = os.getenv('THING_GROUP')
POLICY_NAME = os.getenv('POLICY_NAME')


# define methods
def check_thing_exists(client, thingName): 
    try:
        client.describe_thing(thingName=thingName)
        return True
    except Exception:
        return False

def create_thing(client, thingName):
    try:
        resp = client.create_thing(thingName=thingName)
        print(f"Created thing with name {thingName}")
        return resp
    except Exception as e:
        print(f"Failed to create {thingName} with error {e}")
        raise

def create_keys_and_certificate(client):
    try:
        resp = client.create_keys_and_certificate(setAsActive=True)
        print("Generated keys and cert")
        return resp
    except Exception as e:
        print(f"Failed to generate keys and cert with error: {e}")
        raise

def write_to_file(path: Path, content: str):
    path.write_text(content, encoding="utf-8")

def save_cert(thingName, certData):
    out_dir = Path(os.getenv("VEHICLES_CERTS_FOLDER"))
    thing_dir = out_dir / thingName
    thing_dir.mkdir(parents=True, exist_ok=True)

    write_to_file(thing_dir / "certificate.pem.crt", certData["certificatePem"])
    write_to_file(thing_dir / "private.pem.key", certData["keyPair"]["PrivateKey"])
    write_to_file(thing_dir / "public.pem.key", certData["keyPair"]["PublicKey"])

    print(f"Saved cert files for: {thingName}")

def attach_policy(client, policy, target):
    try:
        client.attach_policy(
            policyName=policy, 
            target=target
        )
        print("Policy attached")
    except Exception as e:
        print(f"Failed to attach policy {policy} to target {target} with error {e}")
        raise

def attach_thing_principal(client, thingName, principal):
    try:
        resp = client.attach_thing_principal(
            thingName=thingName,
            principal=principal
        )
        return resp
    except Exception as e:
        print(f"Failed to attach principal {principal} to thing {thingName} with error {e}")
        raise

def add_thing_to_thing_group(client, thingName, thingGroupName):
    try:
        resp = client.add_thing_to_thing_group(
            thingName=thingName,
            thingGroupName=thingGroupName
        )
        return resp
    except Exception as e:
        print(f"Failed to add thing {thingName} to thing group {thingGroupName} with error {e}")

# instantiate AWS IoT client
try:
    iot_client = boto3.client('iot', region_name=REGION)
    print(f"AWS IoT connected")
except Exception as e:
    print(f"Conn failed with: {e}")
    raise

# create three devices for now
for i in range(0, 5):
    thing_name = f"vehicle{i}"
    print(f"----- Instantiating: {thing_name} -----")

    if check_thing_exists(iot_client, thing_name):
        print(f"Thing of name {thing_name} already exists... skipping")
        continue

    resp = create_thing(iot_client, thing_name)
    # print(resp)

    cert_data = create_keys_and_certificate(iot_client)
    # print(cert_data)
    save_cert(thing_name, cert_data)

    cert_arn = cert_data["certificateArn"]
    attach_policy(iot_client, POLICY_NAME, cert_arn)
    attach_thing_principal(iot_client, thing_name, cert_arn)
    add_thing_to_thing_group(iot_client, thing_name, THING_GROUP)

    print(f"----- Finished instantiating: {thing_name} -----\n")

    
    
