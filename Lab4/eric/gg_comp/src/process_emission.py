max_co2_by_vehicle = {}

def process_message(message: dict) -> dict:
    vehicle_id = message.get("vehicle_id")
    co2 = message.get("vehicle_CO2")

    if vehicle_id is None or co2 is None:
        raise ValueError("Message must contain vehicle_id and co2")

    try:
        co2 = float(co2)
    except Exception as exc:
        raise ValueError("co2 must be numeric") from exc

    current_max = max_co2_by_vehicle.get(vehicle_id, float("-inf"))
    if co2 > current_max:
        max_co2_by_vehicle[vehicle_id] = co2

    return {
        "vehicle_id": vehicle_id,
        "max_co2": max_co2_by_vehicle[vehicle_id]
    }
