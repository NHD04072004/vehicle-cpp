import json
import time
import paho.mqtt.client as mqtt

from test_sub_mqtt_cameras import load_config, normalize_ai_modules, subscribe_camera_list

# Khớp VmsClient::handleZones (src/communication/vms_client.cpp): lọc theo ai_modules,
# `name` chỉ để hiển thị. Zone < MIN_POINTS bị runtime bỏ → test cảnh báo, không giấu.
ZONES_KEYS = ("zones", "polygons", "regions")
NESTED_KEYS = ("data", "payload", "result")
POINTS_KEYS = ("points", "polygon", "coordinates", "coords", "vertices", "area")
MIN_POINTS = 3
# Line vi phạm đi ngược chiều — khớp kReverseDirectionLine (src/common/types.h).
REVERSE_DIRECTION = "REVERSE_DIRECTION"
MIN_LINE_POINTS = 2


def build_polygon_topic(config, camera_code):
    # TOPIC_CAMERA_ZONES: smart_vms/cameras/+/zones
    return config["TOPIC_CAMERA_ZONES"].replace("+", str(camera_code))


def find_zones_array(payload):
    """Đệ quy như findZonesArray: gốc | zones/polygons/regions | data/payload/result."""
    if isinstance(payload, list):
        return payload
    if not isinstance(payload, dict):
        return None
    for key in ZONES_KEYS:
        value = payload.get(key)
        if isinstance(value, list):
            return value
    for key in NESTED_KEYS:
        if key in payload:
            nested = find_zones_array(payload[key])
            if nested is not None:
                return nested
    return None


def find_points_node(zone):
    """Như findPointsNode: zone là array điểm, hoặc điểm nằm ở một trong POINTS_KEYS."""
    if isinstance(zone, list):
        return zone
    if not isinstance(zone, dict):
        return None
    for key in POINTS_KEYS:
        value = zone.get(key)
        if isinstance(value, list):
            return value
    return None


def parse_points(node):
    """Như parsePoints: [[x,y],...] | [{"x":..,"y":..}] | [x1,y1,x2,y2,...]."""
    if not isinstance(node, list):
        return []

    def numeric(v):
        return isinstance(v, (int, float)) and not isinstance(v, bool)

    if node and all(numeric(v) for v in node):
        return [(node[i], node[i + 1]) for i in range(0, len(node) - 1, 2)]

    points = []
    for item in node:
        if isinstance(item, list) and len(item) >= 2 and numeric(item[0]) and numeric(item[1]):
            points.append((item[0], item[1]))
        elif isinstance(item, dict):
            x = next((item[k] for k in ("x", "X", "left") if numeric(item.get(k))), None)
            y = next((item[k] for k in ("y", "Y", "top") if numeric(item.get(k))), None)
            if x is not None and y is not None:
                points.append((x, y))
    return points


def zone_name_of(zone):
    """Như asString(node, "name", asString(node, "zone_name"))."""
    if not isinstance(zone, dict):
        return ""
    return str(zone.get("name") or zone.get("zone_name") or "")


def describe_zones(payload, ai_module):
    """Mô tả mọi zone trong payload: tên, ai_modules, số điểm, có được runtime nhận không."""
    zones = find_zones_array(payload)
    if zones is None:
        return None

    described = []
    for zone in zones:
        points = parse_points(find_points_node(zone))
        modules = normalize_ai_modules(zone.get("ai_modules") if isinstance(zone, dict) else None)
        described.append(
            {
                "name": zone_name_of(zone),
                "ai_modules": modules,
                "num_points": len(points),
                "module_match": ai_module in modules,
                "enough_points": len(points) >= MIN_POINTS,
                "raw": zone,
            }
        )
    return described


def find_lines_array(payload):
    """Mảng "lines" nằm song song với "zones" — khớp findLinesArray (vms_client.cpp)."""
    if not isinstance(payload, dict):
        return None
    if isinstance(payload.get("lines"), list):
        return payload["lines"]
    for key in NESTED_KEYS:
        if key in payload:
            nested = find_lines_array(payload[key])
            if nested is not None:
                return nested
    return None


def parse_direction(node):
    """direction_vector: [dx,dy] hoặc {"x":..,"y":..}; None nếu vector ~0."""

    def numeric(v):
        return isinstance(v, (int, float)) and not isinstance(v, bool)

    if isinstance(node, list) and len(node) >= 2 and numeric(node[0]) and numeric(node[1]):
        vec = (node[0], node[1])
    elif isinstance(node, dict) and numeric(node.get("x")) and numeric(node.get("y")):
        vec = (node["x"], node["y"])
    else:
        return None
    return vec if abs(vec[0]) > 1e-9 or abs(vec[1]) > 1e-9 else None


def has_direction_flag(line):
    if not isinstance(line, dict):
        return False
    cfg = line.get("config")
    if isinstance(cfg, dict) and isinstance(cfg.get("has_direction"), bool):
        return cfg["has_direction"]
    return bool(line.get("has_direction"))


def describe_lines(payload, ai_module):
    """Mô tả mọi line: tên, số điểm, direction — và có dùng được cho WRONG_WAY không."""
    lines = find_lines_array(payload)
    if lines is None:
        return None

    described = []
    for line in lines:
        points = parse_points(find_points_node(line))
        modules = normalize_ai_modules(line.get("ai_modules") if isinstance(line, dict) else None)
        direction = (
            parse_direction(line.get("direction_vector"))
            if has_direction_flag(line) and isinstance(line, dict)
            else None
        )
        name = zone_name_of(line)
        described.append(
            {
                "name": name,
                "ai_modules": modules,
                "num_points": len(points),
                "points": points,
                "direction": direction,
                "is_reverse": name.strip().upper() == REVERSE_DIRECTION,
                "module_match": ai_module in modules,
                "enough_points": len(points) >= MIN_LINE_POINTS,
            }
        )
    return described


def print_lines_report(described):
    if described is None:
        print("--- không có mảng 'lines' trong payload ---")
        return

    print(f"--- {len(described)} line(s) ---")
    for i, ln in enumerate(described):
        if not ln["is_reverse"]:
            verdict = f"BỎ QUA (tên != {REVERSE_DIRECTION})"
        elif not ln["enough_points"]:
            verdict = f"BỊ LOẠI (chỉ {ln['num_points']} điểm, cần >= {MIN_LINE_POINTS})"
        elif ln["direction"] is None:
            verdict = "BỊ LOẠI (thiếu direction_vector/has_direction)"
        else:
            verdict = "DÙNG ĐƯỢC cho WRONG_WAY"
        print(
            f"  [{i}] name={ln['name']!r} ai_modules={ln['ai_modules']} "
            f"points={ln['num_points']} direction={ln['direction']} -> {verdict}"
        )


def print_report(payload, described, ai_module):
    print("--- FULL PAYLOAD ---")
    print(json.dumps(payload, indent=2, ensure_ascii=False))

    if described is None:
        print(f"!! Không tìm thấy mảng zones (đã thử {ZONES_KEYS} + lồng trong {NESTED_KEYS})")
        return

    print(f"--- {len(described)} zone(s), ai_module={ai_module} ---")
    for i, z in enumerate(described):
        if not z["module_match"]:
            verdict = f"BỎ QUA (ai_modules không chứa {ai_module})"
        elif not z["enough_points"]:
            verdict = f"BỊ RUNTIME LOẠI (chỉ {z['num_points']} điểm, cần >= {MIN_POINTS})"
        else:
            verdict = "NHẬN"
        print(
            f"  [{i}] name={z['name']!r} ai_modules={z['ai_modules']} "
            f"points={z['num_points']} -> {verdict}"
        )


def on_connect(client, userdata, flags, reason_code, properties=None):
    print("Connected, reason code:", reason_code)
    topic = userdata["topic"]
    qos = userdata.get("qos", 1)
    client.subscribe(topic, qos=qos)
    print("Subscribed to:", topic)


def on_message(client, userdata, msg):
    print("Received on:", msg.topic)
    raw = msg.payload.decode("utf-8", errors="replace")
    try:
        payload = json.loads(raw)
    except json.JSONDecodeError:
        print("!! Payload không phải JSON hợp lệ, raw:")
        print(raw)
        return

    ai_module = userdata["ai_module"]
    described = describe_zones(payload, ai_module)
    print_report(payload, described, ai_module)

    lines = describe_lines(payload, ai_module)
    print_lines_report(lines)

    userdata["result"].append(
        {"payload": payload, "zones": described or [], "lines": lines or []}
    )


def subscribe_polygon(config, camera_code, timeout=10, ai_module=None):
    topic = build_polygon_topic(config, camera_code)
    result = []

    client = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2,
        userdata={
            "topic": topic,
            "qos": config["QOS_COMMANDS"],
            "result": result,
            "ai_module": ai_module or config["AI_MODULE"],
        },
    )
    client.username_pw_set(config["MQTT_USERNAME"], config["MQTT_PASSWORD"])

    client.on_connect = on_connect
    client.on_message = on_message

    client.connect(
        config["BROKER_HOST"], config["BROKER_PORT"], keepalive=config["KEEPALIVE"]
    )

    print(f"Listening on '{topic}' for {timeout}s...")
    client.loop_start()
    time.sleep(timeout)
    client.loop_stop()
    client.disconnect()

    return result


def pick_camera_with_zones(config, cameras, timeout=5, ai_module=None):
    """Ưu tiên camera có zone dùng được cho ai_module; fallback camera có payload bất kỳ."""
    module = ai_module or config["AI_MODULE"]
    fallback = (None, [])
    for camera in cameras:
        messages = subscribe_polygon(config, camera["code"], timeout=timeout, ai_module=module)
        if not messages:
            continue
        usable = any(
            z["module_match"] and z["enough_points"]
            for m in messages
            for z in m["zones"]
        )
        if usable:
            return camera, messages
        if fallback[0] is None:
            fallback = (camera, messages)
    return fallback


if __name__ == "__main__":
    cfg = load_config()

    cameras = subscribe_camera_list(cfg)
    if not cameras:
        raise SystemExit("No camera found for ai_modules=" + cfg["AI_MODULE"])

    camera, messages = pick_camera_with_zones(cfg, cameras)
    if not camera:
        raise SystemExit(
            "Không nhận được payload zones nào cho camera với ai_modules=" + cfg["AI_MODULE"]
        )

    total = sum(len(m["zones"]) for m in messages)
    usable = sum(
        1 for m in messages for z in m["zones"] if z["module_match"] and z["enough_points"]
    )
    reverse_lines = sum(
        1
        for m in messages
        for ln in m["lines"]
        if ln["is_reverse"] and ln["enough_points"] and ln["direction"] is not None
    )
    print(
        f"Using camera {camera['code']}: {len(messages)} message(s), "
        f"{total} zone(s), {usable} dùng được cho ai_module={cfg['AI_MODULE']}, "
        f"{reverse_lines} line {REVERSE_DIRECTION} dùng được (WRONG_WAY)"
    )
