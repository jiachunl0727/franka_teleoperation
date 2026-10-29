import re

_TOKEN = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


def full_topic(value, namespace="/"):
    if not isinstance(value, str) or not value or value.endswith("/"):
        raise ValueError(f"invalid ROS topic: {value!r}")
    resolved = value if value.startswith("/") else namespace.rstrip("/") + "/" + value
    if not all(_TOKEN.fullmatch(part) for part in resolved[1:].split("/")):
        raise ValueError(f"invalid ROS topic: {resolved!r}")
    return resolved
