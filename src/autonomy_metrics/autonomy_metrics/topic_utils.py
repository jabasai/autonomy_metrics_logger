"""Utility helpers for ROS topic configuration and message serialization."""

from __future__ import annotations

from array import array
from importlib import import_module
import math


def import_msg_type(type_str: str):
    """Import a ROS message class from ``pkg/msg/Type`` or ``pkg/Type``."""
    parts = type_str.split("/")
    if len(parts) == 2:
        package_name, message_name = parts
        submodule = "msg"
    elif len(parts) == 3:
        package_name, submodule, message_name = parts
    else:
        raise ValueError(
            "Message type must be 'pkg/msg/Type' or 'pkg/Type', "
            f"got: {type_str}"
        )

    module = import_module(f"{package_name}.{submodule}")
    return getattr(module, message_name)


def ros_msg_to_dict(value):
    """Convert a ROS 2 Python message into JSON/Mongo-friendly primitives."""
    if value is None:
        return None

    if isinstance(value, (bool, int, str)):
        return value

    if isinstance(value, float):
        if math.isfinite(value):
            return value
        return None

    if isinstance(value, bytes):
        return value.hex()

    if isinstance(value, array):
        return [ros_msg_to_dict(item) for item in value]

    if isinstance(value, (list, tuple)):
        return [ros_msg_to_dict(item) for item in value]

    if hasattr(value, "get_fields_and_field_types"):
        result = {}
        for field_name in value.get_fields_and_field_types().keys():
            result[field_name] = ros_msg_to_dict(getattr(value, field_name))
        return result

    if hasattr(value, "__dict__"):
        result = {}
        for key, item in value.__dict__.items():
            if key.startswith("_"):
                continue
            result[key] = ros_msg_to_dict(item)
        return result

    return str(value)
