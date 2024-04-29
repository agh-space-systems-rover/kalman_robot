import array
import math
import struct
import numpy as np
import rosidl_runtime_py.convert as ros_convert
from typing import Any
from threading import Lock
from rosidl_parser.definition import (
    BasicType,
    INTEGER_TYPES,
    FLOATING_POINT_TYPES,
    BOOLEAN_TYPE,
    UnboundedString,
    UnboundedSequence,
    Array,
    NamespacedType,
    AbstractType,
)


class IntQuantizer:
    def __init__(self, min: int, max: int):
        self.min = min
        self.max = max
        self.bytes = math.ceil((max - min).bit_length() / 8)

    # Compress integer into bytes.
    def compress(self, value: int) -> bytes:
        # Check if the value is within the range.
        if value < self.min:
            raise ValueError(f"Value {value} is below the minimum of {self.min}")
        if value > self.max:
            raise ValueError(f"Value {value} is above the maximum of {self.max}")

        # Remap the value to the range.
        value -= self.min

        # Convert the value to bytes.
        return value.to_bytes(self.bytes, "big")

    # Decompress an array of bytes into an integer.
    def decompress(self, bytes: list[bool]) -> int:
        # Convert bytes to an integer.
        value = int.from_bytes(bytes, "big")

        # Remap the value to the original range.
        value += self.min

        return value


class FloatQuantizer:
    def __init__(self, min: float, max: float, bytes: int):
        self.min = min
        self.max = max
        self.bytes = bytes

    # Compress float into a byte array.
    def compress(self, value: float) -> bytes:
        # Check if the value is within the range.
        if value < self.min:
            raise ValueError(f"Value {value} is below the minimum of {self.min}")
        if value > self.max:
            raise ValueError(f"Value {value} is above the maximum of {self.max}")

        # Remap the value to 0..1 range.
        value -= self.min
        value /= self.max - self.min

        number_of_steps = 2 ** (self.bytes * 8)
        integer = int(value * (number_of_steps - 1) + 0.5)

        return integer.to_bytes(self.bytes, "big")

    # Decompress bytes into a float.
    def decompress(self, bytes: bytes) -> float:
        # Convert bytes to an integer.
        integer = int.from_bytes(bytes, "big")

        # Remap the integer to 0..1 range.
        number_of_steps = 2 ** (self.bytes * 8)
        value = integer / (number_of_steps - 1)

        # Remap the value to the original range.
        value *= self.max - self.min
        value += self.min

        return value


class StringEnum:
    def __init__(self, values: list[str], fallback: str):
        if len(values) == 0:
            raise ValueError("Empty string enum.")

        self.values = values
        self.fallback = fallback
        self.bytes = math.ceil(math.log2(len(values)))

    # Compress string into bytes.
    def compress(self, value: str) -> bytes:
        if value not in self.values:
            value = self.fallback

        if value in self.values:
            index = self.values.index(value)
        else:
            index = len(self.values)
        return index.to_bytes(self.bytes, "big")

    # Decompress bytes into a string.
    def decompress(self, bytes: list[bool]) -> str:
        index = int.from_bytes(bytes, "big")
        if index >= len(self.values):
            return self.fallback
        return self.values[index]


# Dynamically import a class with caching.
cached_import_classes = {}
cached_import_classes_lock = Lock()


def cached_import_class(class_path: str) -> Any:
    with cached_import_classes_lock:
        # Check if the module is already cached.
        if class_path not in cached_import_classes:
            # Split the path into module and class names.
            module_name, class_name = class_path.rsplit(".", 1)
            # Import the module.
            module = __import__(module_name, fromlist=[class_name])
            # Get the class from the module.
            class_ = getattr(module, class_name)
            # Cache the class.
            cached_import_classes[class_path] = class_

        return cached_import_classes[class_path]


# Get a primitive type from rosidl definition.
def primitive_type_from_rosidl_definition(type: AbstractType) -> type:
    if isinstance(type, BasicType):
        if type.typename == BOOLEAN_TYPE:
            return bool
        if type.typename in INTEGER_TYPES:
            return int
        if type.typename in FLOATING_POINT_TYPES:
            return float
    if isinstance(type, UnboundedString):
        return str
    if isinstance(type, UnboundedSequence):
        # If the element type is BasicType, return array.array, otherwise a list.
        if isinstance(type.value_type, BasicType):
            return array.array
        else:
            return list
    if isinstance(type, Array):
        return np.ndarray
    if isinstance(type, NamespacedType):
        module_path = ".".join(type.namespaces)
        class_name = type.name
        return cached_import_class(f"{module_path}.{class_name}")


# Get a nested value from any object by its path.
# Empty yields the object itself.
def get_nested_value(obj, path: str) -> Any:
    path_items = path.split(".") if path else []
    for key in path_items:
        if (
            isinstance(obj, array.array)
            or isinstance(obj, list)
            or isinstance(obj, np.ndarray)
        ):
            obj = obj[int(key)]
        else:
            obj = getattr(obj, key)
            # NOTE: getattr() might throw if the path is invalid.
    return obj


# Set a nested value in any object by its path.
# Empty path is not allowed.
def set_nested_value(obj, path: str, value) -> None:
    if not path:
        raise ValueError("Empty path is not allowed.")

    path_items = path.split(".") if path else []
    for key in path_items[:-1]:
        if (
            isinstance(obj, array.array)
            or isinstance(obj, list)
            or isinstance(obj, np.ndarray)
        ):
            obj = obj[int(key)]
        else:
            obj = getattr(obj, key)
            # NOTE: getattr() might throw if the path is invalid.

    if (
        isinstance(obj, array.array)
        or isinstance(obj, list)
        or isinstance(obj, np.ndarray)
    ):
        obj[int(path_items[-1])] = value
    else:
        setattr(obj, path_items[-1], value)


# Return true if the path starts with any of the prefixes
# or if any of the prefixes start with the path.
def does_path_match_any_prefix(prefixes: list[str], path: str) -> bool:
    path_items = path.split(".") if path else []

    for prefix in prefixes:
        prefix_items = prefix.split(".") if prefix else []

        if (
            path_items[: len(prefix_items)] == prefix_items
            or prefix_items[: len(path_items)] == path_items
        ):
            return True
    return False


# Read paths from config.fields and return them as a list.
# fields:
#   - path: "path.to.field"
#   - path: "path.to.field.but.longer"
# ----------> ["path.to.field", "path.to.field.but.longer"]
def get_prefixes_from_fields_config(fields: dict) -> list[str]:
    return [field["path"] for field in fields]


# Find the longest field object that matches the given path and has the specified key.
# Return the value of the key or None if not found.
# fields:
#   - path: "path.to.field",
#     key: value
#   - path: "path.to.field.but.longer",
#     key: value                               <--- this value will be returned
def get_key_from_deepest_matching_field(
    fields: dict, path: str, key: str
) -> dict | None:
    # Filter out fields that do not contain the key.
    fields = [field for field in fields if key in field]
    # Sort the fields by path length in descending order.
    fields = sorted(fields, key=lambda x: len(x["path"]), reverse=True)
    # Pick first matching field.
    for field in fields:
        if does_path_match_any_prefix([field["path"]], path):
            return field[key]
    return None


# Serialize any ROS message or basic type into a list of bytes.
def serialize_message(msg, fields: dict) -> list[int]:
    prefixes = get_prefixes_from_fields_config(fields)

    def walk_msg(path="") -> list[int]:
        # Skip the field if it does not match any of the prefixes.
        if not does_path_match_any_prefix(prefixes, path):
            return []

        sub_msg = get_nested_value(msg, path)

        # If the field is a boolean, serialize it as a single byte.
        if isinstance(sub_msg, bool):
            return [1 if sub_msg else 0]

        # If the field is an integer, serialize it using int_range compression parameters.
        if isinstance(sub_msg, int):
            int_range_config = get_key_from_deepest_matching_field(
                fields, path, "int_range"
            )
            if int_range_config is not None:
                try:
                    return IntQuantizer(
                        int_range_config["min"], int_range_config["max"]
                    ).compress(sub_msg)
                except ValueError as e:
                    raise ValueError(f"Failed to compress field {path}:\n{e}")
            else:
                return sub_msg.to_bytes(4, "big")

        # If the field is a float, serialize it using float_range compression parameters.
        if isinstance(sub_msg, float):
            float_range_config = get_key_from_deepest_matching_field(
                fields, path, "float_range"
            )
            if float_range_config is not None:
                try:
                    return FloatQuantizer(
                        float_range_config["min"],
                        float_range_config["max"],
                        float_range_config["bytes"],
                    ).compress(sub_msg)
                except ValueError as e:
                    raise ValueError(f"Failed to compress field {path}:\n{e}")
            else:
                return struct.pack("f", sub_msg)

        # If the field is a string, serialize it raw.
        if isinstance(sub_msg, str):
            string_enum_config = get_key_from_deepest_matching_field(
                fields, path, "string_enum"
            )
            if string_enum_config is not None:
                try:
                    return StringEnum(
                        string_enum_config["values"], string_enum_config["fallback"]
                    ).compress(sub_msg)
                except ValueError as e:
                    raise ValueError(f"Failed to compress field {path}:\n{e}")
            else:
                return [ord(c) for c in sub_msg] + [0]  # remember to add a null

        # If the field is a list, serialize it in order.
        if (
            isinstance(sub_msg, array.array)
            or isinstance(sub_msg, list)
            or isinstance(sub_msg, np.ndarray)
        ):
            length = len(sub_msg)
            if length > 255:
                raise ValueError(f"List length {length} is too large to serialize.")
            bytes = [length]
            for i in range(length):
                bytes += walk_msg(f"{path}.{i}".lstrip("."))
            return bytes

        # Else the field is a ROS message, serialize it recursively.
        bytes = []
        slot_types = ros_convert.get_message_slot_types(type(sub_msg))
        for slot_name, _ in slot_types.items():
            bytes += walk_msg(f"{path}.{slot_name}".lstrip("."))
        return bytes

    bytes = walk_msg()
    return bytes


# Deserialize a list of bytes into a ROS message.
def deserialize_message(data: list[int], msg_type: type, fields: dict) -> Any:
    prefixes = get_prefixes_from_fields_config(fields)

    msg = msg_type()

    # Returns new data_i.
    def walk_msg(path="", data_i=0) -> int:
        if len(prefixes) > 0 and not does_path_match_any_prefix(prefixes, path):
            return data_i

        if data_i >= len(data) and path != "":
            raise ValueError(f"Data index {data_i} is out of bounds.")
        # If path is empty, sub_msg will be a message type, which does not necessarily require data to deserialize in case of empty message.

        sub_msg = get_nested_value(msg, path)

        # If the field is a boolean, deserialize it from a single byte.
        if isinstance(sub_msg, bool):
            set_nested_value(msg, path, bool(data[data_i]))
            return data_i + 1

        # If the field is an integer, deserialize it using int_range compression parameters.
        if isinstance(sub_msg, int):
            int_range_config = get_key_from_deepest_matching_field(
                fields, path, "int_range"
            )
            if int_range_config is not None:
                try:
                    int_quant = IntQuantizer(
                        int_range_config["min"], int_range_config["max"]
                    )
                    set_nested_value(
                        msg,
                        path,
                        int_quant.decompress(data[data_i : data_i + int_quant.bytes]),
                    )
                    return data_i + int_quant.bytes
                except ValueError as e:
                    raise ValueError(f"Failed to decompress field {path}:\n{e}")
            else:
                set_nested_value(
                    msg, path, int.from_bytes(data[data_i : data_i + 4], "big")
                )
                return data_i + 4

        # If the field is a float, deserialize it using float_range compression parameters.
        if isinstance(sub_msg, float):
            float_range_config = get_key_from_deepest_matching_field(
                fields, path, "float_range"
            )
            if float_range_config is not None:
                try:
                    float_quant = FloatQuantizer(
                        float_range_config["min"],
                        float_range_config["max"],
                        float_range_config["bytes"],
                    )
                    set_nested_value(
                        msg,
                        path,
                        float_quant.decompress(
                            data[data_i : data_i + float_quant.bytes]
                        ),
                    )
                    return data_i + float_quant.bytes
                except ValueError as e:
                    raise ValueError(f"Failed to decompress field {path}:\n{e}")
            else:
                set_nested_value(
                    msg, path, struct.unpack("f", bytes(data[data_i : data_i + 4]))[0]
                )
                return data_i + 4

        # If the field is a string, deserialize it raw.
        if isinstance(sub_msg, str):
            string_enum_config = get_key_from_deepest_matching_field(
                fields, path, "string_enum"
            )
            if string_enum_config is not None:
                try:
                    enum = StringEnum(
                        string_enum_config["values"], string_enum_config["fallback"]
                    )
                    set_nested_value(
                        msg, path, enum.decompress(data[data_i : data_i + enum.bytes])
                    )
                    return data_i + enum.bytes
                except ValueError as e:
                    raise ValueError(f"Failed to decompress field {path}:\n{e}")
            else:
                string = ""
                while data[data_i] != 0:
                    string += chr(data[data_i])
                    data_i += 1
                set_nested_value(msg, path, string)
                data_i += 1  # skip the null
                return data_i

        # If the field is a list, deserialize it in order.
        if (
            isinstance(sub_msg, array.array)
            or isinstance(sub_msg, list)
            or isinstance(sub_msg, np.ndarray)
        ):
            # If the field is a regular Python list, it will need to be extended to the correct length before deserialization.
            length = data[data_i]
            if not isinstance(sub_msg, np.ndarray):
                # In order to extend the list, we need to know the type of the elements to construct.
                # Split the current path to find the parent message type, and the name of this field.
                # We will query ROS message system to find the type of the elements in this list.
                split_path = path.rsplit(".", 1)
                parent_path, field_name = (
                    split_path if len(split_path) == 2 else ("", split_path[0])
                )
                parent_msg = get_nested_value(msg, parent_path)
                element_def = ros_convert.get_message_slot_types(type(parent_msg))[
                    field_name
                ].value_type
                element_type = primitive_type_from_rosidl_definition(element_def)

                set_nested_value(msg, path, [element_type() for _ in range(length)])
            # If the field is a numpy array, it will already have the correct length and types.

            # Now let's deserialize the elements one by one.
            data_i += 1
            for i in range(length):
                data_i = walk_msg(f"{path}.{i}".lstrip("."), data_i)
            return data_i

        # Else the field is a ROS message, deserialize it recursively.
        slot_types = ros_convert.get_message_slot_types(type(sub_msg))
        for slot_name, slot_type in slot_types.items():
            data_i = walk_msg(f"{path}.{slot_name}".lstrip("."), data_i)
        return data_i

    walk_msg()
    return msg
