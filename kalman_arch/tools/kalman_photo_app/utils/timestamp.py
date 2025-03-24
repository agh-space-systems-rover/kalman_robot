import re


def extract_timestamp(name):
    match = re.search(r"cube-(\d+)", name)
    if match:
        return int(match.group(1))
    raise ValueError("Timestamp not found in the provided name: " + name)
