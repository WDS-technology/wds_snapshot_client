# wds_snapshot_client/commands.py
from enum import Enum


class CommandType(str, Enum):
    SNAPSHOT = "SNAPSHOT"
    RAW_IMAGE = "RAW_IMAGE"


def normalize_command_type(s: str) -> CommandType:
    s_up = s.strip().upper()
    if s_up in ("SNAPSHOT", "RAW_IMAGE"):
        return CommandType(s_up)
    raise ValueError(f"Unknown command type: {s}")
