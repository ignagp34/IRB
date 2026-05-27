DEFAULT_WORKSPACE_LIMITS = {
    "x": (0.05, 0.75),
    "y": (0.05, 0.85),
    "z": (0.85, 1.65),
}

DEFAULT_TOOL0_WORKSPACE_LIMITS = {
    "x": (0.05, 0.75),
    "y": (0.05, 0.85),
    "z": (0.95, 1.65),
}

DEFAULT_SLOTS = {
    "slot_a": {
        "label": "white_slot",
        "aliases": ["slot_a", "white_slot", "left_container"],
        "pose": [0.105, 0.100, 0.90, 0.707, 0.707, 0.0, 0.0],
    },
    "slot_b": {
        "label": "black_slot",
        "aliases": ["slot_b", "black_slot"],
        "pose": [0.105, 0.145, 0.90, 0.707, 0.707, 0.0, 0.0],
    },
    "slot_c": {
        "label": "blue_slot",
        "aliases": ["slot_c", "blue_slot", "right_container"],
        "pose": [0.150, 0.145, 0.90, 0.707, 0.707, 0.0, 0.0],
    },
    "unknown_slot": {
        "label": "unknown_slot",
        "aliases": ["unknown_slot", "no_detection_slot"],
        "pose": [0.150, 0.100, 0.90, 0.707, 0.707, 0.0, 0.0],
    },
}

COLOR_ALIASES = {
    "blue": {"blue", "bluecube", "blue_cube", "azul"},
    "black": {"black", "blackcube", "black_cube", "negro"},
    "white": {"white", "whitecube", "white_cube", "blanco"},
    "cube": {"cube", "unknown", "generic_cube"},
}
