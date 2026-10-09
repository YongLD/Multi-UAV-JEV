"""Viewer camera settings. Independent of the aircraft's onboard camera."""
import math

DEFAULT_CAMERA = {"mode": "follow", "azimuth": 0., "elevation": -13., "distance": 3.8}


def validate_camera(value):
    if not isinstance(value, dict) or value.get("mode") not in ("follow", "orbit", "top"):
        raise ValueError("invalid camera mode")
    result = {"mode": value["mode"]}
    for name, low, high in (("azimuth", -36000., 36000.),
                            ("elevation", -89., -5.), ("distance", 2., 180.)):
        number = float(value.get(name, DEFAULT_CAMERA[name]))
        if not math.isfinite(number):
            raise ValueError("invalid camera value")
        result[name] = max(low, min(high, number))
    result["azimuth"] %= 360
    return result
