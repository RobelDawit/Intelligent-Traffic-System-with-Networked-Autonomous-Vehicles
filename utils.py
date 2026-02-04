"""
utils.py

Utility helpers (distance functions). Replace or extend with your project's
accurate geospatial calculations.
"""
from math import radians, sin, cos, sqrt, atan2
from typing import Tuple

# Earth radius in meters
EARTH_R = 6371000.0


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Returns the great-circle distance between two points (lat/lon) in meters.
    Input coordinates are in decimal degrees.
    """
    # Convert degrees to radians
    phi1, phi2 = radians(lat1), radians(lat2)
    dphi = radians(lat2 - lat1)
    dlambda = radians(lon2 - lon1)

    a = sin(dphi / 2.0) ** 2 + cos(phi1) * cos(phi2) * sin(dlambda / 2.0) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    return EARTH_R * c


def euclidean_distance(x1: float, y1: float, x2: float, y2: float) -> float:
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5


def make_message_id(car_obj: object, position_index: int, use_timestamp: bool = True) -> str:
    """
    Deterministic-ish message id for de-duplication across V2V messages.
    """
    base = f"car-{id(car_obj)}-pos-{position_index}"
    if use_timestamp:
        import time
        return f"{base}-t-{int(time.time())}"
    return base