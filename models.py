"""
models.py

Dataclasses for shared info and placeholders for project-level globals.

TODO: Replace placeholders (G, traffic_light_objects, accident_timers) with
your project's actual objects or pass them into functions/classes instead of
using globals.
"""
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Set

# Project-level placeholders (replace or inject these from your project)
G: Optional[Any] = None  # expected to be a networkx Graph with node attrs 'x', 'y'
traffic_light_objects: Dict[Any, Any] = {}  # node_id -> traffic light object
accident_timers: Dict[Any, float] = {}  # node_id -> timestamp when accident reported

ACCIDENT_INFO_DELAY: float = 0.0  # seconds delay before accident info becomes "valid"


@dataclass
class TrafficLightStateInfo:
    state: Optional[str]
    time_in_state: float
    timestamp: float
    cycle_duration: Optional[float] = None
    red_duration: Optional[float] = None
    yellow_duration: Optional[float] = None
    green_duration: Optional[float] = None


@dataclass
class SharedInfo:
    accidents: Set[Any] = field(default_factory=set)
    closures: Set[Any] = field(default_factory=set)
    density: Dict[Any, int] = field(default_factory=dict)
    traffic_lights: Dict[Any, TrafficLightStateInfo] = field(default_factory=dict)