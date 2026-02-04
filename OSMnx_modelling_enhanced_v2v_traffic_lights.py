"""
Refactored AutonomousCar and simulation runner.

NOTE: This module contains placeholders for functions and objects that were referenced
in the original notebook (e.g., pathfinding routines, graph G, traffic_light_objects,
and accident_timers). They are marked with TODO and should be provided by the main
project when integrating this module.
"""

from __future__ import annotations
import time
import random
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional, Set, Tuple

import numpy as np

# ---------- Configuration / Defaults ----------
DEFAULT_COMMUNICATION_RANGE_M = 800.0  # meters
DEFAULT_SPEED_MPS = 15.65  # ~35 mph in meters/sec
DEFAULT_REEVAL_INTERVAL = 5.0  # seconds
MAX_SIMULATION_STEPS = 1000
DELTA_TIME_DEFAULT = 1.0  # seconds

# TODO: Provide these in integration: G (networkx graph), traffic_light_objects, accident_timers
G = None  # networkx graph (node attributes must include 'x' and 'y')
traffic_light_objects: Dict[Any, Any] = {}
accident_timers: Dict[Any, float] = {}
ACCIDENT_INFO_DELAY = 0.0  # seconds of delay for accident info to be considered valid

# ---------- Helper types ----------
@dataclass
class TrafficLightStateInfo:
    state: str
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

# ---------- Distance helpers (plug your implementations) ----------
def haversine_distance(lat1, lon1, lat2, lon2) -> float:
    """Placeholder. Replace with your project's haversine that returns meters."""
    # TODO: Replace with project implementation
    # Very rough Euclidean fallback (not accurate for lat/lon)
    return ((lat1 - lat2) ** 2 + (lon1 - lon2) ** 2) ** 0.5

def euclidean_distance(x1, y1, x2, y2) -> float:
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

# ---------- AutonomousCar class ----------
class AutonomousCar:
    """
    Represents an autonomous car that can move along a route, communicate via
    V2V/centralized/enhanced_v2v, and keep shared situational awareness.
    """

    def __init__(
        self,
        start_node: Any,
        end_node: Any,
        distance_metric: str = "haversine",
        path_type: str = "shortest",
        communication_strategy: str = "none",
        reeval_interval: float = DEFAULT_REEVAL_INTERVAL,
        communication_range_m: float = DEFAULT_COMMUNICATION_RANGE_M,
        speed_mps: float = DEFAULT_SPEED_MPS,
        shared_info: Optional[SharedInfo] = None,
    ) -> None:
        self.start_node = start_node
        self.end_node = end_node
        self.distance_metric = distance_metric
        self.path_type = path_type
        self.communication_strategy = communication_strategy
        self.communication_range_m = communication_range_m
        self.reeval_interval = reeval_interval
        self.speed_mps = speed_mps

        # state
        self.route: List[Any] = self.get_route() or []
        self.position_index: int = 0
        self.state: str = "moving"
        self.shared_info: SharedInfo = shared_info or SharedInfo()
        self.received_messages: Set[str] = set()
        self.travel_time: float = 0.0
        self.total_distance: float = 0.0
        self.wait_time_at_signals: float = 0.0
        self.rerouting_count: int = 0
        self.last_reeval_time: float = 0.0

    # ----------------- Routing / position -----------------
    def get_route(self) -> List[Any]:
        """
        Obtain route from start_node to end_node using given path_type.
        This is a placeholder; integrate with your pathfinding logic (e.g., OSMnx/NetworkX).
        """
        # TODO: Replace with actual pathfinding call (shortest_path_with_signals etc.)
        if self.start_node == self.end_node:
            return [self.start_node]
        # Example placeholder: a naive direct route
        return [self.start_node, self.end_node]

    def get_position(self) -> Optional[Tuple[float, float]]:
        """
        Return the (x, y) coordinates for the current node on the route.
        Expects graph G with node attributes 'x' and 'y'.
        """
        if not self.route or self.position_index >= len(self.route):
            return None
        node = self.route[self.position_index]
        if G and node in G.nodes:
            nx = G.nodes[node].get("x")
            ny = G.nodes[node].get("y")
            if nx is not None and ny is not None:
                return (nx, ny)
        return None

    # ----------------- Movement -----------------
    def move(self, delta_time: float = DELTA_TIME_DEFAULT) -> None:
        """
        Advance the car along its route based on speed; updates travel_time and total_distance.
        This simplified implementation moves the index forward by one segment per call;
        refine for continuous movement if desired.
        """
        if self.position_index >= len(self.route) - 1:
            return
        current_node = self.route[self.position_index]
        next_node = self.route[self.position_index + 1]

        # Estimate distance between nodes (requires G)
        dist = self._distance_between_nodes(current_node, next_node)
        self.total_distance += dist
        self.travel_time += delta_time

        # Advance to next node (simple discrete step)
        self.position_index += 1

    def _distance_between_nodes(self, n1: Any, n2: Any) -> float:
        if not (G and n1 in G.nodes and n2 in G.nodes):
            return 0.0
        x1, y1 = G.nodes[n1].get("x"), G.nodes[n1].get("y")
        x2, y2 = G.nodes[n2].get("x"), G.nodes[n2].get("y")
        if None in (x1, y1, x2, y2):
            return 0.0
        if self.distance_metric == "haversine":
            return haversine_distance(x1, y1, x2, y2)
        return euclidean_distance(x1, y1, x2, y2)

    # ----------------- Communication -----------------
    def communicate(self, other_cars: Iterable["AutonomousCar"]) -> None:
        """
        Dispatch communication based on strategy.
        """
        if self.communication_strategy == "v2v":
            self._v2v_communicate(other_cars)
        elif self.communication_strategy == "enhanced_v2v":
            self._enhanced_v2v_communicate(other_cars)
        elif self.communication_strategy == "centralized":
            self._centralized_communicate()
        elif self.communication_strategy == "none":
            return

    def _make_message_id(self) -> str:
        # Use car id and current position index and a timestamp to avoid collisions
        return f"car-{id(self)}-pos-{self.position_index}-t-{int(time.time())}"

    def _is_in_range(self, other: "AutonomousCar") -> bool:
        my_pos = self.get_position()
        other_pos = other.get_position()
        if not my_pos or not other_pos:
            return False
        x1, y1 = my_pos
        x2, y2 = other_pos
        if self.distance_metric == "haversine":
            d = haversine_distance(x1, y1, x2, y2)
        else:
            d = euclidean_distance(x1, y1, x2, y2)
        return d <= self.communication_range_m

    def _v2v_communicate(self, other_cars: Iterable["AutonomousCar"]) -> None:
        msg_id = self._make_message_id()
        if msg_id in self.received_messages:
            return
        self.received_messages.add(msg_id)

        current_time = time.time()
        relevant_accidents = {
            node for node in self.shared_info.accidents
            if node in self.route and current_time - accident_timers.get(node, 0) >= ACCIDENT_INFO_DELAY
        }
        relevant_closures = {node for node in self.shared_info.closures if node in self.route}

        if not relevant_accidents and not relevant_closures:
            return

        payload = {
            "accidents": relevant_accidents,
            "closures": relevant_closures,
        }

        for neighbor in (c for c in other_cars if self._is_in_range(c)):
            if msg_id in neighbor.received_messages:
                continue
            neighbor.receive_message(payload, msg_id)

    def _enhanced_v2v_communicate(self, other_cars: Iterable["AutonomousCar"]) -> None:
        """
        Share accidents, closures, own speed and nearby/route traffic light states.
        """
        msg_id = self._make_message_id()
        if msg_id in self.received_messages:
            return
        self.received_messages.add(msg_id)

        current_time = time.time()
        # collect traffic lights for route nodes and nearby nodes (within 1 km)
        traffic_light_info: Dict[Any, TrafficLightStateInfo] = {}
        my_pos = self.get_position()
        for node, tl_obj in traffic_light_objects.items():
            if G and node in G.nodes and my_pos:
                node_x, node_y = G.nodes[node].get("x"), G.nodes[node].get("y")
                if node_x is None or node_y is None:
                    continue
                # compute distance
                if self.distance_metric == "haversine":
                    d = haversine_distance(my_pos[0], my_pos[1], node_x, node_y)
                else:
                    d = euclidean_distance(my_pos[0], my_pos[1], node_x, node_y)
                if node in self.route or d <= 1000:
                    # Expect traffic light objects to expose get_state(), time_in_state, and durations
                    tl_state = getattr(tl_obj, "get_state", lambda: None)()
                    traffic_light_info[node] = TrafficLightStateInfo(
                        state=tl_state,
                        time_in_state=getattr(tl_obj, "time_in_state", 0.0),
                        timestamp=current_time,
                        cycle_duration=getattr(tl_obj, "cycle_duration", None),
                        red_duration=getattr(tl_obj, "red_duration", None),
                        yellow_duration=getattr(tl_obj, "yellow_duration", None),
                        green_duration=getattr(tl_obj, "green_duration", None),
                    )

        relevant_accidents = {
            node for node in self.shared_info.accidents
            if node in self.route and current_time - accident_timers.get(node, 0) >= ACCIDENT_INFO_DELAY
        }
        relevant_closures = {node for node in self.shared_info.closures if node in self.route}

        payload = {
            "accidents": {node: {"severity": None} for node in relevant_accidents},  # severity placeholder
            "closures": relevant_closures,
            "car_speed": self.speed_mps,
            "segment_speed_limit": self.get_segment_speed_limit(),
            "first_decision_node": self.get_first_decision_node(),
            "traffic_lights": traffic_light_info,
        }

        for neighbor in (c for c in other_cars if self._is_in_range(c)):
            if msg_id in neighbor.received_messages:
                continue
            neighbor.receive_message(payload, msg_id)

    def _centralized_communicate(self) -> None:
        """
        Placeholder for centralized communication where cars push info to a central server.
        Implement integration with your central server or aggregator here.
        """
        # TODO: implement centralized reporting if required
        return

    def receive_message(self, shared_payload: Dict[str, Any], message_id: str) -> None:
        """
        Integrate shared payload into local shared_info if message is new.
        """
        if message_id in self.received_messages:
            return
        self.received_messages.add(message_id)

        accidents = shared_payload.get("accidents", {})
        if isinstance(accidents, dict):
            self.shared_info.accidents.update(accidents.keys())
        elif isinstance(accidents, (set, list)):
            self.shared_info.accidents.update(accidents)

        closures = shared_payload.get("closures", ())
        if isinstance(closures, (set, list)):
            self.shared_info.closures.update(closures)

        # Update traffic light info keeping newest timestamps
        for node, info in shared_payload.get("traffic_lights", {}).items():
            existing = self.shared_info.traffic_lights.get(node)
            if existing is None or info.timestamp > existing.timestamp:
                self.shared_info.traffic_lights[node] = info

    # ----------------- Helpers -----------------
    def get_segment_speed_limit(self) -> Optional[float]:
        """
        Placeholder for retrieving the speed limit of the upcoming segment.
        Integrate with edge attributes on your graph.
        """
        # TODO: use G and self.route to find the next edge's speed limit
        return None

    def get_first_decision_node(self) -> Optional[Any]:
        """
        Return the first node in route ahead of current position where decision is required.
        This placeholder simply returns the next node.
        """
        if self.position_index + 1 < len(self.route):
            return self.route[self.position_index + 1]
        return None

# ---------- Simulation runner ----------
def run_simulation(
    entry_exit_distribution: Any,
    num_internal_cars: int,
    distance_metric: str = "haversine",
    path_type: str = "shortest",
    communication_strategy: str = "none",
    consider_accidents: bool = True,
    consider_road_closures: bool = True,
    consider_traffic_signals: bool = True,
    reeval_interval: float = DEFAULT_REEVAL_INTERVAL,
    delta_time: float = DELTA_TIME_DEFAULT,
    max_steps: int = MAX_SIMULATION_STEPS,
    internal_nodes: Optional[List[Any]] = None,
) -> Dict[str, Any]:
    """
    Run a simplified simulation and return aggregate metrics.

    NOTE: Many integration points are left as TODO:
    - Provide internal_nodes list (or derive from graph)
    - Provide traffic_light_objects and G globally or pass them into this function
    """
    if internal_nodes is None:
        internal_nodes = []

    # Initialize shared info that all cars can reference (optional - your architecture may differ)
    global traffic_light_objects, accident_timers
    shared_info_template = SharedInfo(
        accidents=set(),
        closures=set(),
        density={},
        traffic_lights={}
    )

    # Initialize cars
    cars: List[AutonomousCar] = []
    for _ in range(num_internal_cars):
        if not internal_nodes:
            break
        start = random.choice(internal_nodes)
        end = random.choice(internal_nodes)
        car = AutonomousCar(
            start_node=start,
            end_node=end,
            distance_metric=distance_metric,
            path_type=path_type,
            communication_strategy=communication_strategy,
            reeval_interval=reeval_interval,
            shared_info=shared_info_template
        )
        if car.route:
            cars.append(car)

    # Simulation loop
    for step in range(max_steps):
        # Update traffic lights (expects traffic light objects to have update method)
        if consider_traffic_signals:
            for tl in traffic_light_objects.values():
                update_fn = getattr(tl, "update", None)
                if callable(update_fn):
                    update_fn(delta_time)

        # Update cars
        for car in cars:
            if car.position_index < len(car.route) - 1:
                car.move(delta_time=delta_time)
                car.communicate(cars)

        # Termination: all cars reached their last node
        if all(car.position_index >= len(car.route) - 1 for car in cars):
            break

    # Metrics
    metrics = {
        "average_travel_time": float(np.mean([car.travel_time for car in cars])) if cars else 0.0,
        "total_reroutes": sum(car.rerouting_count for car in cars),
        "average_wait_time_at_signals": float(np.mean([car.wait_time_at_signals for car in cars])) if cars else 0.0,
        "total_distance_traveled": sum(car.total_distance for car in cars),
        "total_travel_time": sum(car.travel_time for car in cars),
        "total_wait_time_at_signals": sum(car.wait_time_at_signals for car in cars),
        "average_speed": float(np.mean([car.total_distance / car.travel_time if car.travel_time > 0 else 0.0 for car in cars])) if cars else 0.0,
    }
    return metrics