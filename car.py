"""
car.py

AutonomousCar class. Depends on models.py and utils.py.
Keep integration points (get_route, speed limits, graph access) replaced with
your project's actual implementations.
"""
from typing import Any, Dict, Iterable, List, Optional, Set
import time

from models import SharedInfo, TrafficLightStateInfo, G, traffic_light_objects, accident_timers, ACCIDENT_INFO_DELAY
from utils import haversine_distance, euclidean_distance, make_message_id


DEFAULT_COMMUNICATION_RANGE_M = 800.0
DEFAULT_SPEED_MPS = 15.65  # ~35 mph


class AutonomousCar:
    def __init__(
        self,
        start_node: Any,
        end_node: Any,
        distance_metric: str = "haversine",
        path_type: str = "shortest",
        communication_strategy: str = "none",
        reeval_interval: float = 5.0,
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
        Placeholder for route calculation. Replace with your pathfinder (OSMnx / networkx).
        """
        if self.start_node == self.end_node:
            return [self.start_node]
        # Basic placeholder; replace with proper pathfinding
        return [self.start_node, self.end_node]

    def get_position(self) -> Optional[tuple]:
        """
        Return (x, y) of the current node on route (from G).
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
    def move(self, delta_time: float = 1.0) -> None:
        """
        Simple discrete movement: advances to next node and updates distance/time.
        Replace with continuous movement model if required.
        """
        if self.position_index >= len(self.route) - 1:
            return
        current_node = self.route[self.position_index]
        next_node = self.route[self.position_index + 1]

        dist = self._distance_between_nodes(current_node, next_node)
        self.total_distance += dist
        self.travel_time += delta_time

        # simple discrete step
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
        if self.communication_strategy == "v2v":
            self._v2v_communicate(other_cars)
        elif self.communication_strategy == "enhanced_v2v":
            self._enhanced_v2v_communicate(other_cars)
        elif self.communication_strategy == "centralized":
            self._centralized_communicate()
        elif self.communication_strategy == "none":
            return

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
        msg_id = make_message_id(self, self.position_index)
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

        payload = {"accidents": relevant_accidents, "closures": relevant_closures}

        for neighbor in (c for c in other_cars if self._is_in_range(c)):
            if msg_id in neighbor.received_messages:
                continue
            neighbor.receive_message(payload, msg_id)

    def _enhanced_v2v_communicate(self, other_cars: Iterable["AutonomousCar"]) -> None:
        msg_id = make_message_id(self, self.position_index)
        if msg_id in self.received_messages:
            return
        self.received_messages.add(msg_id)

        current_time = time.time()
        traffic_light_info: Dict[Any, TrafficLightStateInfo] = {}
        my_pos = self.get_position()
        for node, tl_obj in traffic_light_objects.items():
            if G and node in G.nodes and my_pos:
                node_x, node_y = G.nodes[node].get("x"), G.nodes[node].get("y")
                if node_x is None or node_y is None:
                    continue
                if self.distance_metric == "haversine":
                    d = haversine_distance(my_pos[0], my_pos[1], node_x, node_y)
                else:
                    d = euclidean_distance(my_pos[0], my_pos[1], node_x, node_y)
                if node in self.route or d <= 1000:
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
            "accidents": {node: {"severity": None} for node in relevant_accidents},
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
        # TODO: implement central aggregator integration
        return

    def receive_message(self, shared_payload: Dict[str, Any], message_id: str) -> None:
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

        for node, info in shared_payload.get("traffic_lights", {}).items():
            existing = self.shared_info.traffic_lights.get(node)
            # info may be TrafficLightStateInfo or a dict; accept both
            if isinstance(info, TrafficLightStateInfo):
                incoming_ts = info.timestamp
            else:
                incoming_ts = info.get("timestamp", 0)
            if existing is None or incoming_ts > existing.timestamp:
                # store as TrafficLightStateInfo
                if isinstance(info, TrafficLightStateInfo):
                    self.shared_info.traffic_lights[node] = info
                else:
                    self.shared_info.traffic_lights[node] = TrafficLightStateInfo(
                        state=info.get("state"),
                        time_in_state=info.get("time_in_state", 0.0),
                        timestamp=incoming_ts,
                        cycle_duration=info.get("cycle_duration"),
                        red_duration=info.get("red_duration"),
                        yellow_duration=info.get("yellow_duration"),
                        green_duration=info.get("green_duration"),
                    )

    # ----------------- Helpers -----------------
    def get_segment_speed_limit(self):
        # TODO: inspect next edge in self.route using G and return its speed limit attribute
        return None

    def get_first_decision_node(self):
        if self.position_index + 1 < len(self.route):
            return self.route[self.position_index + 1]
        return None