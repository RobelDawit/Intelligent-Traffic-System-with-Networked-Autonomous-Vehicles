"""
sim.py

Simulation runner using AutonomousCar from car.py. Minimal runner — integrates
traffic light updates and car updates. Replace placeholders with your own
entry/exit generation and graph configuration.
"""
from typing import Any, Dict, List, Optional
import random
import numpy as np

from models import traffic_light_objects, accident_timers
from car import AutonomousCar
from models import SharedInfo

DEFAULT_REEVAL_INTERVAL = 5.0
DEFAULT_DELTA_TIME = 1.0
DEFAULT_MAX_STEPS = 1000


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
    delta_time: float = DEFAULT_DELTA_TIME,
    max_steps: int = DEFAULT_MAX_STEPS,
    internal_nodes: Optional[List[Any]] = None,
) -> Dict[str, Any]:
    """
    Run simulation and return metrics.

    Note: internal_nodes must be provided or derived from G externally.
    """
    if internal_nodes is None:
        internal_nodes = []

    shared_info_template = SharedInfo()

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
            shared_info=shared_info_template,
        )
        if car.route:
            cars.append(car)

    for step in range(max_steps):
        # Update traffic lights
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

        if all(car.position_index >= len(car.route) - 1 for car in cars):
            break

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