import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


AREA_M2 = 1000.0
FIELD_SIDE = math.sqrt(AREA_M2)

BASE_WEED_COUNT = 500
CLUSTER_FRACTION = 0.5
MIN_CLUSTER_SIZE = 2
MAX_CLUSTER_SIZE = 10
CLUSTER_STD_DEV = 3.0

MIN_TURNING_RADIUS = 2.0
SHORTLIST_SIZE = 10
HEADING_SAMPLES = 8
PATH_SAMPLE_STEP = 0.25
RANDOM_SEED = 42


@dataclass
class Pose:
    x: float
    y: float
    yaw: float


@dataclass
class DubinsConnection:
    goal: Pose
    segment_types: tuple
    segment_lengths: tuple
    total_length: float
    points: list


def normalize_angle(angle):
    return angle % (2.0 * math.pi)

def distance_to_boundary_ahead(pose):
    direction_x = math.cos(pose.yaw)
    direction_y = math.sin(pose.yaw)
    distances = []
    tolerance = 1e-9

    if direction_x > tolerance:
        distances.append(
            (FIELD_SIDE - pose.x) / direction_x
        )
    elif direction_x < -tolerance:
        distances.append(
            (0.0 - pose.x) / direction_x
        )

    if direction_y > tolerance:
        distances.append(
            (FIELD_SIDE - pose.y) / direction_y
        )
    elif direction_y < -tolerance:
        distances.append(
            (0.0 - pose.y) / direction_y
        )

    return min(distances)

def has_forward_clearance(pose):
    return (
        distance_to_boundary_ahead(pose)
        >= MIN_TURNING_RADIUS
    )

def turning_circle_stays_inside(center_x, center_y):
    radius = MIN_TURNING_RADIUS

    return (
        radius <= center_x <= FIELD_SIDE - radius
        and radius <= center_y <= FIELD_SIDE - radius
    )


def has_maneuvering_clearance(pose):
    if not has_forward_clearance(pose):
        return False

    radius = MIN_TURNING_RADIUS
    sine_yaw = math.sin(pose.yaw)
    cosine_yaw = math.cos(pose.yaw)

    left_center_x = pose.x - radius * sine_yaw
    left_center_y = pose.y + radius * cosine_yaw

    right_center_x = pose.x + radius * sine_yaw
    right_center_y = pose.y - radius * cosine_yaw

    left_turn_is_valid = turning_circle_stays_inside(
        left_center_x,
        left_center_y,
    )

    right_turn_is_valid = turning_circle_stays_inside(
        right_center_x,
        right_center_y,
    )

    return left_turn_is_valid or right_turn_is_valid

def generate_cluster(center, number_of_plants, rng):
    """Generate a complete cluster without allowing plants outside the field."""
    plants = []

    while len(plants) < number_of_plants:
        offset = rng.normal(0.0, CLUSTER_STD_DEV, size=2)
        plant = center + offset

        if (
            0.0 <= plant[0] <= FIELD_SIDE
            and 0.0 <= plant[1] <= FIELD_SIDE
        ):
            plants.append(plant)

    return np.array(plants)

def generate_weeds(rng):
    base_weeds = rng.uniform(
        0.0,
        FIELD_SIDE,
        size=(BASE_WEED_COUNT, 2),
    )

    number_of_cluster_centers = int(
        BASE_WEED_COUNT * CLUSTER_FRACTION
    )

    cluster_center_indices = rng.choice(
        BASE_WEED_COUNT,
        size=number_of_cluster_centers,
        replace=False,
    )

    clusters = []

    for center_index in cluster_center_indices:
        cluster_size = int(
            rng.integers(MIN_CLUSTER_SIZE, MAX_CLUSTER_SIZE + 1)
        )

        cluster = generate_cluster(
            base_weeds[center_index],
            cluster_size,
            rng,
        )

        clusters.append(cluster)

    return np.vstack([base_weeds, *clusters])

def dubins_lsl(alpha, beta, distance):
    squared_straight_length = (
        2.0
        + distance**2
        - 2.0 * math.cos(alpha - beta)
        + 2.0 * distance * (math.sin(alpha) - math.sin(beta))
    )

    if squared_straight_length < 0.0:
        return None

    temporary_angle = math.atan2(
        math.cos(beta) - math.cos(alpha),
        distance + math.sin(alpha) - math.sin(beta),
    )

    first_turn = normalize_angle(-alpha + temporary_angle)
    straight = math.sqrt(squared_straight_length)
    second_turn = normalize_angle(beta - temporary_angle)

    return first_turn, straight, second_turn


def dubins_rsr(alpha, beta, distance):
    squared_straight_length = (
        2.0
        + distance**2
        - 2.0 * math.cos(alpha - beta)
        + 2.0 * distance * (-math.sin(alpha) + math.sin(beta))
    )

    if squared_straight_length < 0.0:
        return None

    temporary_angle = math.atan2(
        math.cos(alpha) - math.cos(beta),
        distance - math.sin(alpha) + math.sin(beta),
    )

    first_turn = normalize_angle(alpha - temporary_angle)
    straight = math.sqrt(squared_straight_length)
    second_turn = normalize_angle(-beta + temporary_angle)

    return first_turn, straight, second_turn

def dubins_lsr(alpha, beta, distance):
    squared_straight_length = (
        -2.0
        + distance**2
        + 2.0 * math.cos(alpha - beta)
        + 2.0 * distance * (math.sin(alpha) + math.sin(beta))
    )

    if squared_straight_length < 0.0:
        return None

    straight = math.sqrt(squared_straight_length)

    temporary_angle = (
        math.atan2(
            -math.cos(alpha) - math.cos(beta),
            distance + math.sin(alpha) + math.sin(beta),
        )
        - math.atan2(-2.0, straight)
    )

    first_turn = normalize_angle(-alpha + temporary_angle)
    second_turn = normalize_angle(-beta + temporary_angle)

    return first_turn, straight, second_turn


def dubins_rsl(alpha, beta, distance):
    squared_straight_length = (
        distance**2
        - 2.0
        + 2.0 * math.cos(alpha - beta)
        - 2.0 * distance * (math.sin(alpha) + math.sin(beta))
    )

    if squared_straight_length < 0.0:
        return None

    straight = math.sqrt(squared_straight_length)

    temporary_angle = (
        math.atan2(
            math.cos(alpha) + math.cos(beta),
            distance - math.sin(alpha) - math.sin(beta),
        )
        - math.atan2(2.0, straight)
    )

    first_turn = normalize_angle(alpha - temporary_angle)
    second_turn = normalize_angle(beta - temporary_angle)

    return first_turn, straight, second_turn

def dubins_rlr(alpha, beta, distance):
    value = (
        6.0
        - distance**2
        + 2.0 * math.cos(alpha - beta)
        + 2.0 * distance * (math.sin(alpha) - math.sin(beta))
    ) / 8.0

    if abs(value) > 1.0:
        return None

    middle_turn = normalize_angle(
        2.0 * math.pi - math.acos(value)
    )

    first_turn = normalize_angle(
        alpha
        - math.atan2(
            math.cos(alpha) - math.cos(beta),
            distance - math.sin(alpha) + math.sin(beta),
        )
        + middle_turn / 2.0
    )

    second_turn = normalize_angle(
        alpha - beta - first_turn + middle_turn
    )

    return first_turn, middle_turn, second_turn

def dubins_lrl(alpha, beta, distance):
    value = (
        6.0
        - distance**2
        + 2.0 * math.cos(alpha - beta)
        + 2.0 * distance * (-math.sin(alpha) + math.sin(beta))
    ) / 8.0

    if abs(value) > 1.0:
        return None

    middle_turn = normalize_angle(
        2.0 * math.pi - math.acos(value)
    )

    first_turn = normalize_angle(
        -alpha
        - math.atan2(
            math.cos(alpha) - math.cos(beta),
            distance + math.sin(alpha) - math.sin(beta),
        )
        + middle_turn / 2.0
    )

    second_turn = normalize_angle(
        beta - alpha - first_turn + middle_turn
    )

    return first_turn, middle_turn, second_turn


DUBINS_PATH_TYPES = (
    (("L", "S", "L"), dubins_lsl),
    (("R", "S", "R"), dubins_rsr),
    (("L", "S", "R"), dubins_lsr),
    (("R", "S", "L"), dubins_rsl),
    (("R", "L", "R"), dubins_rlr),
    (("L", "R", "L"), dubins_lrl),
)

def calculate_dubins_candidates(start, goal):
    delta_x = goal.x - start.x
    delta_y = goal.y - start.y

    normalized_distance = (
        math.hypot(delta_x, delta_y) / MIN_TURNING_RADIUS
    )

    direction = math.atan2(delta_y, delta_x)

    alpha = normalize_angle(start.yaw - direction)
    beta = normalize_angle(goal.yaw - direction)

    candidates = []

    for segment_types, calculation_function in DUBINS_PATH_TYPES:
        segment_lengths = calculation_function(
            alpha,
            beta,
            normalized_distance,
        )

        if segment_lengths is None:
            continue

        total_length = (
            sum(segment_lengths) * MIN_TURNING_RADIUS
        )

        candidates.append(
            (
                total_length,
                segment_types,
                segment_lengths,
            )
        )

    return candidates

def advance_pose(pose, segment_type, normalized_distance):
    radius = MIN_TURNING_RADIUS

    if segment_type == "S":
        distance = normalized_distance * radius

        return Pose(
            pose.x + distance * math.cos(pose.yaw),
            pose.y + distance * math.sin(pose.yaw),
            pose.yaw,
        )

    if segment_type == "L":
        new_yaw = pose.yaw + normalized_distance

        return Pose(
            pose.x
            + radius
            * (math.sin(new_yaw) - math.sin(pose.yaw)),
            pose.y
            + radius
            * (math.cos(pose.yaw) - math.cos(new_yaw)),
            normalize_angle(new_yaw),
        )

    new_yaw = pose.yaw - normalized_distance

    return Pose(
        pose.x
        + radius
        * (math.sin(pose.yaw) - math.sin(new_yaw)),
        pose.y
        + radius
        * (math.cos(new_yaw) - math.cos(pose.yaw)),
        normalize_angle(new_yaw),
    )


def sample_dubins_path(start, segment_types, segment_lengths):
    current_pose = start
    sampled_points = [(start.x, start.y)]

    for segment_type, segment_length in zip(
        segment_types,
        segment_lengths,
    ):
        physical_length = segment_length * MIN_TURNING_RADIUS

        number_of_steps = max(
            1,
            math.ceil(physical_length / PATH_SAMPLE_STEP),
        )

        step_length = segment_length / number_of_steps

        for _ in range(number_of_steps):
            current_pose = advance_pose(
                current_pose,
                segment_type,
                step_length,
            )

            sampled_points.append(
                (current_pose.x, current_pose.y)
            )

    return sampled_points

def path_stays_inside_field(points):
    return all(
        0.0 <= x <= FIELD_SIDE
        and 0.0 <= y <= FIELD_SIDE
        for x, y in points
    )

def find_best_connection(start, target):
    possible_connections = []

    terminal_headings = np.linspace(
        0.0,
        2.0 * math.pi,
        HEADING_SAMPLES,
        endpoint=False,
    )

    for terminal_heading in terminal_headings:
        goal = Pose(
            float(target[0]),
            float(target[1]),
            float(terminal_heading),
        )

        if not has_maneuvering_clearance(goal):
            continue

        for (
            total_length,
            segment_types,
            segment_lengths,
        ) in calculate_dubins_candidates(start, goal):
            possible_connections.append(
                (
                    total_length,
                    goal,
                    segment_types,
                    segment_lengths,
                )
            )

    possible_connections.sort(key=lambda connection: connection[0])

    for (
        total_length,
        goal,
        segment_types,
        segment_lengths,
    ) in possible_connections:
        points = sample_dubins_path(
            start,
            segment_types,
            segment_lengths,
        )

        if path_stays_inside_field(points):
            return DubinsConnection(
                goal=goal,
                segment_types=segment_types,
                segment_lengths=segment_lengths,
                total_length=total_length,
                points=points,
            )

    return None

def find_nearest_indices(current_pose, weeds, unvisited):
    available_indices = np.flatnonzero(unvisited)

    differences = weeds[available_indices] - np.array(
        [current_pose.x, current_pose.y]
    )

    squared_distances = np.sum(differences**2, axis=1)

    number_to_select = min(
        SHORTLIST_SIZE,
        len(available_indices),
    )

    nearest_positions = np.argpartition(
        squared_distances,
        number_to_select - 1,
    )[:number_to_select]

    return available_indices[nearest_positions]


def select_next_connection(
    current_pose,
    weeds,
    candidate_indices,
):
    best_index = None
    best_connection = None

    for weed_index in candidate_indices:
        connection = find_best_connection(
            current_pose,
            weeds[weed_index],
        )

        if connection is None:
            continue

        if (
            best_connection is None
            or connection.total_length
            < best_connection.total_length
        ):
            best_index = int(weed_index)
            best_connection = connection

    return best_index, best_connection


def plan_path(start, weeds):
    unvisited = np.ones(len(weeds), dtype=bool)

    current_pose = start
    route_points = [(start.x, start.y)]
    visited_indices = []

    while np.any(unvisited):
        nearest_indices = find_nearest_indices(
            current_pose,
            weeds,
            unvisited,
        )

        weed_index, connection = select_next_connection(
            current_pose,
            weeds,
            nearest_indices,
        )

        if connection is None:
            all_remaining_indices = np.flatnonzero(unvisited)

            weed_index, connection = select_next_connection(
                current_pose,
                weeds,
                all_remaining_indices,
            )

        if connection is None:
            break

        route_points.extend(connection.points[1:])
        visited_indices.append(weed_index)
        unvisited[weed_index] = False
        current_pose = connection.goal

    skipped_indices = np.flatnonzero(unvisited).tolist()

    return (
        np.array(route_points),
        visited_indices,
        skipped_indices,
    )


def plot_result(
    weeds,
    start,
    route_points,
    visited_indices,
    skipped_indices,
):
    figure, axis = plt.subplots(figsize=(9, 9))

    axis.scatter(
        weeds[:, 0],
        weeds[:, 1],
        color="green",
        s=8,
        alpha=0.7,
        zorder=3,
        label="Generated weeds",
    )

    if len(route_points) > 1:
        axis.plot(
            route_points[:, 0],
            route_points[:, 1],
            color="tab:blue",
            linewidth=0.6,
            alpha=0.45,
            zorder=1,
            label="Dubins route",
        )

    if visited_indices:
        visited_weeds = weeds[visited_indices]

        axis.scatter(
            visited_weeds[:, 0],
            visited_weeds[:, 1],
            color="orange",
            s=14,
            zorder=4,
            label="Visited weeds",
        )
    if skipped_indices:
        skipped_weeds = weeds[skipped_indices]

        axis.scatter(
            skipped_weeds[:, 0],
            skipped_weeds[:, 1],
            color="red",
            marker="x",
            s=35,
            zorder=5,
            label="Unreachable weeds",
        )

    axis.scatter(
        start.x,
        start.y,
        color="black",
        marker="*",
        s=150,
        zorder=6,
        label="Robot start",
    )

    axis.set_xlim(0.0, FIELD_SIDE)
    axis.set_ylim(0.0, FIELD_SIDE)
    axis.set_aspect("equal")
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.set_title("Bounded Dubins path planning")
    axis.grid(True)
    axis.legend()

    output_path = Path.cwd() / "path_planning_result.png"

    figure.tight_layout()
    figure.savefig(output_path, dpi=150)
    plt.close(figure)

    return output_path


def main():
    rng = np.random.default_rng(RANDOM_SEED)

    weeds = generate_weeds(rng)

    while True:
        start = Pose(
            x=float(rng.uniform(0.0, FIELD_SIDE)),
            y=float(rng.uniform(0.0, FIELD_SIDE)),
            yaw=float(rng.uniform(0.0, 2.0 * math.pi)),
        )

        if has_maneuvering_clearance(start):
            break

    route_points, visited_indices, skipped_indices = plan_path(
        start,
        weeds,
    )

    output_path = plot_result(
        weeds,
        start,
        route_points,
        visited_indices,
        skipped_indices,
    )

    print(f"Generated weeds: {len(weeds)}")
    print(f"Visited weeds: {len(visited_indices)}")
    print(f"Skipped weeds: {len(skipped_indices)}")
    print(f"Plot saved to: {output_path}")


if __name__ == "__main__":
    main()