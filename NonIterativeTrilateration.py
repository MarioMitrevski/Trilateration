import math
import random
import numpy as np
from Node import Node
import matplotlib.pyplot as plt


def trilaterate_2d(node_for_trilaterate: Node, anchor_neighbour_nodes):
    if len(anchor_neighbour_nodes) < 3:
        return
    else:
        three_anchor_neighbours = anchor_neighbour_nodes[:3]

    P1 = np.array([three_anchor_neighbours[0][0].x, three_anchor_neighbours[0][0].y])
    P2 = np.array([three_anchor_neighbours[1][0].x, three_anchor_neighbours[1][0].y])
    P3 = np.array([three_anchor_neighbours[2][0].x, three_anchor_neighbours[2][0].y])

    r1 = three_anchor_neighbours[0][1]
    r2 = three_anchor_neighbours[1][1]
    r3 = three_anchor_neighbours[2][1]

    ex = (P2 - P1) / np.linalg.norm(P2 - P1)
    i = np.dot(ex, P3 - P1)
    ey = (P3 - P1 - (i * ex)) / np.linalg.norm(P3 - P1 - (i * ex))
    d = np.linalg.norm(P2 - P1)
    j = np.dot(ey, P3 - P1)
    x = (r1 ** 2 - r2 ** 2 + d ** 2) / (2 * d)
    y = ((r1 ** 2 - r3 ** 2 + i ** 2 + j ** 2) / (2 * j)) - ((i * x) / j)

    predicted_coordinates = P1 + x * ex + y * ey
    if math.isnan(predicted_coordinates[0]) or math.isnan(predicted_coordinates[1]):
        return
    if predicted_coordinates[0] < 0 or predicted_coordinates[1] < 0 or predicted_coordinates[0] > L or \
            predicted_coordinates[1] > L:
        return

    node_for_trilaterate.x_prim = predicted_coordinates[0]
    node_for_trilaterate.y_prim = predicted_coordinates[1]
    node_for_trilaterate.is_found = True


def find_euclidean_distance(x_1, y_1, x_2, y_2):
    x_square = (x_2 - x_1) ** 2
    y_square = (y_2 - y_1) ** 2
    return math.sqrt(x_square + y_square)


def find_distance_with_error(node_to_localize, anchor):
    euclidean_distance = find_euclidean_distance(node_to_localize.x, node_to_localize.y, anchor.x, anchor.y)
    return random.gauss(euclidean_distance, math.sqrt(euclidean_distance * r))


def localize_node(node_to_localize: Node):
    neighbour_anchor_nodes = list()
    for node in sensor_nodes:
        if node.is_anchor:
            dist = find_distance_with_error(node_to_localize, node)
            if dist < R:
                neighbour_anchor_nodes.append((node, dist))

    sorted_neighbour_anchor_nodes = sorted(
        neighbour_anchor_nodes,
        key=lambda t: -t[1]
    )

    trilaterate_2d(node_to_localize, sorted_neighbour_anchor_nodes)


def localize_sensor_nodes():
    for sensor_node in sensor_nodes:
        if not sensor_node.is_anchor:
            localize_node(sensor_node)


def generate_nodes():
    sensor_network = list()
    first_f = 0

    while len(sensor_network) < N:
        new_node = Node(random.randrange(0, L), random.randrange(0, L))
        if first_f <= int(N * F):
            new_node.is_anchor = True

        if len(sensor_network) == 0:
            sensor_network.append(new_node)
            first_f += 1
        else:
            duplicate = False
            for n in sensor_network:
                if n.x == new_node.x and n.y == new_node.y:
                    duplicate = True

            if not duplicate:
                sensor_network.append(new_node)
                first_f += 1
    return sensor_network


def get_fraction_of_localized_nodes():
    sum_of_found_nodes = 0
    for node in sensor_nodes:
        if node.is_found:
            sum_of_found_nodes += 1
    return sum_of_found_nodes / (N - (int(N * F)))


def get_error_of_localization():
    sum_of_distances = 0
    for node in sensor_nodes:
        if node.is_found:
            sum_of_distances += find_euclidean_distance(node.x, node.y, node.x_prim, node.y_prim)
    return (sum_of_distances / (N - (int(N * F)))) / R


if __name__ == '__main__':
    N = 100  # number of nodes
    L = 100  # plot size
    R = 10  # radio range from 10 to 50
    r = 0.1  # radio noise from .1 to .5
    F = 0.2  # fraction of anchor nodes from .2 to .5
    total_rounds = 15

    radio_range_intervals = range(20, 60, 10)
    anchor_fraction_intervals = range(10, 52, 2)
    radio_noise_intervals = range(10, 50, 10)

    plt.ylabel('Average localized nodes')
    plt.xlabel('Anchor Fractions')
    for (radio_range, radio_noise) in zip(radio_range_intervals, radio_noise_intervals):

        avg_localized_nodes_with_anchor_fraction = []
        avg_error_of_localization = []

        R = radio_range
        r = radio_noise / 100.0
        for af in anchor_fraction_intervals:
            mean_fraction_of_localized_nodes = 0

            F = af / 100.0

            for round in range(total_rounds):
                sensor_nodes = generate_nodes()
                localize_sensor_nodes()
                mean_fraction_of_localized_nodes += get_fraction_of_localized_nodes()
            avg_localized_nodes_with_anchor_fraction.append(mean_fraction_of_localized_nodes / total_rounds * 100)
        label = f'Radio Range {radio_range}'
        plt.plot(anchor_fraction_intervals, avg_localized_nodes_with_anchor_fraction, label=label)
    plt.legend()
    plt.show()

    radio_range_intervals = range(20, 60, 10)
    anchor_fraction_intervals = range(10, 52, 2)
    radio_noise_intervals = range(10, 50, 10)

    plt.ylabel(f'Average localization error in percentage from {N}')
    plt.xlabel('Anchor Fractions')
    for (radio_range, radio_noise) in zip(radio_range_intervals, radio_noise_intervals):

        avg_error_of_localization = []

        R = radio_range
        r = radio_noise / 100.0
        for af in anchor_fraction_intervals:
            mean_error_of_localization = 0

            F = af / 100.0

            for round in range(total_rounds):
                sensor_nodes = generate_nodes()
                localize_sensor_nodes()
                mean_error_of_localization += get_error_of_localization()
            avg_error_of_localization.append(mean_error_of_localization / total_rounds * 100)
        label = f'Radio Range {radio_range}'
        plt.plot(anchor_fraction_intervals, avg_error_of_localization, label=label)
    plt.legend()
    plt.show()