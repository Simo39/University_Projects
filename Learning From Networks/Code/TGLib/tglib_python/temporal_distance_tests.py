import pytglib as tgl
import doctest


def temporal_fastest_path_distance_test():
    """
    Test for computing the temporal distances.

    >>> temporal_fastest_path_distance_test()
    True
    VectorTime[0, 1, 4, 7]
    """
    tgs = tgl.load_ordered_edge_list("../example_datasets/example_from_paper.tg")
    c = tgl.minimum_durations(tgs, 0, tgs.getTimeInterval())

    tg = tgl.to_incident_lists(tgs)
    c2 = tgl.minimum_durations(tg, 0, tgs.getTimeInterval())
    print(c == c2)

    return c


def temporal_earliest_arrival_path_distance_test():
    """
    Test for computing the temporal distances.

    >>> temporal_earliest_arrival_path_distance_test()
    True
    VectorTime[0, 3, 6, 8]
    """
    tgs = tgl.load_ordered_edge_list("../example_datasets/example_from_paper.tg")
    c = tgl.earliest_arrival_times(tgs, 0, tgs.getTimeInterval())

    tg = tgl.to_incident_lists(tgs)
    c2 = tgl.earliest_arrival_times(tg, 0, tgs.getTimeInterval())
    print(c == c2)

    return c


def temporal_shortest_path_distance_test():
    """
    Test for computing the temporal distances.

    >>> temporal_shortest_path_distance_test()
    True
    VectorTime[0, 1, 3, 7]
    """
    tgs = tgl.load_ordered_edge_list("../example_datasets/example_from_paper.tg")
    c = tgl.minimum_transition_times(tgs, 0, tgs.getTimeInterval())

    tg = tgl.to_incident_lists(tgs)
    c2 = tgl.minimum_transition_times(tg, 0, tgs.getTimeInterval())
    print(c == c2)

    return c


if __name__ == "__main__":
    doctest.testmod()
