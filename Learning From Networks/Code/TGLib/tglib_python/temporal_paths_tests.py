import pytglib as tgl
import doctest


def temporal_fastest_path_test():
    """
    Test for computing the temporal distances.

    >>> temporal_fastest_path_test()
    VectorTemporalEdge[(0 1 5 2), (1 2 7 2)]
    """
    tgs = tgl.load_ordered_edge_list("../example_datasets/example_from_paper.tg")
    tg = tgl.to_incident_lists(tgs)
    p = tgl.minimum_duration_path(tg, 0, 2, tgs.getTimeInterval())
    print(p)



def temporal_earliest_arrival_path_test():
    """
    Test for computing the temporal distances.

    >>> temporal_earliest_arrival_path_test()
    VectorTemporalEdge[(0 2 1 5)]
    """
    tgs = tgl.load_ordered_edge_list("../example_datasets/example_from_paper.tg")
    tg = tgl.to_incident_lists(tgs)
    p = tgl.earliest_arrival_path(tg, 0, 2, tgs.getTimeInterval())
    print(p)



def temporal_shortest_path_test():
    """
    >>> temporal_shortest_path_test()
    VectorTemporalEdge[(0 1 2 1), (1 2 7 2)]
    """
    tgs = tgl.load_ordered_edge_list("../example_datasets/example_from_paper.tg")
    tg = tgl.to_incident_lists(tgs)
    p = tgl.minimum_transition_time_path(tg, 0, 2, tgs.getTimeInterval())
    print(p)


if __name__ == "__main__":
    doctest.testmod()
