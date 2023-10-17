import pytglib as tgl
import doctest


def temporal_reachability_test():
    """
    Test for computing the temporal distances.

    >>> temporal_reachability_test()
    4
    2
    2
    3
    """
    tgs = tgl.load_ordered_edge_list("../example_datasets/example_from_paper.tg")
    p = tgl.number_of_reachable_nodes(tgs, 0, tgs.getTimeInterval())
    print(p)
    p = tgl.number_of_reachable_nodes(tgs, 1, tgs.getTimeInterval())
    print(p)
    p = tgl.number_of_reachable_nodes(tgs, 2, tgs.getTimeInterval())
    print(p)
    p = tgl.number_of_reachable_nodes(tgs, 3, tgs.getTimeInterval())
    print(p)
