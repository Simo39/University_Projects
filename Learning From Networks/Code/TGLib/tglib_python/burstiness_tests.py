import pytglib as tgl
import doctest


def edge_burstiness_test():
    """
    >>> edge_burstiness_test()
    (0, 1) -1.0
    (2, 3) -1.0
    """
    tgs = tgl.load_ordered_edge_list("../example_datasets/example_from_paper.tg")
    c = tgl.edge_burstiness(tgs, tgs.getTimeInterval())
    for e in c:
        print(e, c[e])


def node_burstiness_test():
    """
    >>> node_burstiness_test()
    VectorDouble[-0.333333, 0, -1, 0]
    """
    tgs = tgl.load_ordered_edge_list("../example_datasets/example_from_paper.tg")
    c = tgl.node_burstiness(tgs, tgs.getTimeInterval())
    return c


if __name__ == "__main__":
    doctest.testmod()
