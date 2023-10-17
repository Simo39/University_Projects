import pytglib as tgl
import doctest


def temporal_edge_betweenness_test():
    """
    >>> temporal_edge_betweenness_test()
    VectorDouble[0, 0, 0, 0, 0, 0, 0]
    """
    tgs = tgl.load_ordered_edge_list("../example_datasets/example_from_paper.tg")
    c = tgl.temporal_edge_betweenness(tgs)
    return c


if __name__ == "__main__":
    doctest.testmod()
