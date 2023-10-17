import pytglib as tgl
import doctest


def temporal_walk_centrality_test():
    """
    >>> temporal_walk_centrality_test()
    VectorDouble[0, 6, 6, 0]
    """
    tgs = tgl.load_ordered_edge_list("../example_datasets/example_from_paper.tg")
    c = tgl.temporal_walk_centrality(tgs, 1, 1)
    return c


if __name__ == "__main__":
    doctest.testmod()
