import pytglib as tgl
import doctest


def temporal_katz_test():
    """
    >>> temporal_katz_test()
    VectorDouble[0, 3, 5, 8]
    """
    tgs = tgl.load_ordered_edge_list("../example_datasets/example_from_paper.tg")
    c = tgl.temporal_katz_centrality(tgs, 1)
    return c


if __name__ == "__main__":
    doctest.testmod()
