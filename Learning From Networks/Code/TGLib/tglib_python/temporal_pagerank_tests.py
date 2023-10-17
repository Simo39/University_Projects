import pytglib as tgl
import doctest


def temporal_pagerank_test():
    """
    >>> temporal_pagerank_test()
    VectorDouble[0, 0, 0, 0]
    """
    tgs = tgl.load_ordered_edge_list("../example_datasets/example_from_paper.tg")
    c = tgl.temporal_pagerank(tgs, 1, 1, 1)
    return c


if __name__ == "__main__":
    doctest.testmod()
