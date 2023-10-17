import pytglib as tgl
import doctest


def temporal_closeness_test():
    """
    Test for computing the temporal closeness.

    >>> temporal_closeness_test()
    VectorDouble[1.39286, 0.5, 0.5, 1.33333]
    """
    tgs = tgl.load_ordered_edge_list("../example_datasets/example_from_paper.tg")
    c = tgl.temporal_closeness(tgs, tgl.Distance_Type.Fastest)
    return c


if __name__ == "__main__":
    doctest.testmod()
