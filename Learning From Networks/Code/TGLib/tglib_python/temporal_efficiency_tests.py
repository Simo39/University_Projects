import pytglib as tgl
import doctest


def temporal_efficiency_test():
    """
    >>> temporal_efficiency_test()
    0.310515873015873
    """
    tgs = tgl.load_ordered_edge_list("../example_datasets/example_from_paper.tg")
    c = tgl.temporal_efficiency(tgs, tgs.getTimeInterval(), tgl.Distance_Type.Fastest)
    return c


if __name__ == "__main__":
    doctest.testmod()
