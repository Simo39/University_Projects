import pytglib as tgl
import doctest


def temporal_diameter_test():
    """
    >>> temporal_diameter_test()
    2
    """
    tgs = tgl.load_ordered_edge_list("../example_datasets/example_from_paper.tg")
    c = tgl.temporal_diameter(tgs, tgs.getTimeInterval(), tgl.Distance_Type.Minimum_Hops)
    return c


def temporal_eccentricity_test():
    """
    >>> temporal_eccentricity_test()
    [2, 1]
    """
    tgs = tgl.load_ordered_edge_list("../example_datasets/example_from_paper.tg")
    c0 = tgl.temporal_eccentricity(tgs, 0, tgs.getTimeInterval(), tgl.Distance_Type.Minimum_Hops)
    c1 = tgl.temporal_eccentricity(tgs, 1, tgs.getTimeInterval(), tgl.Distance_Type.Minimum_Hops)
    return [c0, c1]


if __name__ == "__main__":
    doctest.testmod()
