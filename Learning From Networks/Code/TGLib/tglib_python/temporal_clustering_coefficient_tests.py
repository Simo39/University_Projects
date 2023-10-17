import pytglib as tgl
import doctest


def temporal_clustering_coefficient_test():
    """
    >>> temporal_clustering_coefficient_test()
    0.16666666666666666
    """
    tgs = tgl.load_ordered_edge_list("../example_datasets/example_from_paper.tg")
    tg = tgl.to_incident_lists(tgs)

    c = tgl.temporal_clustering_coefficient(tg, 0, tg.getTimeInterval())
    return c


if __name__ == "__main__":
    doctest.testmod()
