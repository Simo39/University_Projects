import pytglib as tgl
import doctest


def topological_overlap_test():
    """
    >>> topological_overlap_test()
    0.5
    """
    tgs = tgl.load_ordered_edge_list("../example_datasets/example_from_paper.tg")
    tg = tgl.to_incident_lists(tgs)
    c = tgl.topological_overlap(tg, 0, tg.getTimeInterval())
    return c


if __name__ == "__main__":
    doctest.testmod()
