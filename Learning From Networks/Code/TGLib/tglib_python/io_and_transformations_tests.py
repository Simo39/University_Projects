import pytglib as tgl
import doctest


def load_example_test():
    """
    Test for loading an ordered edge stream.

    >>> load_example_test()
    4 7
    (0 2 1 5)
    0 3
    """
    tgs = tgl.load_ordered_edge_list("../example_datasets/example_from_paper.tg")
    print(str(tgs.getNumberOfNodes()), str(tgs.getNumberOfEdges()))
    print(tgs.getEdges()[0])
    u = tgs.getReverseNodeMap()[0]
    v = tgs.getReverseNodeMap()[2]
    print(u, v)


def normalize_test():
    """
    Test for loading and normalizing an ordered edge stream.

    >>> normalize_test()
    4 7
    (0 2 1 5)
    0 3
    """
    tgs = tgl.load_ordered_edge_list("../example_datasets/example_from_paper_with_loops_and_multiedges.tg")
    normalized = tgl.normalize(tgs, True)
    print(str(normalized.getNumberOfNodes()), str(normalized.getNumberOfEdges()))
    print(normalized.getEdges()[0])
    u = normalized.getReverseNodeMap()[0]
    v = normalized.getReverseNodeMap()[2]
    print(u, v)


def scale_timestamps_test():
    """
    Test for loading an ordered edge stream and scaling the time stamps.

    >>> scale_timestamps_test()
    4 7
    (0 2 10 5)
    0 3
    """
    tgs = tgl.load_ordered_edge_list("../example_datasets/example_from_paper.tg")
    scaled = tgl.scale_timestamps(tgs, 10)
    print(str(scaled.getNumberOfNodes()), str(scaled.getNumberOfEdges()))
    print(scaled.getEdges()[0])
    u = scaled.getReverseNodeMap()[0]
    v = scaled.getReverseNodeMap()[2]
    print(u, v)


def unit_transition_times_test():
    """
    Test for loading an ordered edge stream and setting all transition times to one.

    >>> unit_transition_times_test()
    4 7
    (0 2 1 1)
    0 3
    """
    tgs = tgl.load_ordered_edge_list("../example_datasets/example_from_paper.tg")
    transformed = tgl.unit_transition_times(tgs, 1)
    print(str(transformed.getNumberOfNodes()), str(transformed.getNumberOfEdges()))
    print(transformed.getEdges()[0])
    u = transformed.getReverseNodeMap()[0]
    v = transformed.getReverseNodeMap()[2]
    print(u, v)


def to_incident_lists():
    """
    Test for transforming ordered edge stream to incident lists representation.

    >>> to_incident_lists()
    4 7
    """
    tgs = tgl.load_ordered_edge_list("../example_datasets/example_from_paper.tg")
    tg = tgl.to_incident_lists(tgs)
    print(str(tg.getNumberOfNodes()), str(tg.getNumberOfEdges()))
    # todo further tests


if __name__ == "__main__":
    doctest.testmod()
