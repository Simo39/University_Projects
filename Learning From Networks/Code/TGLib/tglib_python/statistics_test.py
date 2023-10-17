import pytglib as tgl
import doctest


def get_statistics_test():
    """
    Test for computing the temporal closeness.

    >>> get_statistics_test()
    number of nodes: 4
    number of edges: 7
    number of static edges: 5
    number of time stamps: 6
    number of transition times: 4
    min. time stamp: 1
    max. time stamp: 8
    min. transition time: 1
    max. transition time: 5
    min. temporal in-degree: 0
    max. temporal in-degree: 3
    min. temporal out-degree: 1
    max. temporal out-degree: 3
    """
    tgs = tgl.load_ordered_edge_list("../example_datasets/example_from_paper.tg")
    s = tgl.get_statistics(tgs)
    print(s)


if __name__ == "__main__":
    doctest.testmod()
