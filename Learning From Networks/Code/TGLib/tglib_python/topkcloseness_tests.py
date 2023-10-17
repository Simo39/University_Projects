import pytglib as tgl
import doctest


def topk_closeness_test():
    """
    >>> topk_closeness_test()
    0
    1.3928571428571428
    3
    1.3333333333333333
    """
    tgs = tgl.load_ordered_edge_list("../example_datasets/example_from_paper.tg")
    tg = tgl.to_incident_lists(tgs)
    c = tgl.compute_topk_closeness(tg, 2, tg.getTimeInterval(), tgl.Distance_Type.Fastest)
    for p in c:
        for n in p:
            print(n)


if __name__ == "__main__":
    doctest.testmod()
