from shapely.geometry import Polygon, Point, LineString



def merge_close_points(path):
    eps = 0.1
    orig_path = path
    distances = [orig_path[i].distance(orig_path[i+1]) for i in range(len(orig_path)-1)]
    rows_to_delete = [i+1 for i, x in enumerate(distances) if distances[i]<eps]
    print rows_to_delete[-1], len(orig_path)-1
    if rows_to_delete[-1] == (len(orig_path)-1):
        rows_to_delete[-1] = len(orig_path) - 2
    final_path = [x for i,x in enumerate(orig_path) if i not in rows_to_delete]
    for i in range(len(final_path)):    
        print final_path[i].xy
    return final_path     
    # less_than = [distances[i]<eps for i in range(len(distances))]
    # print "distances", distances
    # print "rtd", rtd
    # # raw_input()
    # print "original path"
    # for i in range(len(orig_path)):    
    #     print orig_path[i].xy
    # raw_input()
    # print len(orig_path)
    # rows_to_delete = []
    # for i in range(1, len(orig_path)-2):
    #     print [orig_path[idx].xy for idx in range(len(orig_path)-1)]
    #     print "\npoints distance", orig_path[i].distance(orig_path[i+1]), "\n"
    #     if orig_path[i].distance(orig_path[i+1])<eps or orig_path[i].distance(orig_path[i-1])<eps:
    #         print "close points", orig_path[i].xy, orig_path[i+1].xy, "\n"
    #         rows_to_delete.append(i)
    # print "rows_to_delete", rows_to_delete


def main():
    path = [Point(0.0,      0.0),
            Point(0.01,     0.0),
            Point(0.3,      0.3),
            Point(0.301,    0.3),
            Point(0.6,      0.6),
            Point(0.999,    1.0),
            Point(1.0,      1.0)]

    print merge_close_points(path)

if __name__ == "__main__":
    main()
