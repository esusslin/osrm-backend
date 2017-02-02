#include "partition/graph_view.hpp"
#include "partition/graph_generator.hpp"
#include "partition/recursive_bisection_state.hpp"

#include <algorithm>
#include <vector>

#include <boost/test/test_case_template.hpp>
#include <boost/test/unit_test.hpp>

using namespace osrm::partition;
using namespace osrm::util;

BOOST_AUTO_TEST_SUITE(bisection_graph)

BOOST_AUTO_TEST_CASE(separate_left_right)
{
    std::vector<bool> partition(8, false);
    partition[0] = partition[1] = partition[2] = partition[3] = true;

    // 40 entries of left/right edges
    double step_size = 0.01;
    int rows = 2;
    int cols = 4;
    const auto coordinates = makeGridCoordinates(rows, cols, step_size, 0, 0);

    auto grid_edges = makeGridEdges(rows, cols, 0);

    std::random_shuffle(grid_edges.begin(), grid_edges.end());
    groupEdgesBySource(grid_edges.begin(), grid_edges.end());

    const auto graph = makeBisectionGraph(coordinates, adaptToBisectionEdge(std::move(grid_edges)));

    const auto to_row = [cols](const NodeID nid) { return nid / cols; };
    const auto to_col = [cols](const NodeID nid) { return nid % cols; };

    RecursiveBisectionState bisection_state(graph);
}

BOOST_AUTO_TEST_SUITE_END()
