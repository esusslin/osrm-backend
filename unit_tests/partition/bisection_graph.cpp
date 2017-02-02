#include "partition/bisection_graph.hpp"
#include "util/coordinate.hpp"
#include "util/typedefs.hpp"

#include <vector>

#include <boost/test/test_case_template.hpp>
#include <boost/test/unit_test.hpp>

using namespace osrm::partition;
using namespace osrm::util;

struct EdgeWithSomeAdditionalData
{
    NodeID source;
    NodeID target;
    unsigned important_data;
};

std::vector<Coordinate>
makeGridCoordinates(int rows, int columns, double step_size, double lon_base, double lat_base)
{
    std::vector<Coordinate> result;

    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < columns; ++c)
            result.push_back({FloatLongitude{lon_base + c * step_size},
                              FloatLatitude{lat_base + r * step_size}});

    return result;
}

std::vector<EdgeWithSomeAdditionalData> makeGridEdges(int rows, int columns, int id_base)
{
    const int min_id = id_base;
    const int max_id = id_base + rows * columns;
    const auto get_id = [id_base, columns](int r, int c) { return id_base + r * columns + c; };
    const auto valid = [min_id, max_id](int id) { return id >= min_id && id < max_id; };

    std::vector<EdgeWithSomeAdditionalData> edges;

    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < columns; ++c)
        {
            auto id = static_cast<NodeID>(get_id(r, c));
            auto left = get_id(r, c - 1);
            if (valid(left))
                edges.push_back({id, static_cast<NodeID>(left), 1});
            auto right = get_id(r, c + 1);
            if (valid(right))
                edges.push_back({id, static_cast<NodeID>(right), 1});
            auto top = get_id(r - 1, c);
            if (valid(top))
                edges.push_back({id, static_cast<NodeID>(top), 1});
            auto bottom = get_id(r + 1, c);
            if (valid(bottom))
                edges.push_back({id, static_cast<NodeID>(bottom), 1});
        }
    }

    return edges;
}

BOOST_AUTO_TEST_SUITE(bisection_graph)

BOOST_AUTO_TEST_CASE(access_nodes)
{
    // 40 entries of left/right edges
    double step_size = 0.01;
    int rows = 10;
    int cols = 4;
    const auto coordinates = makeGridCoordinates(rows, cols, step_size, 0, 0);

    std::vector<EdgeWithSomeAdditionalData> grid_edges;
    auto graph = makeBisectionGraph(coordinates, adaptToBisectionEdge(std::move(grid_edges)));

    const auto to_row = [cols](const NodeID nid) { return nid / cols; };

    const auto to_col = [cols](const NodeID nid) { return nid % cols; };

    const auto get_expected = [&](const NodeID id) {
        const auto expected_lon = FloatLongitude{to_col(id) * step_size};
        const auto expected_lat = FloatLatitude{to_row(id) * step_size};
        Coordinate compare(expected_lon, expected_lat);
        return compare;
    };

    // check const access
    BOOST_CHECK_EQUAL(graph.NumberOfNodes(), 40);
    {
        NodeID increasing = 0;
        for (const auto &node : graph.Nodes())
        {
            const auto id = graph.GetID(node);
            BOOST_CHECK_EQUAL(id, increasing++);
            BOOST_CHECK_EQUAL(node.coordinate, get_expected(id));
        }
    }
    // non-const access
    {
        NodeID increasing = 0;
        for (auto &node : graph.Nodes())
        {
            const auto id = graph.GetID(node);
            BOOST_CHECK_EQUAL(id, increasing++);
            BOOST_CHECK_EQUAL(node.coordinate, get_expected(id));
        }
    }
    // iterator access
    {
        const auto begin = graph.Begin();
        const auto end = graph.End();
        NodeID increasing = 0;
        for (auto itr = begin; itr != end; ++itr)
        {
            const auto id = static_cast<NodeID>(std::distance(begin, itr));
            BOOST_CHECK_EQUAL(id, increasing++);
            BOOST_CHECK_EQUAL(itr->coordinate, get_expected(id));
        }
    }
    // const iterator access
    {
        const auto begin = graph.CBegin();
        const auto end = graph.CEnd();
        NodeID increasing = 0;
        for (auto itr = begin; itr != end; ++itr)
        {
            const auto id = static_cast<NodeID>(std::distance(begin, itr));
            BOOST_CHECK_EQUAL(id, increasing++);
            BOOST_CHECK_EQUAL(itr->coordinate, get_expected(id));
        }
    }
}

BOOST_AUTO_TEST_CASE(access_edges)
{
    // 40 entries of left/right edges
    const auto left_box = makeGridCoordinates(10, 4, 0.01, 0, 0);
    const auto right_box = makeGridCoordinates(10, 4, 0.01, 10, 0);
    std::vector<Coordinate> coordinates = left_box;
    coordinates.insert(coordinates.end(), right_box.begin(), right_box.end());

    const auto left_edges = makeGridEdges(10, 4, 0);
    const auto right_edges = makeGridEdges(10, 4, 40);
    std::vector<EdgeWithSomeAdditionalData> grid_edges = left_edges;
    grid_edges.insert(grid_edges.end(), right_edges.begin(), right_edges.end());
    groupEdgesBySource(grid_edges.begin(), grid_edges.end());

    const auto graph = makeBisectionGraph(coordinates, adaptToBisectionEdge(std::move(grid_edges)));

    BOOST_CHECK_EQUAL(graph.NumberOfNodes(), 80);
}

BOOST_AUTO_TEST_SUITE_END()
