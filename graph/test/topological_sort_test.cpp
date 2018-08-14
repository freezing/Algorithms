#include <graph.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <iterator>

using namespace algorithm::graph;

void ASSERT_ORDER(const std::vector<NodeIndex>& actual, NodeIndex u, NodeIndex v) {
    ptrdiff_t u_position = std::distance(actual.begin(), find(actual.begin(), actual.end(), u));
    ptrdiff_t v_position = std::distance(actual.begin(), find(actual.begin(), actual.end(), v));
    ASSERT_LT(u_position, v_position);
}

TEST(TopologicalSort, Sanity) {
    // Create a graph as follows (edges go from top to bottom only).
    //   0    1   2
    //   |  /   /
    //   3    4
    //   |    |
    //   5    6    7
    // And additionally 3 -> 7 (can't fit in the ASCII art).
    const uint32_t capacity = 8;
    Graph graph(capacity);
    graph.addEdge(0, 3);
    graph.addEdge(1, 3);
    graph.addEdge(2, 4);
    graph.addEdge(3, 5);
    graph.addEdge(3, 7);
    graph.addEdge(4, 6);

    std::pair<std::vector<NodeIndex>, ErrorCode> result = TopologicalSort(graph);
    ASSERT_EQ(result.second, kSuccess);

    const auto& actual = result.first;
    ASSERT_EQ(actual.size(), graph.nodes().size());

    // Test the property, i.e. for each edge (u, v), v comes after u in the result.
    ASSERT_ORDER(actual, 0, 3);
    ASSERT_ORDER(actual, 1, 3);
    ASSERT_ORDER(actual, 2, 4);
    ASSERT_ORDER(actual, 3, 5);
    ASSERT_ORDER(actual, 3, 7);
    ASSERT_ORDER(actual, 4, 6);
}

TEST(TopologicalSort, DagFails) {
    const uint32_t capacity = 3;
    Graph graph(capacity);
    graph.addEdge(0, 1);
    graph.addEdge(1, 2);
    graph.addEdge(2, 0);

    std::pair<std::vector<NodeIndex>, ErrorCode> result = TopologicalSort(graph);
    ASSERT_EQ(result.second, kGraphNotDag);
}