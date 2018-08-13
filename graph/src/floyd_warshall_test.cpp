#include <graph.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

using namespace algorithm::graph;

TEST(FloydWarshall, DistancesAreCorrect) {
    const int capacity = 9;
    Graph graph(capacity);

    // Build a graph.
    graph.addUndirectedEdge(0, 1, 4);
    graph.addUndirectedEdge(1, 2, 8);
    graph.addUndirectedEdge(2, 3, 7);
    graph.addUndirectedEdge(3, 4, 9);
    graph.addUndirectedEdge(4, 5, 10);
    graph.addUndirectedEdge(5, 6, 2);
    graph.addUndirectedEdge(6, 7, 1);
    graph.addUndirectedEdge(7, 8, 7);
    graph.addUndirectedEdge(7, 0, 8);
    graph.addUndirectedEdge(8, 6, 6);
    graph.addUndirectedEdge(8, 2, 2);
    graph.addUndirectedEdge(2, 5, 4);
    graph.addUndirectedEdge(3, 5, 14);
    graph.addUndirectedEdge(7, 1, 11);

    std::vector<std::vector<Weight>> expected(graph.capacity(), std::vector<Weight>(graph.capacity(), kInfinity));
    expected[0] = { 0, 4, 12, 19, 21, 11, 9, 8, 14 };
    expected[1] = { 4, 0, 8, 15, 22, 12, 12, 11, 10 };
    expected[2] = { 12, 8, 0, 7, 14, 4, 6, 7, 2 };
    expected[3] = { 19, 15, 7, 0, 9, 11, 13, 14, 9 };
    expected[4] = { 21, 22, 14, 9, 0, 10, 12, 13, 16 };
    expected[5] = { 11, 12, 4, 11, 10, 0, 2, 3, 6 };
    expected[6] = { 9, 12, 6, 13, 12, 2, 0, 1, 6 };
    expected[7] = { 8, 11, 7, 14, 13, 3, 1, 0, 7 };
    expected[8] = { 14, 10, 2, 9, 16, 6, 6, 7, 0 };

    auto distances = FloydWarshall(graph);
    ASSERT_EQ(distances, expected);
}

TEST(FloydWarshall, InifiniteDistanceForNodeWithoutPath) {
    const int capacity = 2;
    Graph graph(capacity);

    // Build a graph.
    graph.addUndirectedEdge(0, 0, 10);

    std::vector<std::vector<Weight>> expected(graph.capacity(), std::vector<Weight>(graph.capacity(), kInfinity));
    expected[0][0] = 0;
    // expected[1][1] is kInfinity because the node 1 doesn't exist.

    auto distances = FloydWarshall(graph);
    ASSERT_EQ(distances, expected);
}