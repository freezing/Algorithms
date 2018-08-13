#include <graph.h>
#include <gtest/gtest.h>
#include <gmock/gmock.h>

using namespace algorithm::graph;

TEST(Dijkstra, DistancesAreCorrect) {
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

    std::vector<Weight> distances = Dijkstra(graph, 0);
    ASSERT_THAT(distances, testing::ElementsAre(0, 4, 12, 19, 21, 11, 9, 8, 14));
}

TEST(Dijkstra, InifiniteDistanceForNodeWithoutPath) {
    const int capacity = 2;
    Graph graph(capacity);

    // Build a graph.
    graph.addUndirectedEdge(0, 0, 10);

    std::vector<Weight> distances = Dijkstra(graph, 0);
    ASSERT_THAT(distances, testing::ElementsAre(0, algorithm::graph::kInfinity));
}