#include <graph.h>
#include <gtest/gtest.h>
#include <gmock/gmock.h>

using namespace algorithm::graph;

TEST(Bfs, FullyConnectedGraph) {
    const int capacity = 3;
    Graph graph(capacity);

    // Create undirected graph as follows:
    // 0 - 2 - 1
    graph.addNode(0);
    graph.addNode(1);
    graph.addNode(2);

    graph.addUndirectedEdge(0, 2);
    graph.addUndirectedEdge(2, 1);

    // Bind a function to be reused in this test.
    // It adds visited nodes in the visited_nodes vector.
    std::vector<NodeIndex> visited_nodes;
    auto bfs = std::bind(Bfs, std::ref(graph), std::placeholders::_1, [&visited_nodes](NodeIndex u) {
        visited_nodes.push_back(u);
        return true;
    });

    // Test with 0 as starting node.
    visited_nodes.clear();
    bfs(0);
    ASSERT_THAT(visited_nodes, testing::ElementsAre(0, 2, 1));

    // Test with 1 as starting node.
    visited_nodes.clear();
    bfs(1);
    ASSERT_THAT(visited_nodes, testing::ElementsAre(1, 2, 0));

    // Test with 2 as starting node.
    visited_nodes.clear();
    bfs(2);
    // Åssert that first node is 2 and all visited nodes are 0, 1, 2.
    ASSERT_EQ(visited_nodes[0], 2);
    ASSERT_THAT(visited_nodes, testing::UnorderedElementsAre(0, 1, 2));
}