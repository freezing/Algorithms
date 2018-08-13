#include <graph.h>
#include <gtest/gtest.h>
#include <gmock/gmock.h>

using namespace algorithm::graph;

TEST(StronglyConnectedComponents, ManySccs) {
    const int capacity = 15;
    Graph graph(capacity);

    // Create a graph with the following SCCs:
    // 0 -> 1 -> 2 -> 0 (SCC)
    // 3 -> 4 -> 3 (SCC)
    // 5 -> 6 -> 5 (SCC)
    // 7 -> 7 (SCC)
    // 3 -> 1 (Cross-Edge)
    // 5 -> 2 (Cross-Edge)
    // 7 -> 4 (Cross-Edge)
    // 8 (SCC, lonely node with no edges)

    for (NodeIndex node = 0; node <= 8; node++) {
        graph.addNode(node);
    }

    // 0 -> 1 -> 2 -> 0 (SCC)
    graph.addEdge(0, 1);
    graph.addEdge(1, 2);
    graph.addEdge(2, 0);

    // 3 -> 4 -> 3 (SCC)
    graph.addEdge(3, 4);
    graph.addEdge(4, 3);

    // 5 -> 6 -> 5 (SCC)
    graph.addEdge(5, 6);
    graph.addEdge(5, 5);

    // 7 -> 7 (SCC)
    graph.addEdge(7, 7);

    // Cross-Edges.
    graph.addEdge(3, 1);
    graph.addEdge(5, 2);
    graph.addEdge(7, 4);

    std::vector<StronglyConnectedComponent> sccs = Tarjan(graph);
    ASSERT_EQ(sccs.size(), 5);
    ASSERT_THAT(sccs, testing::Contains(testing::UnorderedElementsAre(0, 1, 2)));
    ASSERT_THAT(sccs, testing::Contains(testing::UnorderedElementsAre(3, 4)));
    ASSERT_THAT(sccs, testing::Contains(testing::UnorderedElementsAre(5, 6)));
    ASSERT_THAT(sccs, testing::Contains(testing::UnorderedElementsAre(7)));
    ASSERT_THAT(sccs, testing::Contains(testing::UnorderedElementsAre(8)));
}