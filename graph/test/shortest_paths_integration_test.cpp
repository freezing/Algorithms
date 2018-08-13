#include <graph.h>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <random>

using namespace algorithm::graph;

TEST(ShortestPathsIntegrationTest, DijkstraAndFloydWarshallMediumSize) {
    const int kSeed = 1439714;
    const uint32_t kCapacity = 100;
    const int kNumEdges = 1000;
    const Weight kMaxWeight = 10000;

    std::default_random_engine generator(kSeed);
    std::uniform_int_distribution<NodeIndex> node_index_distribution(0, kCapacity - 1);
    std::uniform_int_distribution<Weight> weight_distribution(0, kMaxWeight);

    Graph graph(kCapacity);
    for (int i = 0; i < kNumEdges; i++) {
        NodeIndex source = node_index_distribution(generator);
        NodeIndex destination = node_index_distribution(generator);
        Weight weight = weight_distribution(generator);
        graph.addEdge(source, destination, weight);
    }

    std::vector<std::vector<Weight>> floyd_distances = FloydWarshall(graph);
    const auto& nodes = graph.nodes();
    for (NodeIndex initial_node : nodes) {
        std::vector<Weight> dijkstra_distances = Dijkstra(graph, initial_node);
        ASSERT_EQ(floyd_distances[initial_node], dijkstra_distances);
    }
}