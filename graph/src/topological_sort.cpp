#include <graph.h>
#include <algorithm>

namespace algorithm {
namespace graph {

namespace {
    const int kUnmarked = 0;
    const int kTemporaryMark = 1;
    const int kPermanentMark = 2;
}

    ErrorCode TopologicalSortDfs(const Graph& graph, NodeIndex node,
                                 std::vector<NodeIndex>& sorted_nodes, std::vector<int>& marks) {
        if (marks[node] == kPermanentMark) {
            return kSuccess;
        } else if (marks[node] == kTemporaryMark) {
            return kGraphNotDag;
        }

        marks[node] = kTemporaryMark;
        const auto& edges = graph.edges(node);
        for (const auto& edge : edges) {
            ErrorCode error_code = TopologicalSortDfs(graph, edge.destination(), sorted_nodes, marks);
            if (error_code != kSuccess) {
                return error_code;
            }
        }
        marks[node] = kPermanentMark;
        sorted_nodes.push_back(node);
        return kSuccess;
    }

    std::pair<std::vector<NodeIndex>, ErrorCode> TopologicalSort(const Graph& graph) {
        const auto& nodes = graph.nodes();
        std::vector<int> marks(graph.capacity(), 0);
        std::vector<NodeIndex> sorted_nodes;
        sorted_nodes.reserve(nodes.size());

        for (NodeIndex node : nodes) {
            if (marks[node] == kUnmarked) {
                ErrorCode error_code = TopologicalSortDfs(graph, node, sorted_nodes, marks);
                if (error_code != kSuccess) {
                    return std::make_pair(std::vector<NodeIndex>(), error_code);
                }
            }
        }
        std::reverse(sorted_nodes.begin(), sorted_nodes.end());
        return std::make_pair(sorted_nodes, kSuccess);
    }

}
}