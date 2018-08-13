#include <benchmark/benchmark.h>

#include <iostream>
#include <graph.h>

using namespace algorithm::graph;

static void BFS(benchmark::State &state) {
    const uint32_t capacity = state.range(0);
    Graph graph(capacity);
    for (NodeIndex u = 0; u < capacity - 1; u++) {
        graph.addNode(u);
        graph.addNode(u + 1);
        graph.addUndirectedEdge(u, u + 1);
    }

    for (auto _ : state) {
        Bfs(graph, 0, [](NodeIndex n) { return true; });
    }
    state.SetComplexityN(state.range(0));
    state.SetItemsProcessed(state.iterations());
}

BENCHMARK(BFS)
        ->Repetitions(4)
        ->Range(1 << 4, 1 << 20)
        ->RangeMultiplier(2)
        ->Unit(benchmark::kNanosecond)
        ->ReportAggregatesOnly(true)
        ->Complexity(benchmark::oN);

BENCHMARK_MAIN();