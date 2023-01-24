#ifndef SMS_PARTITION_SOLVER_HPP
#define SMS_PARTITION_SOLVER_HPP

#include <vector>
#include "networkit/graph/Graph.hpp"
#include "sms/auxiliary/biconnected_partition.hpp"
#include "sms/auxiliary/graphs.hpp"


class PartitionSolver {
public:
    NetworKit::Graph *graph;

    PartitionSolver(NetworKit::Graph *g) {
        graph = g;
        value = 0;
        cut = std::vector<bool>(g->numberOfNodes(), false);
        has_run = false;
    };

    void solve();

    double getSolutionValue() const;

    std::vector<bool> getCutAssignment();

    bool hasRun() const;

private:
    std::vector<bool> cut;
    double value;
    bool has_run = false;
};


#endif //SMS_PARTITION_SOLVER_HPP
