#include <vector>

#include "networkit/graph/Graph.hpp"
#include "networkit/graph/KruskalMSF.hpp"
#include "networkit/graph/RandomMaximumSpanningForest.hpp"
#include "networkit/distance/BFS.hpp"

#include "sms/auxiliary/mst_heuristic.hpp"
#include "sms/auxiliary/odd_closed_walk.hpp"

using NetworKit::node;
using NetworKit::edgeweight;
using NetworKit::Edge;
using NetworKit::Graph;


void MSTHeuristic::updateWeights(node u, node v, edgeweight w) {
    assert(originalGraph_->hasEdge(u, v));
    edgeweight fixedVal = std::min(std::max(0., w), 1.);

    lpWeightedGraph_.setWeight(u, v, fixedVal);
    lpShiftedGraph_.setWeight(u, v, std::abs(fixedVal - 0.5)); // (use negative weight such that MST finds maximum)
}

void MSTHeuristic::computeSpanningTree() {
    //NetworKit::KruskalMSF mstAlgo(lpShiftedGraph_);
    NetworKit::RandomMaximumSpanningForest mstAlgo(lpShiftedGraph_);

    mstAlgo.run();

    //tree_ = mstAlgo.getForest();
    tree_ = mstAlgo.getMSF();
    assert(tree_.numberOfNodes() == lpShiftedGraph_.numberOfNodes());
//    assert(tree_.numberOfEdges() == (lpShiftedGraph_.numberOfNodes() - 1));
}

std::vector<OddClosedWalk> MSTHeuristic::getViolatedOCs() {
    std::vector<OddClosedWalk> ocws;

    for (auto e: lpWeightedGraph_.edgeWeightRange()) {
        if (!tree_.hasEdge(e.u, e.v)) {
            // BFS for OCW
            NetworKit::BFS bfs(tree_, e.u, true, false, e.v);
            bfs.run();
            auto oc_candidate = bfs.getPath(e.v);

            //compute weight vector
            std::vector<edgeweight> weights;

            for (uint i = 0; i < oc_candidate.size() - 1; i++) {
                weights.push_back(lpWeightedGraph_.weight(oc_candidate[i], oc_candidate[i + 1]));
            }

            weights.push_back(lpWeightedGraph_.weight(oc_candidate[oc_candidate.size() - 1], oc_candidate[0]));

            auto ocw = violatedOddSelectionClosedWalk(oc_candidate, weights);

            if (ocw.has_value())
                ocws.push_back(ocw.value());
        }
    }

    return ocws;
}

McSolution MSTHeuristic::getPrimalSolution(node s) {
    McSolution solution(originalGraph_);

    node current = s;

    std::vector<int8_t> visited(tree_.numberOfNodes(), -1);
    std::queue<node> queue;
    solution.toPartition0(current);
    visited[current] = 0;

    queue.push(current);

    while (!queue.empty()) {
        current = queue.front();
        queue.pop();
        int8_t currentAssignment = visited[current];

        for (auto v: tree_.neighborRange(current)) {
            if (visited[v] == -1) {
                queue.push(v);
                if (lpWeightedGraph_.weight(current, v) > 0.5) {
                    if (currentAssignment == 1) {
                        solution.toPartition0(v);
                        visited[v] = 0;
                    } else if (currentAssignment == 0) {
                        solution.toPartition1(v);
                        visited[v] = 1;
                    }
                } else {
                    if (currentAssignment == 1) {
                        solution.toPartition1(v);
                        visited[v] = 1;
                    } else if (currentAssignment == 0) {
                        solution.toPartition0(v);
                        visited[v] = 0;
                    }
                }
            }
        }
    }

//    assert(solution.isValid());
    return solution;
}
