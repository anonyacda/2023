#include "sms/solver/basic_ocw.hpp"

#include <utility>

#include "sms/auxiliary/scip_exception.hpp"
#include "sms/auxiliary/scip.hpp"
#include "sms/auxiliary/small_ccs.hpp"
#include "sms/auxiliary/mc_solution.hpp"
#include "sms/branch/branchrule_degree.hpp"
#include "sms/branch/branchrule_nm.hpp"
#include "sms/branch/branchrule_pscost.hpp"
#include "sms/conshdlr/conshdlr_triangles.hpp"
#include "sms/conshdlr/conshdlr_oscw.hpp"
#include "sms/conshdlr/conshdlr_holes.hpp"
#include "sms/eventhdlr/eventhdlr_history.hpp"
#include "sms/eventhdlr/eventhdlr_root.hpp"
#include "sms/probdata/mc_rooted.hpp"
#include "sms/probdata/mc_edges_only.hpp"
#include "sms/auxiliary/graphs.hpp"


BasicOcwSolver::BasicOcwSolver(graph_ptr g, int seed, double timelimit, int nodelimit)
        : scip_(nullptr), g_(g), seed_(seed), timelimit_(timelimit), nodelimit_(nodelimit),
          paramFile_("default_params.set") {}

BasicOcwSolver::BasicOcwSolver(graph_ptr g, int seed, double timelimit, int nodelimit, std::string paramFile)
        : scip_(nullptr), g_(g), seed_(seed), timelimit_(timelimit), nodelimit_(nodelimit),
          paramFile_(std::move(paramFile)) {}

BasicOcwSolver::~BasicOcwSolver() {
    SCIP_CALL_EXC_NO_THROW(SCIPfree(&scip_))
#ifndef NDEBUG
    BMScheckEmptyMemory();
#endif
}

bool BasicOcwSolver::hasRun() const {
    return run_;
}

McSolution BasicOcwSolver::run(const std::string &stats,
                               const std::string &statsScip, const std::string &logScip,
                               const std::string &history,
                               bool nodeVars) {
    if (nodeVars) {
        return solveRooted(stats, statsScip, logScip, history);
    } else {
        return solveEdgesOnly(stats, statsScip, logScip, history);
    }
}

McSolution
BasicOcwSolver::solveEdgesOnly(const std::string &stats, const std::string &statsScip, const std::string &logScip,
                               const std::string &history) {

    std::cout << "Started solver with no node-vars" << std::endl;

    run_ = false;
    buildEdgesOnlyMcModel(&scip_, g_);

    if (!logScip.empty()) {
        SCIPsetMessagehdlrLogfile(scip_, logScip.c_str());
    }

    EventhdlrHistory *eventHandler = nullptr;
    if (!history.empty()) {
        eventHandler = new EventhdlrHistory(scip_);
        SCIP_CALL_EXC(SCIPincludeObjEventhdlr(scip_, eventHandler, TRUE))
    }

    if (!statsScip.empty()) {
        std::filesystem::path p(statsScip);
        auto extension = p.extension();
        p.replace_extension(".root" + extension.string());
        auto eventHandlerRoot = new EventhdlrRoot(scip_, p.string());
        SCIP_CALL_EXC(SCIPincludeObjEventhdlr(scip_, eventHandlerRoot, TRUE))
    }


    // Add odd cycle ocwHandler
    auto *ocwFastHandler = new ConshdlrOscw(scip_, g_);
    SCIP_CALL_EXC(SCIPincludeObjConshdlr(scip_, ocwFastHandler, TRUE));
    // Add a constraint for the odd cycle ocwHandler, to trigger callbacks to it
    SCIP_CONS *cons;
    SCIP_CALL_EXC(SCIPcreateConsBasicOddCyclesFast(scip_, &cons, "Odd Cycles", g_))
    SCIP_CALL_EXC(SCIPaddCons(scip_, cons))
    SCIP_CALL_EXC(SCIPreleaseCons(scip_, &cons))

    // Add triangle handler
    auto *triangleHandler = new ConshdlrTriangles(scip_, g_);
    SCIP_CALL_EXC(SCIPincludeObjConshdlr(scip_, triangleHandler, TRUE))

    // Add hole handler
    auto *holeHandler = new ConshdlrHoles(scip_, g_);
    SCIP_CALL_EXC(SCIPincludeObjConshdlr(scip_, holeHandler, TRUE))

    setScipParams();

    addWarmStartSolutionsEdgesOnlyToScip();

    // Run the solver
    SCIP_CALL_EXC(SCIPsolve(scip_));

    if (!stats.empty()) {
        nlohmann::json j;
        scipFinalStatsToJsonFile(scip_, stats, j);
    }

    if (!statsScip.empty()) {
        printScipStatistics(scip_, statsScip);
    }

    if (!history.empty() && eventHandler != nullptr) {
        auto eventHandlerLog = std::ofstream(history);
        eventHandler->to_json(eventHandlerLog);
        eventHandlerLog.close();
    }

    run_ = true;
    std::cout << "Solver finished." << std::endl;

    return getBestSolutionEdgesOnly();
}

McSolution
BasicOcwSolver::solveRooted(const std::string &stats, const std::string &statsScip, const std::string &logScip,
                            const std::string &history) {
    run_ = false;

    auto [maxDegNode, maxDeg] = maxDegreeNode(*g_);

    // Build model
    NetworKit::node sun = maxDegNode;
    std::cout << "Picked " << sun << " as sun node. Degree is " << maxDeg << std::endl;
    buildRootMcModel(&scip_, g_, sun, false);


    if (!logScip.empty()) {
        SCIPsetMessagehdlrLogfile(scip_, logScip.c_str());
    }

    EventhdlrHistory *eventHandler = nullptr;
    if (!history.empty()) {
        eventHandler = new EventhdlrHistory(scip_);
        SCIP_CALL_EXC(SCIPincludeObjEventhdlr(scip_, eventHandler, TRUE))
    }

    if (!statsScip.empty()) {
        std::filesystem::path p(statsScip);
        auto extension = p.extension();
        p.replace_extension(".root" + extension.string());
        auto eventHandlerRoot = new EventhdlrRoot(scip_, p.string());
        SCIP_CALL_EXC(SCIPincludeObjEventhdlr(scip_, eventHandlerRoot, TRUE))
    }


    // Add odd cycle ocwHandler
    auto *ocwFastHandler = new ConshdlrOscw(scip_, g_);
    SCIP_CALL_EXC(SCIPincludeObjConshdlr(scip_, ocwFastHandler, TRUE));
    // Add a constraint for the odd cycle ocwHandler, to trigger callbacks to it
    SCIP_CONS *cons;
    SCIP_CALL_EXC(SCIPcreateConsBasicOddCyclesFast(scip_, &cons, "Odd Cycles", g_))
    SCIP_CALL_EXC(SCIPaddCons(scip_, cons))
    SCIP_CALL_EXC(SCIPreleaseCons(scip_, &cons))

    // Add triangle handler
    auto *triangleHandler = new ConshdlrTriangles(scip_, g_);
    SCIP_CALL_EXC(SCIPincludeObjConshdlr(scip_, triangleHandler, TRUE))

    // Add hole handler
    auto *holeHandler = new ConshdlrHoles(scip_, g_);
    SCIP_CALL_EXC(SCIPincludeObjConshdlr(scip_, holeHandler, TRUE))

    //auto *oscHandler = new ConshdlrOddSubsetCycles(scip_);
    //SCIP_CALL_EXC(SCIPincludeObjConshdlr(scip_, oscHandler, TRUE))

    if (warmStartSolutions_.empty()) {
        std::cout << "No warmstart solution provided, this may take a while!" << std::endl;
    }

    //Add branching heuristic
    auto brachingDegreeBased = new BranchruleDegree(scip_, false);
    SCIP_CALL_EXC(SCIPincludeObjBranchrule(scip_, brachingDegreeBased, TRUE))
    //auto brachingPseudoCost = new BranchrulePscost(scip_);
    //SCIP_CALL_EXC(SCIPincludeObjBranchrule(scip_, brachingPseudoCost, TRUE))
    auto branchingNM = new BranchruleNM(scip_);
    SCIP_CALL_EXC(SCIPincludeObjBranchrule(scip_, branchingNM, TRUE))

    setScipParams();

    addWarmStartSolutionsToScip();

    // Run the solver
    SCIP_CALL_EXC(SCIPsolve(scip_));

    if (!stats.empty()) {
        nlohmann::json j;
        scipFinalStatsToJsonFile(scip_, stats, j);
    }

    if (!statsScip.empty()) {
        printScipStatistics(scip_, statsScip);
    }

    if (!history.empty() && eventHandler != nullptr) {
        auto eventHandlerLog = std::ofstream(history);
        eventHandler->to_json(eventHandlerLog);
        eventHandlerLog.close();
    }

    run_ = true;
    std::cout << "Solver finished." << std::endl;

    return getBestSolution();
}

void BasicOcwSolver::setScipParams() {
    SCIPreadParams(scip_, paramFile_.c_str());

    SCIPsetIntParam(scip_, "randomization/randomseedshift", seed_);
    SCIPsetRealParam(scip_, "limits/time", timelimit_);
    SCIPsetLongintParam(scip_, "limits/totalnodes", nodelimit_);

    SCIPsetIntParam(scip_, "display/freq", 1);
}


void BasicOcwSolver::setScipDefaultParams() {
    // Cut related
    turnOffAllCuts(scip_);
    SCIPsetIntParam(scip_, "separating/maxaddrounds", 2);
    //SCIPsetIntParam(scip_, "separating/maxstallrounds", -1);
    //SCIPsetIntParam(scip_, "separating/maxstallroundsroot", -1);
    SCIPsetIntParam(scip_, "separating/maxcuts", 10000);
    SCIPsetIntParam(scip_, "separating/maxcutsroot", 10000);
    SCIPsetIntParam(scip_, "separating/cutagelimit", 5); // default 80
    SCIPsetIntParam(scip_, "separating/poolfreq", 5); // default 10
    //SCIPsetRealParam(scip_, "separating/minefficacy", 0.0001); // default  0.0001
    //SCIPsetRealParam(scip_, "separating/minefficacyroot", 0.0001); // default  0.0001
    // Presolving
    SCIPsetIntParam(scip_, "presolving/maxrounds", 0);// turning off hurts performance
    SCIPsetIntParam(scip_, "presolving/maxrestarts", 0);
    //SCIPsetStringParam(scip_, "visual/bakfilename", "output.bak");
    //SCIPsetIntParam(scip_, "branching/pscost/priority", 20000);
    turnOffAllPrimalHeuristics(scip_);
    turnOffConflictAnalysis(scip_);
}


void BasicOcwSolver::addWarmStartSolution(const McSolution &solution) {
    warmStartSolutions_.push_back(solution);
}

void BasicOcwSolver::addWarmStartSolutionsEdgesOnlyToScip() {
    for (auto &solution: warmStartSolutions_) {
        auto *probData = SCIPgetObjProbData(scip_);
        auto probDataMc = dynamic_cast<ProbDataEdgesMc *>(probData);

        SCIP_SOL *newSol;
        SCIP_CALL_EXC(SCIPcreateSol(scip_, &newSol, nullptr));

        for (auto e: solution.getCutEdges()) {
            SCIP_Real edgeVarValue = 1; // all other edges are init to 0 through solution constructor
            SCIP_CALL_EXC(SCIPsetSolVal(scip_, newSol, probDataMc->edgeToVar(e.u, e.v), edgeVarValue));
        }

        assert(!SCIPsolIsPartial(newSol));

        SCIP_Bool added;
        SCIP_CALL_EXC(SCIPaddSolFree(scip_, &newSol, &added));
        assert(added);
    }
}

void BasicOcwSolver::addWarmStartSolutionsToScip() {
    for (auto &solution: warmStartSolutions_) {
        auto *probData = SCIPgetObjProbData(scip_);
        auto probDataSunMc = dynamic_cast<ProbDataRootedMc *>(probData);
        auto sun = probDataSunMc->getSun();

        if (solution.getPartition(sun) == 1) {
            solution.flipPartitioning();
        }

        SCIP_SOL *newSol;
        SCIP_CALL_EXC(SCIPcreateSol(scip_, &newSol, nullptr));

        for (auto u: solution.getPartition0()) {
            if (u != sun) {
                SCIP_VAR *var = probDataSunMc->nodeToVar(u);
                SCIP_CALL_EXC(SCIPsetSolVal(scip_, newSol, var, 0));
            }
        }
        for (auto u: solution.getPartition1()) {
            if (u != sun) {
                SCIP_VAR *var = probDataSunMc->nodeToVar(u);
                SCIP_CALL_EXC(SCIPsetSolVal(scip_, newSol, var, 1));
            }
        }

        for (auto e: solution.getCutEdges()) {
            SCIP_Real edgeVarValue = 1; // all other edges are init to 0 through solution constructor
            SCIP_CALL_EXC(SCIPsetSolVal(scip_, newSol, probDataSunMc->edgeToVar(e.u, e.v), edgeVarValue));
        }

        assert(!SCIPsolIsPartial(newSol));

        SCIP_Bool added;
        SCIP_CALL_EXC(SCIPaddSolFree(scip_, &newSol, &added));
        assert(added);
    }
}

McSolution BasicOcwSolver::getBestSolution() {
    assert(run_);
    auto bestSolution = McSolution(g_);

    auto *probData = SCIPgetObjProbData(scip_);
    auto probDataSunMc = dynamic_cast<ProbDataRootedMc *>(probData);

    node sun = probDataSunMc->getSun();
    bestSolution.toPartition0(sun);

    auto sol = SCIPgetBestSol(scip_);
    for (auto u: g_->nodeRange()) {
        if (u != sun) {
            if (SCIPisZero(scip_, SCIPgetSolVal(scip_, sol, probDataSunMc->nodeToVar(u)))) {
                bestSolution.toPartition0(u);
            } else {
                bestSolution.toPartition1(u);
            }
        }
    }

    return bestSolution;
}

McSolution BasicOcwSolver::getBestSolutionEdgesOnly() {
    assert(run_);
    auto sol = SCIPgetBestSol(scip_);
    auto bestSolution = McSolution(g_);

    auto *probData = SCIPgetObjProbData(scip_);
    auto probDataMc = dynamic_cast<ProbDataEdgesMc *>(probData);

    std::vector<node> stack;

    for (unsigned int j = 0; j < g_->upperNodeIdBound(); j++) {
        if (bestSolution.getPartition(j) == bestSolution.kUNASSIGNED) {
            stack.push_back(j);
            bestSolution.toPartition0(j);

            while (!stack.empty()) {
                auto currentNode = stack.back();
                stack.pop_back();

                for (auto curNeighbor: g_->neighborRange(currentNode)) {
                    auto edgeVar = probDataMc->edgeToVar(currentNode, curNeighbor);
                    assert(isBinary(scip_, edgeVar, sol));
                    auto edgeVal = SCIPgetSolVal(scip_, sol, edgeVar);
                    uint8_t binaryEdgeVal = edgeVal < 0.5 ? 0 : 1;

                    if (bestSolution.getPartition(curNeighbor) == bestSolution.kUNASSIGNED) {
                        if (binaryEdgeVal ^ (bestSolution.getPartition(j) == 1)) {
                            bestSolution.toPartition1(curNeighbor);
                        } else {
                            bestSolution.toPartition0(curNeighbor);
                        }
                        stack.push_back(curNeighbor);
                    }
                }
            }
        }
    }

    return bestSolution;
}
