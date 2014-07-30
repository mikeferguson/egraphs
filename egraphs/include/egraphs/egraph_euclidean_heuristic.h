#ifndef EGRAPH_EUCLIDEAN_HEURISTIC_H
#define EGRAPH_EUCLIDEAN_HEURISTIC_H

#include<egraphs/egraph_heuristic.h>
#include<egraphs/egraph.h>
#include<sbpl/headers.h>
#include<map>

class EGraphEuclideanHeuristic : public EGraphHeuristic<vector<double> >{
  public:
    EGraphEuclideanHeuristic(const EGraphable<vector<double> >& env);
    void setGoal(const vector<double>& goal);
    int getHeuristic(const vector<double>& coord);
    void getEGraphVerticesWithSameHeuristic(const vector<double>& coord, vector<EGraph::EGraphVertex*>& vertices);
    void runPrecomputations();
    void getDirectShortcut(int component, vector<EGraph::EGraphVertex*>& shortcuts);
    void resetShortcuts();

  protected:
    CHeap heap;

    std::vector<EGraph::EGraphVertex*> shortcut_cache_;
    const EGraphable<vector<double> >& env_;
    vector<double> goal_;
};

#endif
