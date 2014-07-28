#ifndef EGRAPH_EUCLIDEAN_HEURISTIC_H
#define EGRAPH_EUCLIDEAN_HEURISTIC_H

#include<egraphs/egraph_heuristic.h>
#include<egraphs/egraph.h>
#include<sbpl/headers.h>
#include<map>

class EGraphEuclideanHeuristic : public EGraphHeuristic<vector<double> >{
  public:
    EGraphEuclideanHeuristic(const EGraphable<vector<double> >& env) : env_(env) {};
    void setGoal(const vector<double>& goal){};
    int getHeuristic(const vector<double>& coord){return 0;};
    void getEGraphVerticesWithSameHeuristic(const vector<double>& coord, vector<EGraph::EGraphVertex*>& vertices){};
    void runPrecomputations(){};
    void getDirectShortcut(int component, vector<EGraph::EGraphVertex*>& shortcuts){};
    virtual void resetShortcuts(){};

  protected:
    CHeap heap;

    std::vector<bool> empty_components_;
    std::vector<EGraph::EGraphVertex*> shortcut_cache_;
    const EGraphable<vector<double> >& env_;
};

#endif
