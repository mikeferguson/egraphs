#ifndef EGRAPH_EUCLIDEAN_HEURISTIC_H
#define EGRAPH_EUCLIDEAN_HEURISTIC_H

#include<egraphs/egraph_heuristic.h>
#include<egraphs/egraph.h>
#include<sbpl/headers.h>
#include<map>

class EGraphEuclideanHeuristic : public EGraphHeuristic<vector<double> >{
  public:
    EGraphEuclideanHeuristic(const EGraphable<vector<double> >& env, double distance_inflation);
    EGraphEuclideanHeuristic(const EGraphable<vector<double> >& env, const vector<double>& element_diff_inflation);
    void setGoal(const vector<double>& goal);
    int getHeuristic(const vector<double>& coord);
    void getEGraphVerticesWithSameHeuristic(const vector<double>& coord, vector<EGraph::EGraphVertex*>& vertices);
    void runPrecomputations();
    void getDirectShortcut(int component, vector<EGraph::EGraphVertex*>& shortcuts);
    void resetShortcuts();
    inline int euclideanDistance(const vector<double>& c1, const vector<double>& c2);

  protected:

    class EGraphEuclideanState : public AbstractSearchState{
      public:
        int id;
        int g;
        vector<double> coord;
    };

    CHeap heap;

    double dist_inflation;
    vector<double> inflation;
    std::vector<EGraph::EGraphVertex*> shortcut_cache_;
    const EGraphable<vector<double> >& env_;
    vector<double> goal_;

    vector<EGraphEuclideanState> verts;
};

#endif
