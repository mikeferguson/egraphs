#ifndef EGRAPH_2D_GRID_HEURISTIC_H
#define EGRAPH_2D_GRID_HEURISTIC_H

#include<egraphs/egraph_heuristic.h>
#include<egraphs/egraph.h>
#include<sbpl/headers.h>
#include<map>

/*
-a general N-dimensional dijkstra
--the constructor
---a function which given a coordinate (an egraph vertex) returns a vector of length n (the down-projection)
--setGoal will get a new collision grid, wipe g-values from the last search, and insert the goal into the queue
--getHeuristic is an on-demand dijkstra which uses the given successors and the e-graph edges provided by the setEGraph function until queried cell is filled in
--setEGraph gets a list of edges in the e-graph. it turns these into n-dimensional edges by calling the down-project function on the endpoints of each edge
-a 2D, 8-connected uniform cost dijkstra (an optimized version of the n-dimensional version)
*/

class EGraph2dGridHeuristic : public EGraphHeuristic<vector<int> >{
  public:
    EGraph2dGridHeuristic(const EGraphable<vector<int> >& env, int size_x, int size_y, int move_cost);
    void setGrid(const vector<vector<bool> >& grid);
    void setGoal(const vector<int>& goal);
    int getHeuristic(const vector<int>& coord);
    void getEGraphVerticesWithSameHeuristic(const vector<int>& coord, vector<EGraph::EGraphVertex*>& vertices);
    void runPrecomputations();
    void getDirectShortcut(int component, vector<EGraph::EGraphVertex*>& shortcuts);
    virtual void resetShortcuts();

  protected:
    class EGraph2dGridHeuristicCell: public AbstractSearchState{
      public:
        EGraph2dGridHeuristicCell(){
          open_iteration = 0;
          closed_iteration = 0;
        };
        ~EGraph2dGridHeuristicCell(){};
          
        int open_iteration;
        int closed_iteration;
        int id;
        int cost;
        vector<EGraph::EGraphVertex*> egraph_vertices;
    };

    int iteration_;

    int sizex_;
    int sizey_;
    int width_;
    int height_;
    int planeSize_;
    int cost_1_move_;
    int inflated_cost_1_move_;
    CHeap heap;
    CHeap sc_heap;
    vector<int> goal_dp_;

    std::vector<bool> empty_components_;
    std::vector<EGraph::EGraphVertex*> shortcut_cache_;
    vector<EGraph2dGridHeuristicCell> heur;
    vector<EGraph2dGridHeuristicCell> sc;
    const EGraphable<vector<int> >& env_;
};

#endif
