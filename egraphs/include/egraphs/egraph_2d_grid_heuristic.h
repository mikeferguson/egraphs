#ifndef EGRAPH_2D_GRID_HEURISTIC_H
#define EGRAPH_2D_GRID_HEURISTIC_H

#include<egraphs/egraph_heuristic.h>
#include<egraphs/egraph.h>
#include<egraphs/egraph_down_project.h>
#include<sbpl/headers.h>

/*
-a general N-dimensional dijkstra
--the constructor
---a function which given a coordinate (an egraph vertex) returns a vector of length n (the down-projection)
--setGoal will get a new collision grid, wipe g-values from the last search, and insert the goal into the queue
--getHeuristic is an on-demand dijkstra which uses the given successors and the e-graph edges provided by the setEGraph function until queried cell is filled in
--setEGraph gets a list of edges in the e-graph. it turns these into n-dimensional edges by calling the down-project function on the endpoints of each edge
-a 2D, 8-connected uniform cost dijkstra (an optimized version of the n-dimensional version)
*/

class EGraph2dGridHeuristic : public EGraphHeuristic{
  public:
    EGraph2dGridHeuristic(EGraphDownProject* downProject, int size_x, int size_y, int move_cost);
    void setGrid(vector<vector<bool> >& grid);
    void setGoal(vector<double> goal);
    int getHeuristic(vector<double> coord);
    void getEGraphVerticesWithSameHeuristic(vector<double> coord, vector<EGraph::EGraphVertex*> vertices);
    void runPrecomputations();

  protected:
    class EGraph2dGridHeuristicCell: public AbstractSearchState{
      public:
        EGraph2dGridHeuristicCell(){};
        ~EGraph2dGridHeuristicCell(){};
          
        bool closed;
        int id;
        int cost;
        vector<EGraph::EGraphVertex*> egraph_vertices;
    };

    int sizex_;
    int sizey_;
    int width_;
    int height_;
    int planeSize_;
    int cost_1_move_;
    int inflated_cost_1_move_;
    CHeap heap;
    vector<int> goal_dp_;

    vector<EGraph2dGridHeuristicCell> heur;
    EGraphDownProject* downProject_;
};

#endif
