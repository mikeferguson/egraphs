#ifndef EGRAPH_DOWN_PROJECT_H
#define EGRAPH_DOWN_PROJECT_H

class EGraphDownProject{
  public:
    //requires a downProject function which takes a coordinate in the state space and projects it down into the simpler heuristic space
    virtual void downProject(const vector<double>& coord, vector<int>& dp) = 0;
};

#endif
