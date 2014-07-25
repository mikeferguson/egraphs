#ifndef EGRAPH_DISCRETIZE
#define EGRAPH_DISCRETIZE

class EGraphDiscretize{
  public:
    virtual void discToCont(const vector<int>& d, vector<double>& c) = 0;
    virtual void contToDisc(const vector<double>& c, vector<int>& d) = 0;
};

#endif
