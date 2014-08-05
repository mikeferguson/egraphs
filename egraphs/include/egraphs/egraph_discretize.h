#ifndef EGRAPH_DISCRETIZE
#define EGRAPH_DISCRETIZE

#include<vector>

class EGraphDiscretize{
  public:
    virtual void discToCont(const std::vector<int>& d, std::vector<double>& c) = 0;
    virtual void contToDisc(const std::vector<double>& c, std::vector<int>& d) = 0;
};

#endif
