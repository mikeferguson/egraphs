#include <algorithm>
#include <vector>
#include <stdio.h>
#include <queue>
#include <limits>
#include <assert.h>

#define traceSearch false

template<typename T, int (*distance)( const T&, const T& )>
class GhTree{
  public:
    GhTree() : root_(0) {}

    ~GhTree() {
      delete root_;
    }

    void create( const std::vector<T>& items, int leaf_size ) {
      delete root_;
      items_ = items;
      //a leaf can always contain 2 points, but for search efficiency we always want 2 children, or no children so 
      //so if there are 3 or less points we want to make it a leaf (there will only be 1 child with 3 points)
      leaf_size_ = std::max(leaf_size,3);
      root_ = buildFromPoints(0, items.size());//, 0);
    }

    void search(const T& target, T& result, int& dist, double eps){
      eps_ = eps;
      tau_ = std::numeric_limits<int>::max();
      target_ = target;
      search(root_);

      result = items_[result_idx];
      dist = tau_;
    }

  private:
    std::vector<T> items_;
    int tau_;
    double eps_;
    T target_;
    int result_idx;
    int leaf_size_;

    struct Node {
      int index1, index2;
      int threshold;// median dv1 - dv2 
      Node* left;
      Node* right;
      //int depth;
      std::vector<int> leaflets;//MIKE PICK UP HERE! add multi-item leaves!

      Node() :
        index1(0), index2(0), threshold(0), left(0), right(0) {}

      ~Node() {
        delete left;
        delete right;
      }
    }* root_;

    struct DistanceComparator
    {
      const T& item1;
      const T& item2;
      DistanceComparator( const T& i1, const T& i2 ) : item1(i1), item2(i2) {}
      bool operator()(const T& a, const T& b) {
        return (distance(item1,a)-distance(item2,a)) < (distance(item1,b)-distance(item2,b));
      }
    };

    //Node* buildFromPoints( int lower, int upper , int depth){
    Node* buildFromPoints( int lower, int upper){
      //we don't have any data points left
      if(upper == lower){
        return NULL;
      }

      Node* node = new Node();
      //node->depth = depth;

      /*
      //we have one data point
      if(upper-lower == 1){
        node->index1 = lower;
        node->index2 = lower;
        return node;
      }
      //we have two data points
      if(upper-lower == 2){
        node->index1 = lower;
        node->index2 = lower+1;
        return node;
      }
      //if we make it here we must have at least 3 points, so we will make at least 1 child
      */

      if(upper-lower <= leaf_size_){//leaf_size_ is always at least 3
        node->index1 = -1;
        node->leaflets.resize(upper-lower);
        for(int i=lower; i<upper; ++i)
          node->leaflets[i-lower] = i;
        return node;
      }
      //if we make it here we must have at least 4 points, so we will make 2 children

      //choose a point at random
      int r = (int)((double)rand() / RAND_MAX * (upper - lower - 1) ) + lower;
      std::swap(items_[lower], items_[r]);
      //find the point farthest from it and make that v1
      int farthest_dist = 0;
      int v1 = -1;
      for(int i=lower+1; i<upper; i++){
        int d = distance(items_[lower], items_[i]);
        if(d > farthest_dist){
          farthest_dist = d;
          v1 = i;
        }
      }
      std::swap(items_[lower], items_[v1]);
      //find the point farthest from v1 and make that v2
      farthest_dist = 0;
      int v2 = -1;
      for(int i=lower+1; i<upper; i++){
        int d = distance(items_[lower], items_[i]);
        if(d > farthest_dist){
          farthest_dist = d;
          v2 = i;
        }
      }
      std::swap(items_[lower+1], items_[v2]);
      //at this point lower contains v1 and lower+1 contains v2

      //we will now partition the remaining elements (from lower+2 and up) into 2 roughly even groups
      //the half of the points closest to v1 and the rest being the half closer to v2
      //find median of (dv1 - dv2), everything below the median goes to v1 and everything greater or equal goes to v2

      //this is the median
      int median = (upper + lower + 2)/2;

      // partitian around the median distance
      std::nth_element( 
          items_.begin() + lower + 2, 
          items_.begin() + median,
          items_.begin() + upper,
          DistanceComparator(items_[lower], items_[lower+1]));
      
      // what was the median dv1-dv2?
      node->threshold = distance(items_[lower], items_[median]) - distance(items_[lower+1], items_[median]);

      node->index1 = lower;
      node->index2 = lower+1;
      node->left = buildFromPoints(lower+2, median);//, depth+1);
      node->right = buildFromPoints(median, upper);//, depth+1);

      return node;
    }

    void search(Node* node){
      //if(node == NULL) 
        //return;

      if(node->index1 < 0){
        //this is a leaf node
        for(unsigned int i=0; i<node->leaflets.size(); ++i){
          int d = distance(items_[node->leaflets[i]], target_);
          if(d < tau_){
            tau_ = d;
            result_idx = node->leaflets[i];
          }
        }
        return;
      }

      int d1 = distance(items_[node->index1], target_);
      int d2 = distance(items_[node->index2], target_);

      if(tau_ > d1){
        tau_ = d1;
        result_idx = node->index1;
      }
      if(tau_ > d2){
        tau_ = d2;
        result_idx = node->index2;
      }
      //if(traceSearch)
        //printf("(depth=%d) d1=%f d2=%f tau=%f\n", node->depth, d1, d2, tau_ );

      //if(node->left == NULL && node->right == NULL)
        //return;

      //if there is only 1 child, it will be the right child
      //so we must always have a right child at this point...
      //assert( node->right );

      //if(node->left == NULL){
        //if(traceSearch)
          //printf("%d right\n",node->depth);
        //search(node->right);
        //return;
      //}

      //at this point we must have both children and a valid threshold
      int boundary_dist = d1-d2;
      //if(traceSearch){
        //printf("(depth=%d) anchor dist = %f\n", node->depth, distance(items_[node->index1],items_[node->index2]));
        //printf("(depth=%d) boundary_dist (d1-d2) = %f, threshold=%f\n",node->depth,boundary_dist,node->threshold);
      //}

      if(boundary_dist < node->threshold){
        //we are on the left of the boundary so we must recurse left
        //if(traceSearch)
          //printf("%d left\n",node->depth);
        search(node->left);
        //for optimality, check if we also have to recurse right
        //if( boundary_dist + 2*tau_/eps_ >= node->threshold  ){
        if( 2*tau_ >= eps_*(node->threshold-boundary_dist)  ){
          //if(traceSearch)
            //printf("%d right\n",node->depth);
          search(node->right);
        }
      }
      else{
        //we are on the right of the boundary so we must recurse right
        //if(traceSearch)
          //printf("%d right\n",node->depth);
        search(node->right);
        //for optimality, check if we also have to recurse left
        //if( boundary_dist - 2*tau_/eps_ < node->threshold  ){
        if( -2*tau_ < eps_*(node->threshold-boundary_dist)  ){
          //if(traceSearch)
            //printf("%d left\n",node->depth);
          search(node->left);
        }

      }
    }
};

