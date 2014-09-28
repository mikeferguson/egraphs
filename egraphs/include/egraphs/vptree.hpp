// A VP-Tree implementation, by Steve Hanov. (steve.hanov@gmail.com)
// Released to the Public Domain
// Based on "Data Structures and Algorithms for Nearest Neighbor Search" by Peter N. Yianilos
//#include <stdlib>
#include <algorithm>
#include <vector>
//#include <stdio>
#include <queue>
#include <limits>

template<typename T, int (*distance)( const T&, const T& )>
class VpTree{
  public:
    VpTree() : _root(0) {}

    ~VpTree() {
      delete _root;
    }

    void create( const std::vector<T>& items, int leaf_size ) {
      delete _root;
      _items = items;
      _leaf_size = std::max(leaf_size, 2);
      _root = buildFromPoints(0, items.size());
    }

    void search( const T& target, T& result, int& dist, double eps) 
    {
      _tau = std::numeric_limits<int>::max();
      _eps = eps;
      _target = target;
      search( _root );

      result = _items[_result_idx];
      dist = _tau;
    }

  private:
    std::vector<T> _items;
    int _tau;
    double _eps;
    T _target;
    int _result_idx;
    int _leaf_size;

    struct Node 
    {
      int index;
      int threshold;
      Node* left;
      Node* right;
      std::vector<int> leaflets;

      Node() :
        index(0), threshold(0.), left(0), right(0) {}

      ~Node() {
        delete left;
        delete right;
      }
    }* _root;

    struct DistanceComparator
    {
      const T& item;
      DistanceComparator( const T& item ) : item(item) {}
      bool operator()(const T& a, const T& b) {
        return distance( item, a ) < distance( item, b );
      }
    };

    Node* buildFromPoints( int lower, int upper )
    {
      if ( upper == lower ) {
        return NULL;
      }

      Node* node = new Node();

      if(upper-lower <= _leaf_size){//_leaf_size is always at least 2
        node->index = -1;
        node->leaflets.resize(upper-lower);
        for(int i=lower; i<upper; ++i)
          node->leaflets[i-lower] = i;
        return node;
      }
      //if we make it here we must have at least 3 points, so we will make 2 children

      // choose an arbitrary point and move it to the start
      int i = (int)((double)rand() / RAND_MAX * (upper - lower - 1) ) + lower;
      std::swap( _items[lower], _items[i] );

      int median = (upper + lower + 1)/2; 

      // partitian around the median distance
      std::nth_element( 
          _items.begin() + lower + 1, 
          _items.begin() + median,
          _items.begin() + upper,
          DistanceComparator( _items[lower] ));

      // what was the median?
      node->threshold = distance( _items[lower], _items[median] );

      node->index = lower;
      node->left = buildFromPoints( lower + 1, median );
      node->right = buildFromPoints( median, upper );

      return node;
    }

    void search( Node* node){

      if(node->index < 0){
        //this is a leaf node
        for(unsigned int i=0; i<node->leaflets.size(); ++i){
          int d = distance(_items[node->leaflets[i]], _target);
          if(d < _tau){
            _tau = d;
            _result_idx = node->leaflets[i];
          }
        }
        return;
      }

      int dist = distance( _items[node->index], _target );
      //printf("dist=%g tau=%gn", dist, _tau );

      if ( dist < _tau ) {
        _tau = dist;
        _result_idx = node->index;
      }

      if( dist < node->threshold ) {
        search( node->left);

        if ( _tau >= _eps*(node->threshold-dist) ) 
          search( node->right );

      } 
      else {
        search( node->right );

        if ( _tau >= _eps*(dist-node->threshold) )
          search( node->left );

      }
    }
};

