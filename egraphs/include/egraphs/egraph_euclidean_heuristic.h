#ifndef EGRAPH_EUCLIDEAN_HEURISTIC_H
#define EGRAPH_EUCLIDEAN_HEURISTIC_H

#include<egraphs/egraph_heuristic.h>
#include<egraphs/egraph.h>
#include<sbpl/headers.h>
#include<map>
#include<vector>
#include <flann/flann.hpp>
#include <angles/angles.h>

#define const_arm_xyz_weight (40.0/0.02 * 0.1)
#define const_arm_fa_weight (40.0/5.0/(2.0*M_PI/180.0) * 0.01)
#define const_base_xy_weight (40.0/0.02)
#define const_base_xy_weight2 (251.0/(M_PI/8.0))
#define const_base_z_weight (3000.0/0.02 * 0.001)

template<class T>
struct EG_DIST
{
  typedef bool is_kdtree_distance;

  typedef T ElementType;
  typedef typename flann::Accumulator<T>::Type ResultType;

  template <typename Iterator1, typename Iterator2>
    ResultType operator()(Iterator1 a, Iterator2 b, size_t size, ResultType = -1) const
    {
      ResultType result = ResultType();
      ResultType diff;

      diff = (*a++ - *b++) * const_arm_xyz_weight;
      result += diff*diff;
      diff = (*a++ - *b++) * const_arm_xyz_weight;
      result += diff*diff;
      diff = (*a++ - *b++) * const_arm_xyz_weight;
      result += diff*diff;

      diff = (*a++ - *b++) * const_arm_fa_weight;
      result += diff*diff;

      diff = (*a++ - *b++) * const_base_xy_weight;
      result += diff*diff;
      diff = (*a++ - *b++) * const_base_xy_weight;
      result += diff*diff;

      diff = (*a++ - *b++) * const_base_xy_weight2;
      result += diff*diff;
      diff = (*a++ - *b++) * const_base_xy_weight2;
      result += diff*diff;

      diff = (*a++ - *b++) * const_base_z_weight;
      result += diff*diff;

      return result;
    }

  template <typename U, typename V>
    inline ResultType accum_dist(const U& a, const V& b, int idx) const
    {  
      ResultType diff;
      switch(idx){
        case 0:
          diff = (a-b) * const_arm_xyz_weight;
          return diff*diff;
        case 1:
          diff = (a-b) * const_arm_xyz_weight;
          return diff*diff;
        case 2:
          diff = (a-b) * const_arm_xyz_weight;
          return diff*diff;
        case 3:
          diff = (a-b) * const_arm_fa_weight;
          return diff*diff;
        case 4:
          diff = (a-b) * const_base_xy_weight;
          return diff*diff;
        case 5:
          diff = (a-b) * const_base_xy_weight;
          return diff*diff;
        case 6:
          diff = (a-b) * const_base_xy_weight2;
          return diff*diff;
        case 7:
          diff = (a-b) * const_base_xy_weight2;
          return diff*diff;
        case 8:
          diff = (a-b) * const_base_z_weight;
          return diff*diff;
        default:
          return 0;
      }
    }
};

/*
#define const_arm_xyz_weight (40.0/0.02 * 0.1)
#define const_arm_rpy_weight (40.0/2.0/(5.625*M_PI/180.0) * 0.1)
#define const_arm_fa_weight (40.0/5.0/(2.0*M_PI/180.0) * 0.01)
#define const_base_xy_weight (40.0/0.02)
#define const_base_z_weight (3000.0/0.02 * 0.001)
#define const_base_theta_weight (251.0/(M_PI/8.0))
    
template<class T>
struct EG_DIST
{
  typedef bool is_kdtree_distance;

  typedef T ElementType;
  typedef typename flann::Accumulator<T>::Type ResultType;

  template <typename Iterator1, typename Iterator2>
    ResultType operator()(Iterator1 a, Iterator2 b, size_t size, ResultType = -1) const
    {
      ResultType result = ResultType();
      ResultType diff;

      diff = (*a++ - *b++) * const_arm_xyz_weight;
      result += diff*diff;
      diff = (*a++ - *b++) * const_arm_xyz_weight;
      result += diff*diff;
      diff = (*a++ - *b++) * const_arm_xyz_weight;
      result += diff*diff;

      diff = angles::shortest_angular_distance(*b++, *a++) * const_arm_rpy_weight;
      result += diff*diff;
      diff = angles::shortest_angular_distance(*b++, *a++) * const_arm_rpy_weight;
      result += diff*diff;
      diff = angles::shortest_angular_distance(*b++, *a++) * const_arm_rpy_weight;
      result += diff*diff;

      diff = (*a++ - *b++) * const_arm_fa_weight;
      result += diff*diff;
      diff = (*a++ - *b++) * const_arm_fa_weight;
      result += diff*diff;

      diff = (*a++ - *b++) * const_base_xy_weight;
      result += diff*diff;
      diff = (*a++ - *b++) * const_base_xy_weight;
      result += diff*diff;

      diff = (*a++ - *b++) * const_base_z_weight;
      result += diff*diff;

      diff = angles::shortest_angular_distance(*b++, *a++) * const_base_theta_weight;
      result += diff*diff;

      return result;
    }

  template <typename U, typename V>
    inline ResultType accum_dist(const U& a, const V& b, int idx) const
    {  
      ResultType diff;
      switch(idx){
        case 0:
          diff = (a-b) * const_arm_xyz_weight;
          return diff*diff;
        case 1:
          diff = (a-b) * const_arm_xyz_weight;
          return diff*diff;
        case 2:
          diff = (a-b) * const_arm_xyz_weight;
          return diff*diff;
        case 3:
          diff = angles::shortest_angular_distance(b, a) * const_arm_rpy_weight;
          return diff*diff;
        case 4:
          diff = angles::shortest_angular_distance(b, a) * const_arm_rpy_weight;
          return diff*diff;
        case 5:
          diff = angles::shortest_angular_distance(b, a) * const_arm_rpy_weight;
          return diff*diff;
        case 6:
          diff = (a-b) * const_arm_fa_weight;
          return diff*diff;
        case 7:
          diff = (a-b) * const_arm_fa_weight;
          return diff*diff;
        case 8:
          diff = (a-b) * const_base_xy_weight;
          return diff*diff;
        case 9:
          diff = (a-b) * const_base_xy_weight;
          return diff*diff;
        case 10:
          diff = (a-b) * const_base_z_weight;
          return diff*diff;
        case 11:
          diff = angles::shortest_angular_distance(b, a) * const_base_theta_weight;
          return diff*diff;
        default:
          return 0;
      }
    }
};
*/

class EGraphEuclideanHeuristic : public EGraphHeuristic<std::vector<double> >{
  public:

    EGraphEuclideanHeuristic(const EGraphable<std::vector<double> >& env);

    EGraphEuclideanHeuristic(const EGraphable<std::vector<double> >& env, double distance_inflation);
    EGraphEuclideanHeuristic(const EGraphable<std::vector<double> >& env, const std::vector<double>& element_diff_inflation);
    EGraphEuclideanHeuristic(const EGraphable<std::vector<double> >& env, 
        double distance_inflation, const std::vector<bool>& cont);
    EGraphEuclideanHeuristic(const EGraphable<std::vector<double> >& env, 
        const std::vector<double>& element_diff_inflation, const std::vector<bool>& cont);

    ~EGraphEuclideanHeuristic();

    void setGoal(const std::vector<double>& goal);
    int getHeuristic(const std::vector<double>& coord);
    void getEGraphVerticesWithSameHeuristic(const std::vector<double>& coord, std::vector<EGraph::EGraphVertex*>& vertices);
    void runPrecomputations();
    void getDirectShortcut(int component, std::vector<EGraph::EGraphVertex*>& shortcuts);
    void resetShortcuts();
    inline int euclideanDistance(const std::vector<double>& c1, const std::vector<double>& c2);

    void setSnapDist(const std::vector<double>& sd){
      dist_to_snap = sd;
    }

  protected:

    class EGraphEuclideanState : public AbstractSearchState{
      public:
        int id;
        int g;
        std::vector<double> coord;
    };

    CHeap heap;

    double dist_inflation;
    std::vector<double> inflation;
    std::vector<EGraph::EGraphVertex*> shortcut_cache_;
    const EGraphable<std::vector<double> >& env_;
    std::vector<double> goal_;
    std::vector<bool> continuous_joint;
    int last_best_idx;
    std::vector<double> dist_to_snap;

    std::vector<EGraphEuclideanState> verts;


    void initNaive();
    int naiveGetHeuristic(const std::vector<double>& coord, int& best_idx);

    //kd-tree stuff
    std::vector<std::pair<int,int> > gval;
    std::vector<std::pair<int,int> > gval_sorted;
    std::vector<std::vector<int> > fw_matrix;
    std::vector<std::vector<double> > coords_;
    //flann::Index<flann::L2<float> >* index;
    //flann::Matrix<float> dataMatrix;
    flann::Index<EG_DIST<float> >* index;
    flann::Matrix<float> dataMatrix;
    bool print;
    EG_DIST<float> eg_dist;
    flann::Matrix<int> indices;
    flann::Matrix<float> dists;
    flann::Matrix<float> query;
};

#endif
