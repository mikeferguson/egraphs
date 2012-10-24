#ifndef EGRAPH_VISUALIZER_H
#define EGRAPH_VISUALIZER_H

#include<egraphs/egraph.h>
#include<egraph_vis/egraph_marker_maker.h>
#include<visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>

class EGraphVisualizer{
  public:
    EGraphVisualizer(EGraph* eg, EGraphMarkerMaker* converter);
    void visualize();

  protected:
    EGraph* eg_;
    EGraphMarkerMaker* converter_;
    ros::Publisher marker_pub_;
};

#endif
