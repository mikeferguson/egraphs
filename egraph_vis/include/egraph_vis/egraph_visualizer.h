#ifndef EGRAPH_VISUALIZER_H
#define EGRAPH_VISUALIZER_H

#include<egraphs/egraph.h>
#include<egraph_vis/egraph_marker_maker.h>
#include<visualization_msgs/Marker.h>
#include<interactive_markers/interactive_marker_server.h>
#include<interactive_markers/menu_handler.h>

class EGraphVisualizer{
  public:
    EGraphVisualizer(EGraph* eg, EGraphMarkerMaker* converter);
    ~EGraphVisualizer();
    void visualize();

  protected:
    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    EGraph* eg_;
    EGraphMarkerMaker* converter_;

    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    interactive_markers::MenuHandler menu_handler_;
};

#endif
