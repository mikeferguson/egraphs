#ifndef EGRAPH_MARKER_MAKER_H
#define EGRAPH_MARKER_MAKER_H

#include<vector>
#include<visualization_msgs/Marker.h>
using namespace std;

class EGraphMarkerMaker{
  public:
    virtual visualization_msgs::Marker stateToVisualizationMarker(vector<double> coord) = 0;
    virtual visualization_msgs::Marker edgeToVisualizationMarker(vector<double> coord, vector<double> coord2) = 0;
    virtual visualization_msgs::Marker stateToDetailedVisualizationMarker(vector<double> coord) = 0;
};

#endif

/*
change the name of this class.....

a EGraphVisualizer that takes an egraph and an EGraphMarkerMaker
it has a visualize function which runs through the egraph and
calls stateToVisualizationMarker and edgeToVisualizationMarker 
each state and edge. It wraps interactive markers around them,
and publishes. 
When the user clicks on a state, it draws the detailed version of the 
state using stateToDetailedVisualizationMarker. it also highlights
neighbors, shows costs to each, shows heuristics for the selected state
and its neighbors. it also highlights shortcuts for the selected state.
this may look crowded...so i may have check boxes in the menu attached 
to the selected state
the user write the 3 conversion functions to adhere to the EGraphMarkerMaker interface

we may need to have the egraph maintain a version number internally
each time the addPath function is called it updates the number
this way when the user interacts we can make sure the egraph hasn't changed under us and cause a crash...
*/
