#include "ros/ros.h"
#include "costmap_2d/costmap_2d.h"
#include "costmap_2d/costmap_2d_publisher.h"
#include "costmap_2d/footprint.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/PolygonStamped.h"

class Covered_Area{
  public:
    Covered_Area(){
      ROS_INFO("Initialize node covered_area");
      sub = n.subscribe("/move_base/global_costmap/footprint",100,&Covered_Area::footprintCallback,this);
      nav_msgs::MapMetaData map = *ros::topic::waitForMessage<nav_msgs::MapMetaData>("/map_metadata");
      covered_costmap = costmap_2d::Costmap2D(map.width, map.height, map.resolution, map.origin.position.x, map.origin.position.y);
    }
    
    void footprintCallback(geometry_msgs::PolygonStamped footprint){
      ROS_INFO("x %.2f y %.2f",footprint.polygon.points[0].x,footprint.polygon.points[0].y);
      covered_costmap.setConvexPolygonCost(costmap_2d::toPointVector(footprint.polygon),254);
    }

    ros::NodeHandle n;
    ros::Subscriber sub;
    costmap_2d::Costmap2D covered_costmap;
};

int main(int argc, char **argv){
  ros::init(argc, argv, "covered_area");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  Covered_Area cv;
  costmap_2d::Costmap2DPublisher pub = costmap_2d::Costmap2DPublisher(&cv.n,&cv.covered_costmap,"map","/covered_costmap");
  pub.publishCostmap();
  ros::waitForShutdown();
  return 0;
}
 

