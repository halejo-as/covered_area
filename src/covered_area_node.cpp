#include "ros/ros.h"
#include "costmap_2d/costmap_2d.h"
#include "costmap_2d/costmap_2d_publisher.h"
#include "costmap_2d/footprint.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Twist.h"

class Covered_Area{
  public:
    Covered_Area(bool r_area){
      ROS_INFO("Initialize node covered_area");
      sub = n.subscribe("/cmd_vel",100,&Covered_Area::movementCallback,this);
      nav_msgs::MapMetaData map = *ros::topic::waitForMessage<nav_msgs::MapMetaData>("/map_metadata");
      covered_costmap = costmap_2d::Costmap2D(map.width, map.height, map.resolution, map.origin.position.x, map.origin.position.y);
      area = 0;
      redundant_area = r_area;
    }

    bool setConvexPolygonVisited(const std::vector<geometry_msgs::Point>& polygon){
      // we assume the polygon is given in the global_frame... we need to transform it to map coordinates
      std::vector<costmap_2d::MapLocation> map_polygon;
      for (unsigned int i = 0; i < polygon.size(); ++i)
      {
        costmap_2d::MapLocation loc;
        if (!covered_costmap.worldToMap(polygon[i].x, polygon[i].y, loc.x, loc.y))
        {
          // ("Polygon lies outside map bounds, so we can't fill it");
          return false;
        }
        map_polygon.push_back(loc);
      }
      std::vector<costmap_2d::MapLocation> polygon_cells;

      // get the cells that fill the polygon
      covered_costmap.convexFillCells(map_polygon, polygon_cells);

      // set the cost of those cells
      for (unsigned int i = 0; i < polygon_cells.size(); ++i)
      {
        unsigned int index = covered_costmap.getIndex(polygon_cells[i].x, polygon_cells[i].y);
        if(redundant_area){
          if ( std::find(last_footprint_indx.begin(), last_footprint_indx.end(), index) == last_footprint_indx.end() ){
            area++;
          }
          footprint_indx.push_back(index);
        }
        else if( covered_costmap.getCharMap()[index] != 254){
          area ++;
        }
        else{
        }

        covered_costmap.getCharMap()[index] = 254;

      }
      if(redundant_area){
        last_footprint_indx = footprint_indx;
        footprint_indx = {};
      }
      return true;
    }
    
    //void footprintCallback(geometry_msgs::PolygonStamped footprint){
    void movementCallback(geometry_msgs::Twist vel){
      geometry_msgs::PolygonStamped footprint = *ros::topic::waitForMessage<geometry_msgs::PolygonStamped>("/move_base/global_costmap/footprint");
      //covered_costmap.setConvexPolygonCost(costmap_2d::toPointVector(footprint.polygon),254);
      setConvexPolygonVisited(costmap_2d::toPointVector(footprint.polygon));
      ROS_INFO("area %.4f\n",area*covered_costmap.getResolution()*covered_costmap.getResolution());
    }
    ros::NodeHandle n;
    ros::Subscriber sub;
    costmap_2d::Costmap2D covered_costmap;
    int area;
    bool redundant_area;
    std::vector<unsigned int> last_footprint_indx;
    std::vector<unsigned int> footprint_indx;
};

int main(int argc, char **argv){
  ros::init(argc, argv, "covered_area");
  Covered_Area cv(true);
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::Rate loop_rate(10);
  costmap_2d::Costmap2DPublisher pub = costmap_2d::Costmap2DPublisher(&cv.n,&cv.covered_costmap,"map","/covered_costmap",true);
  while(ros::ok()){
    pub.publishCostmap();
    loop_rate.sleep();
  }
  ros::waitForShutdown();
  return 0;
}
 

