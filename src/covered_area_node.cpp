#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/footprint.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Twist.h>
#include "covered_area/Reset_Area.h"

class Covered_Area{
  public:
    /* Construction of Covered Area class
     * @param node handler
     * @param r_area True if you wish to add to the calculation the area already
     *               visited or false to compute only new area visited
     */
    Covered_Area(ros::NodeHandle *nh, bool r_area = true ):
      n(*nh),redundant_area(r_area), area(0)
    {
      sub = n.subscribe("/cmd_vel",5,&Covered_Area::movementCallback,this);
      srv = n.advertiseService("Reset_Area",&Covered_Area::ResetArea,this);
      nav_msgs::MapMetaData map = *ros::topic::waitForMessage<nav_msgs::MapMetaData>("/map_metadata");
      covered_costmap = costmap_2d::Costmap2D(map.width, map.height, map.resolution, map.origin.position.x, map.origin.position.y);
      map_res = map.resolution;
      n.setParam("redundant_area",redundant_area);
    }
    
    /* Adaptation for the function used in the move base library*/
    bool setConvexPolygonVisited(const std::vector<geometry_msgs::Point>& polygon)
    {
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

      n.param("redundant_area",redundant_area,true);
      // set the cost of the cells covered for the footprint
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
    
    // Callback when the robot is moving show the value of the area covered
    void movementCallback(geometry_msgs::Twist vel)
    {
      geometry_msgs::PolygonStamped footprint = *ros::topic::waitForMessage<geometry_msgs::PolygonStamped>("/move_base/global_costmap/footprint");
      setConvexPolygonVisited(costmap_2d::toPointVector(footprint.polygon));
      ROS_INFO("area %.4f m^2\n",area*map_res*map_res);
    }

    // ROS Service to reset all the variables and start again the area calculation
    bool ResetArea(covered_area::Reset_Area::Request &req, covered_area::Reset_Area::Response &res)
    {
      ROS_INFO("Reset Area ...");
      area = 0;
      nav_msgs::MapMetaData map = *ros::topic::waitForMessage<nav_msgs::MapMetaData>("/map_metadata");
      covered_costmap = costmap_2d::Costmap2D(map.width, map.height, map.resolution, map.origin.position.x, map.origin.position.y);
      res.success = true;
      return true;
    }

    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::ServiceServer srv;
    costmap_2d::Costmap2D covered_costmap;
    float map_res;
    int area;
    bool redundant_area;
    std::vector<int> last_footprint_indx;
    std::vector<int> footprint_indx;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "covered_area");
  ros::NodeHandle nh("~");
  ROS_INFO("Start node %s",nh.getNamespace().c_str());
  Covered_Area cv(&nh);
  ros::AsyncSpinner spinner(0); // Helps to a right visualisation in RViz
  spinner.start();
  ros::Rate loop_rate(10);
  // Publish the cotmap to show the area covered
  costmap_2d::Costmap2DPublisher pub = costmap_2d::Costmap2DPublisher(&nh,&cv.covered_costmap,"map","covered_costmap",true);
  while(ros::ok()){
    pub.publishCostmap();
    loop_rate.sleep();
  }
  ros::waitForShutdown();
  return 0;
}
 

