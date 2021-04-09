# Covered Area
Package to compute the area covered for the footprint of a robot.  
Using ROS Noetic and Move Base

![Screenshot from 2021-04-09 12-12-01](https://user-images.githubusercontent.com/59148199/114165479-d2d95000-992c-11eb-909f-fce6e50bddf8.png)

# USE

Clone this package into your workspace and compile it.  
**IMPORTANT!**  
You have to get the **move base node** running with a map before running this node.  
``` rosrun covered_area covered_area_node ```  

The area is calculated taking into account the map resolution.  

### SERVICES
``` rosservice call /covered_area/Reset_Area``` 
### PARAMETERS
``` rosparam set /covered_area/redundant_area false ```  
Set to false this parameter to compute ONLY the new area  

# VIDEO
https://youtu.be/xBXFLSIzrRs
