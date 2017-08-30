#ifndef COSTMAP_TOOLS_H_
#define COSTMAP_TOOLS_H_

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Polygon.h>

#include <costmap_2d/costmap_2d.h>
#include <boost/foreach.hpp>
#include <ros/ros.h>
#include <math.h>
#include <random>


namespace frontier_exploration{

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

/**
 * @brief Determine 4-connected neighbourhood of an input cell, checking for map edges
 * @param idx input cell index
 * @param costmap Reference to map data
 * @return neighbour cell indexes
 */
std::vector<unsigned int> nhood4(unsigned int idx, const costmap_2d::Costmap2D& costmap){
    //get 4-connected neighbourhood indexes, check for edge of map
    std::vector<unsigned int> out;

    unsigned int size_x_ = costmap.getSizeInCellsX(), size_y_ = costmap.getSizeInCellsY();

    if (idx > size_x_ * size_y_ -1){
        ROS_WARN("Evaluating nhood for offmap point");
        return out;
    }

    if(idx % size_x_ > 0){
        out.push_back(idx - 1);
    }
    if(idx % size_x_ < size_x_ - 1){
        out.push_back(idx + 1);
    }
    if(idx >= size_x_){
        out.push_back(idx - size_x_);
    }
    if(idx < size_x_*(size_y_-1)){
        out.push_back(idx + size_x_);
    }
    return out;

}

/**
 * @brief Determine 8-connected neighbourhood of an input cell, checking for map edges
 * @param idx input cell index
 * @param costmap Reference to map data
 * @return neighbour cell indexes
 */
std::vector<unsigned int> nhood8(unsigned int idx, const costmap_2d::Costmap2D& costmap){
    //get 8-connected neighbourhood indexes, check for edge of map
    std::vector<unsigned int> out = nhood4(idx, costmap);

    unsigned int size_x_ = costmap.getSizeInCellsX(), size_y_ = costmap.getSizeInCellsY();

    if (idx > size_x_ * size_y_ -1){
        return out;
    }

    if(idx % size_x_ > 0 && idx >= size_x_){
        out.push_back(idx - 1 - size_x_);
    }
    if(idx % size_x_ > 0 && idx < size_x_*(size_y_-1)){
        out.push_back(idx - 1 + size_x_);
    }
    if(idx % size_x_ < size_x_ - 1 && idx >= size_x_){
        out.push_back(idx + 1 - size_x_);
    }
    if(idx % size_x_ < size_x_ - 1 && idx < size_x_*(size_y_-1)){
        out.push_back(idx + 1 + size_x_);
    }

    return out;

}

/**
 * @brief Find nearest cell of a specified value
 * @param result Index of located cell
 * @param start Index initial cell to search from
 * @param val Specified value to search for
 * @param costmap Reference to map data
 * @return True if a cell with the requested value was found
 */
bool nearestCell(unsigned int &result, unsigned int start, unsigned char val, const costmap_2d::Costmap2D& costmap){

    const unsigned char* map = costmap.getCharMap();
    const unsigned int size_x = costmap.getSizeInCellsX(), size_y = costmap.getSizeInCellsY();

    if(start >= size_x * size_y){
        return false;
    }

    //initialize breadth first search
    std::queue<unsigned int> bfs;
    std::vector<bool> visited_flag(size_x * size_y, false);

    //push initial cell
    bfs.push(start);
    visited_flag[start] = true;

    //search for neighbouring cell matching value
    while(!bfs.empty()){
        unsigned int idx = bfs.front();
        bfs.pop();

        //return if cell of correct value is found
        if(map[idx] == val){
            result = idx;
            return true;
        }

        //iterate over all adjacent unvisited cells
        BOOST_FOREACH(unsigned nbr, nhood8(idx, costmap)){
            if(!visited_flag[nbr]){
                bfs.push(nbr);
                visited_flag[nbr] = true;
            }
        }
    }

    return false;
}


/**
 * @brief  Looks at cells around 'robot' and outputs the found free space cells
 * @param  robot current index of robot position
 * @param  size_of_field how many cells in all directions are considered
 * @param  costmap current costmap
 * @return list of free space cells
 */
std::vector<unsigned int> initialFreeSpace(unsigned int robot, int size_of_field, const costmap_2d::Costmap2D& costmap){
  std::vector<unsigned int> out;
  
  unsigned int size_x_ = costmap.getSizeInCellsX(), size_y_ = costmap.getSizeInCellsY();  
  unsigned char* map = costmap.getCharMap();  
  
  out.push_back(robot);
 
  for (int i = 1; i < size_of_field + 1; ++i){
    //if (map[robot - i - i * size_x_] == FREE_SPACE)
      out.push_back(robot - i - i * size_x_);
    //if (map[robot + i + i * size_x_] == FREE_SPACE)
      out.push_back(robot + i + i * size_x_);  
    for (int x = 1; x < 2*i+1; x++){
      //if (map[robot - i + x - i * size_x_] == FREE_SPACE)
        out.push_back(robot - i + x - i * size_x_);
      //if (map[robot - i - (i - x) * size_x_] == FREE_SPACE)
        out.push_back(robot - i - (i - x) * size_x_);
      if (x != 2*i) {
        //if (map[robot + i - x + i * size_x_] == FREE_SPACE)
          out.push_back(robot + i - x + i * size_x_); 
        //if (map[robot + i + (i - x) * size_x_] == FREE_SPACE)
          out.push_back(robot + i + (i - x) * size_x_); 
      }
    }    
  }    

  return out;
}


/**
 * @brief Determine cells on the edge of a circle around cell using midpoint circle algorithm
 * @param idx input cell index
 * @param radius defines size of circle in meters
 * @param costmap Reference to map data
 * @return cell indexes of cells on circle
 */
std::vector<unsigned int> circleCells(unsigned int idx, double radius, const costmap_2d::Costmap2D& costmap){

  std::vector<unsigned int> nnw, nne, een, ees, ssw, sse, wwn, wws;
  unsigned int size_x_ = costmap.getSizeInCellsX(), size_y_ = costmap.getSizeInCellsY();
  
  int cellRadius = radius / costmap.getResolution();
  
  // find x and y cell coordinates of circle center
  unsigned int x0,y0;
  costmap.indexToCells(idx, x0, y0);
  
  int x = cellRadius, y = 0,  dy = 1, dx = 1;
  int err = dx - (cellRadius << 1);

  if(idx >= x * size_x_ + x && idx + x * size_x_ + x < size_x_*(size_y_-1)) {
      while (x >= y) {
          // avoid adding the same point multiple times
          if (x == y || y == 0) {
              ees.push_back(idx + x + y * size_x_);           // EastEastSouth
              wwn.push_back(idx - x - y * size_x_);           // WestWestNorth
              ssw.push_back(idx - y + x * size_x_);           // SouthSouthWest
              nne.push_back(idx + y - x * size_x_);           // NorthNorthEast

          } else {
              een.insert(een.begin(), idx + x - y * size_x_); // EastEastNorth
              ees.push_back(idx + x + y * size_x_);           // EastEastSouth
              wwn.push_back(idx - x - y * size_x_);           // WestWestNorth
              wws.insert(wws.begin(), idx - x + y * size_x_); // WestWestSouth
              ssw.push_back(idx - y + x * size_x_);           // SouthSouthWest
              sse.insert(sse.begin(), idx + y + x * size_x_); // SouthSouthEast
              nnw.insert(nnw.begin(), idx - y - x * size_x_); // NorthNorthWest
              nne.push_back(idx + y - x * size_x_);           // NorthNorthEast
          }

          if (err <= 0){
              y += 1;
              err += dy;
              dy += 2;
          }

          if (err > 0 ){
              x -= 1;
              dx += 2;
              err += (-cellRadius << 1) + dx;
          }
      }
  } else {
      ROS_ERROR("At least one point of circle outside of map! Undefined behavior!");
  }
    
  // concatenate points counter clock wise
  std::vector<unsigned int> out(ees);
  out.insert( out.end(), sse.begin(), sse.end() );
  out.insert( out.end(), ssw.begin(), ssw.end() );
  out.insert( out.end(), wws.begin(), wws.end() );
  out.insert( out.end(), wwn.begin(), wwn.end() );
  out.insert( out.end(), nnw.begin(), nnw.end() );
  out.insert( out.end(), nne.begin(), nne.end() );
  out.insert( out.end(), een.begin(), een.end() );

  return out;
}

/**
 * @brief A certain number of consectutive points of a circle are selected with random starting point
 * @param circle list of points that form a circle
 * @param nofPoints number of point to select (at least 1)
 * @return list of points
 */
std::vector<unsigned int> selectConsecutivePointsOfCircleRandomly(std::vector<unsigned int> circle, int nofPoints){

        std::vector<unsigned int> out;
        std::random_device rd;
        std::mt19937 rng(rd());
        unsigned int sizeOfCircle = circle.size();

        std::uniform_int_distribution<unsigned int> uni(0, sizeOfCircle-1);

        unsigned int rand = uni(rng);

        out.push_back(circle[rand]);
        for (int i = 1; i < nofPoints; ++i){
            if (rand + i == sizeOfCircle) {
                rand = -i;
            }
            out.push_back(circle[rand + i]);
        }

        return out;
}

/**
 * @brief Determine how many consecutive pixels of a circle are needed to reach a certain angle
 * @param angle desired angle in degree
 * @param idx input cell index
 * @param radius defines size of circle in meters
 * @param costmap Reference to map data
 * @return number of pixels
 */
int angleToNumberOfPixels(double desired_angle, unsigned int idx, double radius, const costmap_2d::Costmap2D& costmap){

    std::vector<unsigned int> circle = circleCells(idx, radius,  costmap);

    unsigned int ix0, ix1, iy0, iy1, ix2, iy2;
    double x0, x1, y0, y1, x2, y2;
    double current_angle = 0.0;

    int number_of_pixels = 0;

    costmap.indexToCells(idx,ix2,iy2);
    costmap.mapToWorld(ix2,iy2,x2,y2);

    costmap.indexToCells(circle[0],ix0,iy0);
    costmap.mapToWorld(ix0,iy0,x0,y0);

    // compute vector between circle midpoint and first point of ring
    x0 = x0 - x2;
    y0 = y0 - y2;

    BOOST_FOREACH(unsigned point, circle) {
        costmap.indexToCells(point,ix1,iy1);
        costmap.mapToWorld(ix1,iy1,x1,y1);
        x1 = x1 - x2;
        y1 = y1 - y2;

        current_angle = acos((x0 * x1 + y0 * y1)/(sqrt(x0 * x0 + y0 * y0) * sqrt(x1 * x1 + y1 * y1))) * (180.0/3.14);

        //std::cout << "current_angle: " << current_angle << std::endl;

        if (current_angle > desired_angle) {
            break;
        }

        //std::cout << "Nof: " << number_of_pixels << std::endl;

        ++number_of_pixels;
    }

    return number_of_pixels;
}

std::vector<unsigned int> bresenham2D(unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a,
                        int offset_b, unsigned int offset, unsigned int max_length){
                        
  std::vector<unsigned int> out;
  unsigned int end = std::min(max_length, abs_da);
  for (unsigned int i = 0; i < end; ++i)
  {
    out.push_back(offset);
    offset += offset_a;
    error_b += abs_db;
    if ((unsigned int)error_b >= abs_da)
    {
      offset += offset_b;
      error_b -= abs_da;
    }
  }
  out.push_back(offset);
  return out;
}

inline int sign(int x)
{
  return x > 0 ? 1.0 : -1.0;
}


/**
 * @brief  Raytrace a line and apply some action at each step
 * @param  at The action to take... a functor
 * @param  x0 The starting x coordinate
 * @param  y0 The starting y coordinate
 * @param  x1 The ending x coordinate
 * @param  y1 The ending y coordinate
 * @param  max_length The maximum desired length of the segment... allows you to not go all the way to the endpoint
 */
std::vector<unsigned int> raytraceLine(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1, const costmap_2d::Costmap2D& costmap,
                             unsigned int max_length = UINT_MAX){
                             
  int dx = x1 - x0;
  int dy = y1 - y0;
  
  unsigned int size_x_ = costmap.getSizeInCellsX(), size_y_ = costmap.getSizeInCellsY();

  unsigned int abs_dx = abs(dx);
  unsigned int abs_dy = abs(dy);

  int offset_dx = sign(dx);
  int offset_dy = sign(dy) * size_x_;

  unsigned int offset = y0 * size_x_ + x0;

  // we need to chose how much to scale our dominant dimension, based on the maximum length of the line
  double dist = hypot(dx, dy);
  double scale = (dist == 0.0) ? 1.0 : std::min(1.0, max_length / dist);

  // if x is dominant
  if (abs_dx >= abs_dy)
  {
    int error_y = abs_dx / 2;
    return(bresenham2D(abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, (unsigned int)(scale * abs_dx)));
  }

  // otherwise y is dominant
  int error_x = abs_dy / 2;
  return(bresenham2D(abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, (unsigned int)(scale * abs_dy)));
}

bool pointInPolygon(double x, double y, const geometry_msgs::Polygon &polygon){
    int cross = 0;
    for (int i = 0, j = polygon.points.size()-1; i < polygon.points.size(); j = i++) {
        if ( ((polygon.points[i].y > y) != (polygon.points[j].y>y)) &&
            (x < (polygon.points[j].x-polygon.points[i].x) * (y-polygon.points[i].y) / (polygon.points[j].y-polygon.points[i].y) + polygon.points[i].x) ){
            cross++;
        }
    }
    return bool(cross % 2);
}

/**
* @brief Calculate the yaw of vector defined by origin and end points
* @param origin Origin point
* @param end End point
* @return Yaw angle of vector
*/
    double yawOfVector(double x1, double y1, double x2, double y2) {

        double delta_x, delta_y;
        delta_x = x2 - x1;
        delta_y = y2 - y1;

        double yaw = atan(delta_y / delta_x);

        if (delta_x < 0) {
            yaw = M_PI - yaw;
        }

        return yaw;
    }


}
#endif









