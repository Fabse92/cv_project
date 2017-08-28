#ifndef COSTMAP_TOOLS_H_
#define COSTMAP_TOOLS_H_

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Polygon.h>

#include <costmap_2d/costmap_2d.h>
#include <boost/foreach.hpp>
#include <ros/ros.h>


namespace frontier_exploration{

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
 * @brief Determine cells on the edge of a circle around cell using midpoint circle algorithm
 * @param idx input cell index
 * @param radius defines size of circle in meters
 * @param costmap Reference to map data
 * @return cell indexes of cells on circle
 */
std::vector<unsigned int> circleCells(unsigned int idx, double radius, const costmap_2d::Costmap2D& costmap){

  std::vector<unsigned int> out;
  unsigned int size_x_ = costmap.getSizeInCellsX(), size_y_ = costmap.getSizeInCellsY();
  
  int cellRadius = radius / costmap.getResolution();
  
  // find x and y cell coordinates of circle center
  unsigned int x0,y0;
  costmap.indexToCells(idx, x0, y0);
  
  int x = cellRadius, y = 0,  dy = 1, dx = 1;
  int err = dx - (cellRadius << 1);
  
  // TODO do not add same point multiple times 
  
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
    
  while (x >= y)
    {
      if(idx >= x * size_x_ + x && idx + x * size_x_ + x < size_x_*(size_y_-1)){
      
        out.push_back(idx + x - y * size_x_); // right above
        out.push_back(idx + x + y * size_x_); // right below
        
        out.push_back(idx - x - y * size_x_); // left above
        out.push_back(idx - x + y * size_x_); // left below
        
        out.push_back(idx - y + x * size_x_); // below left
        out.push_back(idx + y + x * size_x_); // below right
        
        out.push_back(idx - y - x * size_x_); // above left
        out.push_back(idx + y - x * size_x_); // above right
      
      } else{
        ROS_ERROR("At least one point of circle outside of map! Undefined behavior!");
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

  return out;
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

bool pointInPolygon(unsigned int x, unsigned int y, const geometry_msgs::Polygon &polygon){
    int cross = 0;
    for (int i = 0, j = polygon.points.size()-1; i < polygon.points.size(); j = i++) {
        if ( ((polygon.points[i].y > y) != (polygon.points[j].y>y)) &&
            (x < (polygon.points[j].x-polygon.points[i].x) * (y-polygon.points[i].y) / (polygon.points[j].y-polygon.points[i].y) + polygon.points[i].x) ){
            cross++;
        }
    }
    return bool(cross % 2);
}

}
#endif









