#include <frontier_exploration/frontier_search.h>

#include <costmap_2d/costmap_2d.h>
#include<costmap_2d/cost_values.h>
#include <geometry_msgs/Point.h>
#include <boost/foreach.hpp>

#include <frontier_exploration/costmap_tools.h>
#include <frontier_exploration/Frontier.h>
#include <math.h>   /* exp */

namespace frontier_exploration{

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

FrontierSearch::FrontierSearch(costmap_2d::Costmap2D &costmap) : costmap_(costmap) { }

void FrontierSearch::setCostmap(costmap_2d::Costmap2D& costmap){
  costmap_ = costmap_2d::Costmap2D(costmap);
}

std::list<Frontier> FrontierSearch::searchFrom(geometry_msgs::Point position, std::string method, double circle_radius, geometry_msgs::Polygon polygon){

    std::list<Frontier> frontier_list;

    //Sanity check that robot is inside costmap bounds before searching
    unsigned int mx,my;
    if (!costmap_.worldToMap(position.x,position.y,mx,my)){
        ROS_ERROR("Robot out of costmap bounds, cannot search for frontiers");
        return frontier_list;
    }

    //make sure map is consistent and locked for duration of search
    boost::unique_lock < costmap_2d::Costmap2D::mutex_t > lock(*(costmap_.getMutex()));

    map_ = costmap_.getCharMap();
    size_x_ = costmap_.getSizeInCellsX();
    size_y_ = costmap_.getSizeInCellsY();

    //initialize flag arrays to keep track of visited and frontier cells
    std::vector<bool> frontier_flag(size_x_ * size_y_, false);
    std::vector<bool> visited_flag(size_x_ * size_y_, false);

    //initialize breadth first search
    std::queue<unsigned int> bfs;

    //find closest clear cell to start search
    unsigned int clear, pos = costmap_.getIndex(mx,my);
    if(nearestCell(clear, pos, FREE_SPACE, costmap_)){
        bfs.push(clear);
    }else{
        bfs.push(pos);
        ROS_WARN("Could not find nearby clear cell to start search");
    }
    visited_flag[bfs.front()] = true;
    std::vector<bool> processed_flag(size_x_ * size_y_, false);

    int counter = 0, counter2 = 0;
    
    while(!bfs.empty()){
        unsigned int idx = bfs.front();
        bfs.pop();

        //iterate over 4-connected neighbourhood
        BOOST_FOREACH(unsigned nbr, nhood4(idx, costmap_)){
        
            // distinguish between different methods how to find and define 'frontiers'
            // 'frontier' is the original method of the package            
            // the size of a frontier defines the number of connected cells on the border between discovered free cells and undiscovered cells   
            if(method == "frontier"){
              //add to queue all free, unvisited cells, use descending search in case initialized on non-free cell
              if(map_[nbr] <= map_[idx] && !visited_flag[nbr]){
                  visited_flag[nbr] = true;
                  bfs.push(nbr);
                    //check if cell is new frontier cell (unvisited, NO_INFORMATION, free neighbour)
              }else if(isNewFrontierCell(nbr, frontier_flag)){
                  frontier_flag[nbr] = true;
                  Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag);
                  if(new_frontier.size > 1){
                      frontier_list.push_back(new_frontier);
                  }
              }
              
              // 'information_gain' is a new method
              // it defines 'frontiers' as possible next poses on a free cell
              // frontiers might be randomly sampled from all possible reachable free cells
            }else if (method == "information_gain"){
              std::vector<unsigned int> circle;
              unsigned int ix, iy;
              double x, y;
              costmap_.indexToCells(nbr,ix,iy);
              costmap_.mapToWorld(ix,iy,x,y);
              if (!visited_flag[nbr]) {
                visited_flag[nbr] = true;
                if(map_[nbr] == FREE_SPACE && pointInPolygon(x,y,polygon)){ 
                  ++counter;                                        
                  unsigned int x0, y0, x1, y1;
                  costmap_.indexToCells(idx,x0,y0);
                  std::cout << counter << std::endl;
                  BOOST_FOREACH(unsigned point, circleCells(nbr, circle_radius, costmap_)){ 
                    bfs.push(nbr); 
                    if (counter == 10){                      
                      //counter = 0;
                      costmap_.indexToCells(nbr,x0,y0); 
                      costmap_.indexToCells(point,x1,y1);                 
                      //BOOST_FOREACH(unsigned point2, raytraceLine(x0,y0,x1,y1,costmap_)){
                      //  if(map_[point2] == LETHAL_OBSTACLE){   
                      //    break;
                      //  }
                        if (!processed_flag[point]){
                          processed_flag[point] = true;
                          ++counter2;
                          if (counter2 < 70 && counter2 > 10){                           
                            Frontier new_frontier = buildSimpleFrontier(point, 1.0);
                            frontier_list.push_back(new_frontier);         
                          }
                        }  
                      //}         
                    }
                  }
                }
                  /*
                  circle = circleCells(nbr, circle_radius, costmap_);
                  std::vector<bool> processed_flag(size_x_ * size_y_, false);
                  Frontier new_frontier = buildInformationGainFrontier(pos, nbr, circle, processed_flag);
                  frontier_list.push_back(new_frontier);                  
                  bfs.push(nbr);
                  std::cout << "one frontier computed" << std::endl;                  
                }*/
              }
           } 
        }
    }  
    std::cout << "ALL FRONTIERS COMPUTED" << std::endl;
    return frontier_list;

}

Frontier FrontierSearch::buildInformationGainFrontier(unsigned int robot_position, unsigned int cell, std::vector<unsigned int> circle, std::vector<bool> processed_flag){
  
  Frontier output;      
  output.size = 1;
  
  unsigned char previous_point = map_[cell];
  unsigned int x0, y0, x1, y1, ix, iy;
  double robot_x, robot_y, cell_x, cell_y;
  
  costmap_.indexToCells(cell,x0,y0); 
  costmap_.mapToWorld(x0,y0,cell_x,cell_y);
  
  costmap_.indexToCells(robot_position,ix,iy);
  costmap_.mapToWorld(ix,iy,robot_x,robot_y);
  
  double distance = sqrt(pow((cell_x-robot_x),2.0) + pow((cell_y-robot_y),2.0));
  double inf_gain = 0.0;
  double c1 = 0.05;

  //record initial contact point for frontier
  output.initial.x = cell_x;
  output.initial.y = cell_y;
  output.centroid.x = cell_x;
  output.centroid.y = cell_y;
  output.middle.x = cell_x;
  output.middle.y = cell_y;
  
  BOOST_FOREACH(unsigned circle_point, circle){
    costmap_.indexToCells(circle_point,x1,y1);
    BOOST_FOREACH(unsigned line_point, raytraceLine(x0,y0,x1,y1,costmap_)){
      if(map_[line_point] == LETHAL_OBSTACLE){ 
        if(processed_flag[line_point] == false){
          if(previous_point == NO_INFORMATION){
            inf_gain += 10; // bonus value for unknown pixels infront of obstacle
          } else {
            inf_gain += 2;  // obstacle bonus
          }
          processed_flag[line_point] = true;
        }        
        break;
      } else if(processed_flag[line_point] == false){
        if(map_[line_point] == NO_INFORMATION){
          inf_gain += 1;    // value of a pixel with unknown value
        }
      }
      previous_point = map_[line_point];   
    }
  }
  
  // TODO use orientation difference 
  // the frontier with smalles min_distance will be selected as next frontier,
  // therefore negated information gain is weighted with the distance from robot to the pose
  output.min_distance = -inf_gain * exp(-c1 * distance);
  
  std::cout << "Information gain: " << inf_gain << ", Distance: " << distance << ", Value: " << output.min_distance << std::endl; 
  
  return output;
}

Frontier FrontierSearch::buildSimpleFrontier(unsigned int cell, double dist){
  //initialize frontier structure
  Frontier output;
  output.centroid.x = 0;
  output.centroid.y = 0;
  output.size = 1;
  output.min_distance = dist;

  //record initial contact point for frontier
  unsigned int ix, iy;
  costmap_.indexToCells(cell,ix,iy);
  costmap_.mapToWorld(ix,iy,output.initial.x,output.initial.y);

  return output;
}


Frontier FrontierSearch::buildNewFrontier(unsigned int initial_cell, unsigned int reference, std::vector<bool>& frontier_flag){

    //initialize frontier structure
    Frontier output;
    output.centroid.x = 0;
    output.centroid.y = 0;
    output.size = 1;
    output.min_distance = std::numeric_limits<double>::infinity();

    //record initial contact point for frontier
    unsigned int ix, iy;
    costmap_.indexToCells(initial_cell,ix,iy);
    costmap_.mapToWorld(ix,iy,output.initial.x,output.initial.y);

    //push initial gridcell onto queue
    std::queue<unsigned int> bfs;
    bfs.push(initial_cell);

    //cache reference position in world coords
    unsigned int rx,ry;
    double reference_x, reference_y;
    costmap_.indexToCells(reference,rx,ry);
    costmap_.mapToWorld(rx,ry,reference_x,reference_y);

    while(!bfs.empty()){
        unsigned int idx = bfs.front();
        bfs.pop();

        //try adding cells in 8-connected neighborhood to frontier
        BOOST_FOREACH(unsigned int nbr, nhood8(idx, costmap_)){
            //check if neighbour is a potential frontier cell
            if(isNewFrontierCell(nbr,frontier_flag)){

                //mark cell as frontier
                frontier_flag[nbr] = true;
                unsigned int mx,my;
                double wx,wy;
                costmap_.indexToCells(nbr,mx,my);
                costmap_.mapToWorld(mx,my,wx,wy);

                //update frontier size
                output.size++;

                //update centroid of frontier
                output.centroid.x += wx;
                output.centroid.y += wy;

                //determine frontier's distance from robot, going by closest gridcell to robot
                double distance = sqrt(pow((double(reference_x)-double(wx)),2.0) + pow((double(reference_y)-double(wy)),2.0));
                if(distance < output.min_distance){
                    output.min_distance = distance;
                    output.middle.x = wx;
                    output.middle.y = wy;
                }

                //add to queue for breadth first search
                bfs.push(nbr);
            }
        }
    }

    //average out frontier centroid
    output.centroid.x /= output.size;
    output.centroid.y /= output.size;
    return output;
}

bool FrontierSearch::isNewFrontierCell(unsigned int idx, const std::vector<bool>& frontier_flag){

    //check that cell is unknown and not already marked as frontier
    if(map_[idx] != NO_INFORMATION || frontier_flag[idx]){
        return false;
    }

    //frontier cells should have at least one cell in 4-connected neighbourhood that is free
    BOOST_FOREACH(unsigned int nbr, nhood4(idx, costmap_)){
        if(map_[nbr] == FREE_SPACE){
            return true;
        }
    }

    return false;

}

}
