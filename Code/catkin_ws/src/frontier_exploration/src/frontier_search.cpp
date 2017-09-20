#include <frontier_exploration/frontier_search.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>
#include <geometry_msgs/Point.h>
#include <boost/foreach.hpp>

#include <frontier_exploration/RequestLabelCertainties.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>

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
  real_costmap_ = costmap_;
  costmap_ = costmap_2d::Costmap2D(costmap);
}

std::list<Frontier> FrontierSearch::searchFrom(geometry_msgs::Point position, std::string method, double circle_radius, double horizontal_fov, geometry_msgs::Polygon polygon, ros::NodeHandle nh, std::vector<int> &IoR_map){

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
    
    unsigned int ix, iy, ix0, iy0;
   
    if(method != "frontier"){   
      costmap_.worldToMap(0.0, 0.0, ix0, iy0);
      BOOST_FOREACH(unsigned int point, initialFreeSpace(costmap_.getIndex(ix0,iy0), 3, costmap_)){ 
        costmap_.indexToCells(point,ix,iy);
        costmap_.setCost(ix,iy,FREE_SPACE);
      }
    } 
        
    if(method == "information_gain_with_candidates"){   
       
        ros::ServiceClient request_client = nh.serviceClient<frontier_exploration::RequestLabelCertainties>("/octomap_server/certainty_occupancy_grid"); 
        request_client.waitForExistence();
        
        frontier_exploration::RequestLabelCertainties request_srv;
        
        unsigned int mx, my; 

        ROS_INFO("Requesting for label certainty");
        if (request_client.call(request_srv))
        {
          ROS_INFO("Received label certainty");
          
          for (unsigned int i = 0; i < request_srv.response.certainties.size(); ++i){
            double wx = (double)request_srv.response.X[i].data;
            double wy = (double)request_srv.response.Y[i].data;
            std_msgs::UInt8 certainty_msg = request_srv.response.certainties[i]; 
            unsigned int certainty = certainty_msg.data;        
            
            //if (certainty != LETHAL_OBSTACLE and certainty != NO_INFORMATION and certainty != FREE_SPACE and certainty != INSCRIBED_INFLATED_OBSTACLE){
            if (certainty > 0){
              //ROS_INFO_STREAM("(" << wx << ", " << wy << "), Certainty: " << (unsigned int) certainty);
              costmap_.worldToMap(wx, wy, mx, my);
              costmap_.setCost(mx,my,certainty);
              
              //Frontier new_frontier = buildSimpleFrontier(costmap_.getIndex(mx,my), 1.0);
              //frontier_list.push_back(new_frontier);
            }
          }
          
        }
        else
        {
          ROS_ERROR("Failed to provide label certainty");
        }
        
    }

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
    int pixelsOfAngle = angleToNumberOfPixels(horizontal_fov,pos,circle_radius,costmap_);
    
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
            }else if (method == "random"){
              if (!visited_flag[nbr]) {
                visited_flag[nbr] = true;    
                double x, y;            
                costmap_.indexToCells(nbr,ix,iy);
                costmap_.mapToWorld(ix,iy,x,y);
                if(map_[nbr] == FREE_SPACE && pointInPolygon(x,y,polygon)){ 
                  bfs.push(nbr);  
                  std::random_device rd;
                  std::mt19937 rng(rd());
                  std::uniform_int_distribution<int> uni(-100000,100000);
                  int rand = uni(rng);
                  Frontier new_frontier = buildSimpleFrontier(nbr, (double) rand);
                  frontier_list.push_back(new_frontier);
                }
              }
            // 'information_gain' is a new method
            // it defines 'frontiers' as possible next poses on a free cell
            // frontiers might be randomly sampled from all possible reachable free cells
            }else{
              std::vector<unsigned int> circle_part;
              if (!visited_flag[nbr]) {
                visited_flag[nbr] = true;    
                double x, y;            
                costmap_.indexToCells(nbr,ix,iy);
                costmap_.mapToWorld(ix,iy,x,y);
                if(map_[nbr] == FREE_SPACE && pointInPolygon(x,y,polygon)){ 
                    /* //bfs.push(nbr);
                    BOOST_FOREACH(unsigned point, initialFreeSpace(pos, 4, costmap_)){                    
                      Frontier new_frontier = buildSimpleFrontier(point, 1.0);
                      frontier_list.push_back(new_frontier);
                    }
                    /*++counter;
                    unsigned int x0, y0, x1, y1;
                    costmap_.indexToCells(idx,x0,y0);
                    //std::cout << counter << std::endl;
                    bfs.push(nbr);
                    BOOST_FOREACH(unsigned point, selectConsecutivePointsOfCircleRandomly(circleCells(nbr, circle_radius, costmap_), pixelsOfAngle)){
                    //BOOST_FOREACH(unsigned point, circleCells(nbr, circle_radius, costmap_)){
                        if (counter == 1){
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
                              if (counter2 < 350 && counter2 > 0){
                                Frontier new_frontier = buildSimpleFrontier(point, 1.0);
                                frontier_list.push_back(new_frontier);
                              }
                            } else {
                                std::cout << "NANU NANU" << std::endl;
                            }
                      //}         
                        }
                    }
                } */
                  circle_part = selectConsecutivePointsOfCircleRandomly(circleCells(nbr, circle_radius, costmap_), pixelsOfAngle);
                  std::vector<bool> processed_flag(size_x_ * size_y_, false);
                  Frontier new_frontier = buildInformationGainFrontier(pos, nbr, circle_part, processed_flag, polygon, method, nh, IoR_map);
                  if (new_frontier.min_distance != 1)
                    frontier_list.push_back(new_frontier);                  
                  bfs.push(nbr);
                  //std::cout << "one frontier computed" << std::endl;
                }
              }
           } 
        }
    }
    
    if(method != "random" && method != "frontier"){ 
      BOOST_FOREACH(unsigned obstacle, obstacle_list_){
        IoR_map[obstacle] += 4;      
      }
      
      BOOST_FOREACH(unsigned cell, IoR_map){
        if (IoR_map[cell] > 0)
          IoR_map[cell] -= 1;
      }
      
      std::cout << "ALL FRONTIERS COMPUTED" << std::endl;
    }
    
    return frontier_list;

}

Frontier FrontierSearch::buildInformationGainFrontier(unsigned int robot_position, unsigned int cell, std::vector<unsigned int> points, std::vector<bool> processed_flag, geometry_msgs::Polygon polygon, std::string method, ros::NodeHandle nh, std::vector<int> &IoR_map){
  
  Frontier output;      
  output.size = 1;

  std::vector<unsigned int> obstacle_list;
  
  unsigned char previous_point = map_[cell];
  unsigned int x0, y0, x1, y1, ix, iy;
  double robot_x, robot_y, cell_x, cell_y, wx, wy, x, y;
  
  costmap_.indexToCells(cell,x0,y0); 
  costmap_.mapToWorld(x0,y0,cell_x,cell_y);
  
  costmap_.indexToCells(robot_position,ix,iy);
  costmap_.mapToWorld(ix,iy,robot_x,robot_y);
  
  double distance = sqrt(pow((cell_x-robot_x),2.0) + pow((cell_y-robot_y),2.0));
  
  if (distance > 2 || distance < 0.2) {
    output.min_distance = 1;
    return output;
  }
  
  double inf_obstacle, inf_obstacle_unexplored, inf_object, inf_object_unexplored, inf_unexplored; 
  nh.param<double>("/explore_server/inf_obstacle", inf_obstacle, -1.0);
  nh.param<double>("/explore_server/inf_obstacle_unexplored", inf_obstacle_unexplored, -1.0);
  nh.param<double>("/explore_server/inf_object", inf_object, -1.0);
  nh.param<double>("/explore_server/inf_object_unexplored", inf_object_unexplored, -1.0);
  nh.param<double>("/explore_server/inf_unexplored", inf_unexplored, -1.0);
  
  if (inf_obstacle == -1.0 || inf_obstacle_unexplored == -1.0 || inf_object == -1.0 || inf_object_unexplored == -1.0 || inf_unexplored == -1.0){
    ROS_WARN("Information gain parameter not set!");
  }
  
  double inf_gain = 0.0;
  double c1 = 0.05;

  //record initial contact point for frontier
  output.initial.x = cell_x;
  output.initial.y = cell_y;
  output.centroid.x = cell_x;
  output.centroid.y = cell_y;
  output.middle.x = 0;
  output.middle.y = 0;
  
  BOOST_FOREACH(unsigned point, points){
    costmap_.indexToCells(point,x1,y1);
    costmap_.mapToWorld(x1,y1,wx,wy);
    output.middle.x += wx;
    output.middle.y += wy;
    BOOST_FOREACH(unsigned line_point, raytraceLine(x0,y0,x1,y1,costmap_)){           
      costmap_.indexToCells(line_point,ix,iy);
      costmap_.mapToWorld(ix,iy,x,y);
      if(pointInPolygon(x,y,polygon)){
        if(map_[line_point] == LETHAL_OBSTACLE || (map_[line_point] > 0 && map_[line_point] < 253)){
          if(processed_flag[line_point] == false){
            if(previous_point == NO_INFORMATION){
              if(map_[line_point] == LETHAL_OBSTACLE){
                inf_gain += inf_obstacle_unexplored; // bonus value for unknown pixels infront of obstacle
              } else {
                inf_gain += inf_object_unexplored; // bonus value for unknown pixels infront of object candidate
              }
            }
            if(map_[line_point] == LETHAL_OBSTACLE){
              if (IoR_map[line_point] < 6) // is this obstacle not inhibited?
                inf_gain += inf_obstacle;  // obstacle bonus
              obstacle_list.push_back(line_point);
            } else {
              inf_gain += inf_object * std::pow(2.0,-(map_[line_point]-1));  // object candidate bonus
            }
            processed_flag[line_point] = true;
          }
          break;
        } else if(processed_flag[line_point] == false){
          if(map_[line_point] == NO_INFORMATION){
            inf_gain += inf_unexplored;    // value of a pixel with unknown value
          }
          processed_flag[line_point] = true;
        }
        previous_point = map_[line_point];
      }
    }
  }

  // compute the average position of all considered points
  output.middle.x /= points.size();
  output.middle.y /= points.size();

  // TODO use orientation difference !?
  //double orientation_distance = yawOfVector(robot_x, robot_y, output.middle.x, output.middle.y);


  // the frontier with smalles min_distance will be selected as next frontier,
  // therefore negated information gain is weighted with the distance from robot to the pose

  output.min_distance = -inf_gain * exp(-c1 * distance);
  
  if (output.min_distance < min_dist_){
    min_dist_ = output.min_distance;
    obstacle_list_ = obstacle_list;
  }
  
  ROS_DEBUG_STREAM("Information gain: " << inf_gain << ", Distance: " << distance << ", Value: " << output.min_distance); 
  
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
