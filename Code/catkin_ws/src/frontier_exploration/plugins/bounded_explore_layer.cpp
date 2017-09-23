#include <frontier_exploration/bounded_explore_layer.h>

#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/footprint.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

#include <frontier_exploration/Frontier.h>
#include <frontier_exploration/UpdateBoundaryPolygon.h>
#include <frontier_exploration/GetNextFrontier.h>
#include <frontier_exploration/frontier_search.h>
#include <frontier_exploration/geometry_tools.h>

PLUGINLIB_EXPORT_CLASS(frontier_exploration::BoundedExploreLayer, costmap_2d::Layer)

double yaw(double x1, double y1, double x2, double y2) {

    double delta_x, delta_y;
    delta_x = x2 - x1;
    delta_y = y2 - y1;

    double yaw = atan(delta_y / delta_x);

    if (delta_x < 0) {
        yaw = M_PI - yaw;
    }

    return yaw;
}

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

namespace frontier_exploration
{

    using costmap_2d::LETHAL_OBSTACLE;
    using costmap_2d::NO_INFORMATION;
    using costmap_2d::FREE_SPACE;

    BoundedExploreLayer::BoundedExploreLayer(){}

    BoundedExploreLayer::~BoundedExploreLayer(){
        polygonService_.shutdown();
        frontierService_.shutdown();
        delete dsrv_;
        dsrv_ = 0;
    }

    void BoundedExploreLayer::onInitialize(){

        nh_ = ros::NodeHandle("~/" + name_);
        frontier_cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("frontiers",5);
        configured_ = false;
        marked_ = false;

        bool explore_clear_space;
        nh_.param("explore_clear_space", explore_clear_space, true);
        if(explore_clear_space){
            default_value_ = NO_INFORMATION;
        }else{
            default_value_ = FREE_SPACE;
        }

        matchSize();

        nh_.param<bool>("resize_to_boundary", resize_to_boundary_, true);
        nh_.param<std::string>("frontier_travel_point", frontier_travel_point_, "closest");
        nh_.param<std::string>("method", method_, "frontier");
        nh_.param<double>("circle_radius", circle_radius_, 3.0);
        nh_.param<double>("horizontal_fov", horizontal_fov_, 60.0);

        polygonService_ = nh_.advertiseService("update_boundary_polygon", &BoundedExploreLayer::updateBoundaryPolygonService, this);
        frontierService_ = nh_.advertiseService("get_next_frontier", &BoundedExploreLayer::getNextFrontierService, this);

        dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh_);
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
                    &BoundedExploreLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        prev_x = -1.0;
        prev_y = -1.0;

    }


    void BoundedExploreLayer::matchSize(){
        Costmap2D* master = layered_costmap_->getCostmap();
        resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
                  master->getOriginX(), master->getOriginY());
    }


    void BoundedExploreLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level){
        enabled_ = config.enabled;
    }

    bool BoundedExploreLayer::getNextFrontierService(frontier_exploration::GetNextFrontier::Request &req, frontier_exploration::GetNextFrontier::Response &res){
        return getNextFrontier(req.start_pose, res.next_frontier);
    }

    bool BoundedExploreLayer::getNextFrontier(geometry_msgs::PoseStamped start_pose, geometry_msgs::PoseStamped &next_frontier){

        //wait for costmap to get marked with boundary
        ros::Rate r(10);
        while(!marked_){
            ros::spinOnce();
            r.sleep();
        }
        
        if(start_pose.header.frame_id != layered_costmap_->getGlobalFrameID()){
            //error out if no transform available
            if(!tf_listener_.waitForTransform(layered_costmap_->getGlobalFrameID(), start_pose.header.frame_id,ros::Time::now(),ros::Duration(10))) {
                ROS_ERROR_STREAM("Couldn't transform from "<<layered_costmap_->getGlobalFrameID()<<" to "<< start_pose.header.frame_id);
                return false;
            }
            geometry_msgs::PoseStamped temp_pose = start_pose;
            tf_listener_.transformPose(layered_costmap_->getGlobalFrameID(),temp_pose,start_pose);
        }
        
        if (first_){
          first_ = false;
          unsigned int size_x_ = original_costmap_.getSizeInCellsX();
          unsigned int size_y_ = original_costmap_.getSizeInCellsY();
          std::vector<int> IoR_map(size_x_ * size_y_, 0);
          IoR_map_ = IoR_map;
        }
        
        bool toSwitch;
        nh_.param<bool>("/switch", toSwitch, false);

        //initialize frontier search implementation
        FrontierSearch frontierSearch(*(layered_costmap_->getCostmap()));
        if (method_ != "frontier" && (method_ != "frontier_plus" || toSwitch == true)){
          frontierSearch.setCostmap(original_costmap_);        
        }
        
        //get list of frontiers from search implementation
        std::list<Frontier> frontier_list = frontierSearch.searchFrom(start_pose.pose.position, method_, circle_radius_, horizontal_fov_, polygon_, nh_, IoR_map_);

        if(frontier_list.size() == 0){
            ROS_DEBUG("No frontiers found, exploration complete");
            return false;
        }

        //create placeholder for selected frontier
        Frontier selected;
        selected.min_distance = std::numeric_limits<double>::infinity();

        //pointcloud for visualization purposes
        pcl::PointCloud<pcl::PointXYZRGB> frontier_cloud_viz;
        pcl::PointXYZRGB frontier_point_viz(0,0,0);
        int max;
        int counter = 0;
        int red = 0;
        //double x, y;
        BOOST_FOREACH(Frontier frontier, frontier_list){
            ++counter;
            //load frontier into visualization poitncloud
            frontier_point_viz.x = frontier.initial.x;
            frontier_point_viz.y = frontier.initial.y;
            //x += frontier.initial.x;
            //y += frontier.initial.y;
            if (counter % 3 == 0){
              red += 2;
            }
            //frontier_point_viz.r = red;
            frontier_point_viz.r = 0;
            if (frontier.min_distance == 1) {
              frontier_point_viz.r = 255;
            }
            frontier_cloud_viz.push_back(frontier_point_viz);

            //check if this frontier is the nearest to robot
            if (frontier.min_distance < selected.min_distance){
                // check that this is not the same frontier as the previous one
                if (prev_x != frontier.initial.x && prev_y != frontier.initial.y){
                    selected = frontier;
                    max = frontier_cloud_viz.size()-1;
                }
            }
        }

        //x /= counter;
        //y /= counter;
        
        prev_x = selected.initial.x;
        prev_y = selected.initial.y;

        //color selected frontier
        frontier_cloud_viz[max].r = 0;
        frontier_cloud_viz[max].b = 255;

        //publish visualization point cloud
        sensor_msgs::PointCloud2 frontier_viz_output;
        pcl::toROSMsg(frontier_cloud_viz,frontier_viz_output);
        frontier_viz_output.header.frame_id = layered_costmap_->getGlobalFrameID();
        frontier_viz_output.header.stamp = ros::Time::now();
        frontier_cloud_pub.publish(frontier_viz_output);

        //set goal pose to next frontier
        next_frontier.header.frame_id = layered_costmap_->getGlobalFrameID();
        next_frontier.header.stamp = ros::Time::now();

        //
        if (method_ == "frontier" || (method_ == "frontier_plus" && toSwitch == false)) {
            if (frontier_travel_point_ == "closest") {
                next_frontier.pose.position = selected.initial;
            } else if (frontier_travel_point_ == "middle") {
                next_frontier.pose.position = selected.middle;
            } else if (frontier_travel_point_ == "centroid") {
                next_frontier.pose.position = selected.centroid;
            } else {
                ROS_ERROR("Invalid 'frontier_travel_point' parameter, falling back to 'closest'");
                next_frontier.pose.position = selected.initial;
            }
        } else {
            next_frontier.pose.position = selected.initial;
        }

        //next_frontier.pose.position = start_pose.pose.position;
        //next_frontier.pose.position.x = x;
        //next_frontier.pose.position.y = y;

        if (method_ == "frontier" || (method_ == "frontier_plus" && toSwitch == false)) {
            next_frontier.pose.orientation = tf::createQuaternionMsgFromYaw( yawOfVector(start_pose.pose.position, next_frontier.pose.position) );
        } else if (method_ == "random") {
            next_frontier.pose.orientation = tf::createQuaternionMsgFromYaw(fRand(-1.570796,1.570796));
        } else {
            next_frontier.pose.orientation = tf::createQuaternionMsgFromYaw( yaw(selected.initial.x, selected.initial.y, selected.middle.x, selected.middle.y) );
            std::cout << "Orientation: " << next_frontier.pose.orientation << std::endl;
        }

        return true;

    }

    bool BoundedExploreLayer::updateBoundaryPolygonService(frontier_exploration::UpdateBoundaryPolygon::Request &req, frontier_exploration::UpdateBoundaryPolygon::Response &res){

        return updateBoundaryPolygon(req.explore_boundary);

    }

    void BoundedExploreLayer::reset(){

        //reset costmap_ char array to default values
        marked_ = false;
        configured_ = false;
        memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));

    }

    bool BoundedExploreLayer::updateBoundaryPolygon(geometry_msgs::PolygonStamped polygon_stamped){

        //clear existing boundary, if any
        polygon_.points.clear();

        //error if no transform available between polygon and costmap
        if(!tf_listener_.waitForTransform(layered_costmap_->getGlobalFrameID(), polygon_stamped.header.frame_id,ros::Time::now(),ros::Duration(10))) {
            ROS_ERROR_STREAM("Couldn't transform from "<<layered_costmap_->getGlobalFrameID()<<" to "<< polygon_stamped.header.frame_id);
            return false;
        }

        //Transform all points of boundary polygon into costmap frame
        geometry_msgs::PointStamped in, out;
        in.header = polygon_stamped.header;
        BOOST_FOREACH(geometry_msgs::Point32 point32, polygon_stamped.polygon.points){
            in.point = costmap_2d::toPoint(point32);
            tf_listener_.transformPoint(layered_costmap_->getGlobalFrameID(),in,out);
            polygon_.points.push_back(costmap_2d::toPoint32(out.point));
        }

        //if empty boundary provided, set to whole map
        if(polygon_.points.empty()){
            geometry_msgs::Point32 temp;
            temp.x = getOriginX();
            temp.y = getOriginY();
            polygon_.points.push_back(temp);
            temp.y = getSizeInMetersY();
            polygon_.points.push_back(temp);
            temp.x = getSizeInMetersX();
            polygon_.points.push_back(temp);
            temp.y = getOriginY();
            polygon_.points.push_back(temp);
        }

        if(resize_to_boundary_){
            updateOrigin(0,0);

            //Find map size and origin by finding min/max points of polygon
            double min_x = std::numeric_limits<double>::infinity();
            double min_y = std::numeric_limits<double>::infinity();
            double max_x = -std::numeric_limits<double>::infinity();
            double max_y = -std::numeric_limits<double>::infinity();

            BOOST_FOREACH(geometry_msgs::Point32 point, polygon_.points){
                min_x = std::min(min_x,(double)point.x);
                min_y = std::min(min_y,(double)point.y);
                max_x = std::max(max_x,(double)point.x);
                max_y = std::max(max_y,(double)point.y);
            }

            //resize the costmap to polygon boundaries, don't change resolution
            int size_x, size_y;
            worldToMapNoBounds(max_x - min_x, max_y - min_y, size_x, size_y);
            layered_costmap_->resizeMap(size_x, size_y, layered_costmap_->getCostmap()->getResolution(), min_x, min_y);
            matchSize();
        }

        configured_ = true;
        marked_ = false;
        return true;
    }


    void BoundedExploreLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y){

        //check if layer is enabled and configured with a boundary
        if (!enabled_ || !configured_){ return; }

        //update the whole costmap
        *min_x = getOriginX();
        *min_y = getOriginY();
        *max_x = getSizeInMetersX()+getOriginX();
        *max_y = getSizeInMetersY()+getOriginY();

    }

    void BoundedExploreLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
        original_costmap_ = Costmap2D(master_grid);
        //check if layer is enabled and configured with a boundary
        if (!enabled_ || !configured_){ return; }

        //draw lines between each point in polygon
        MarkCell marker(costmap_, LETHAL_OBSTACLE);

        //circular iterator
        for(int i = 0, j = polygon_.points.size()-1; i < polygon_.points.size(); j = i++){

            int x_1, y_1, x_2, y_2;
            worldToMapEnforceBounds(polygon_.points[i].x, polygon_.points[i].y, x_1,y_1);
            worldToMapEnforceBounds(polygon_.points[j].x, polygon_.points[j].y, x_2,y_2);

            raytraceLine(marker,x_1,y_1,x_2,y_2);
        }
        //update the master grid from the internal costmap
        mapUpdateKeepObstacles(master_grid, min_i, min_j, max_i, max_j);


    }

    void BoundedExploreLayer::mapUpdateKeepObstacles(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
        if (!enabled_)
            return;

        unsigned char* master = master_grid.getCharMap();
        unsigned int span = master_grid.getSizeInCellsX();

        for (int j = min_j; j < max_j; j++)
        {
            unsigned int it = span*j+min_i;
            for (int i = min_i; i < max_i; i++)
            {
                //only update master grid if local costmap cell is lethal/higher value, and is not overwriting a lethal obstacle in the master grid
                if(master[it] != LETHAL_OBSTACLE && (costmap_[it] == LETHAL_OBSTACLE || costmap_[it] > master[it])){
                    master[it] = costmap_[it];
                }
                it++;
            }
        }
        marked_ = true;
    }
}
