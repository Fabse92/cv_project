#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <geometry_msgs/PolygonStamped.h>

#include <frontier_exploration/ExploreTaskAction.h>
#include <frontier_exploration/GetNextFrontier.h>
#include <frontier_exploration/UpdateBoundaryPolygon.h>

#include <tf/transform_listener.h>

#include <move_base_msgs/MoveBaseAction.h>

#include <frontier_exploration/geometry_tools.h>


#include "object_candidates/Snapshot.h"
#include "evaluation/Evaluate.h"

#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include "object_candidates/ArrayImages.h"

#include "candidate_locator/LocateCandidates.h"
#include "candidate_locator/ArrayPointClouds.h"

#include "octomap_ros/conversions.h"
#include "octomap_msgs/MergeCandidates.h"
#include "octomap_msgs/GetOctomap.h"

namespace frontier_exploration{

/**
 * @brief Server for frontier exploration action, runs the state machine associated with a
 * structured frontier exploration task and manages robot movement through move_base.
 */
class FrontierExplorationServer
{

public:

    double starting_time, available_time;
    std::string method;
    /**
     * @brief Constructor for the server, sets up this node's ActionServer for exploration and ActionClient to move_base for robot movement.
     * @param name Name for SimpleActionServer
     */
    FrontierExplorationServer(std::string name) :
        tf_listener_(ros::Duration(10.0)),
        private_nh_("~"),
        as_(nh_, name, boost::bind(&FrontierExplorationServer::executeCb, this, _1), false),
        move_client_("move_base",true),
        retry_(5)
    {
        private_nh_.param<double>("frequency", frequency_, 0.0);
        private_nh_.param<double>("available_time", available_time, 60.0);
        ROS_INFO_STREAM("Remaining time " << available_time);
        private_nh_.param<double>("goal_aliasing", goal_aliasing_, 0.1);
        private_nh_.param<std::string>("method", method, "frontier");

        explore_costmap_ros_ = boost::shared_ptr<costmap_2d::Costmap2DROS>(new costmap_2d::Costmap2DROS("explore_costmap", tf_listener_));

        as_.registerPreemptCallback(boost::bind(&FrontierExplorationServer::preemptCb, this));
        starting_time = ros::Time::now().toSec();
        as_.start();        
    }

private:
    
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    tf::TransformListener tf_listener_;
    actionlib::SimpleActionServer<frontier_exploration::ExploreTaskAction> as_;

    boost::shared_ptr<costmap_2d::Costmap2DROS> explore_costmap_ros_;
    double frequency_, goal_aliasing_;
    bool success_, moving_;
    int retry_;

    boost::mutex move_client_lock_;
    frontier_exploration::ExploreTaskFeedback feedback_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_client_;
    move_base_msgs::MoveBaseGoal move_client_goal_;
    
    ros::ServiceClient evaluation_client = nh_.serviceClient<evaluation::Evaluate>("evaluate");

    void finishedTask(bool restart){
      evaluation::Evaluate eval_srv;
      eval_srv.request.restart.data = restart;
      ROS_INFO("Requesting evaluation");
      if (evaluation_client.call(eval_srv))
      {
        ROS_INFO_STREAM("Evaluation performed!");
      }
      if (restart){
        ROS_INFO_STREAM("Restarting system after evaluation attempt");
        as_.setSucceeded();
        boost::unique_lock<boost::mutex> lock(move_client_lock_);
        move_client_.cancelGoalsAtAndBeforeTime(ros::Time::now());
      }
    }

    /**
     * @brief Execute callback for actionserver, run after accepting a new goal
     * @param goal ActionGoal containing boundary of area to explore, and a valid centerpoint for the area.
     */

    void executeCb(const frontier_exploration::ExploreTaskGoalConstPtr &goal)
    {
        success_ = false;
        moving_ = false;

        double delay_time;
        private_nh_.param<double>("exploration_delay_time", delay_time, 0.0);
        ROS_INFO_STREAM(delay_time);

        explore_costmap_ros_->resetLayers();

        //create costmap services
        ros::ServiceClient updateBoundaryPolygon = private_nh_.serviceClient<frontier_exploration::UpdateBoundaryPolygon>("explore_costmap/explore_boundary/update_boundary_polygon");
        ros::ServiceClient getNextFrontier = private_nh_.serviceClient<frontier_exploration::GetNextFrontier>("explore_costmap/explore_boundary/get_next_frontier");

        //wait for move_base and costmap services
        if(!move_client_.waitForServer() || !updateBoundaryPolygon.waitForExistence() || !getNextFrontier.waitForExistence()){
            as_.setAborted();
            return;
        }

        //set region boundary on costmap
        if(ros::ok() && as_.isActive()){
            frontier_exploration::UpdateBoundaryPolygon srv;
            srv.request.explore_boundary = goal->explore_boundary;
            if(updateBoundaryPolygon.call(srv)){
                ROS_INFO("Region boundary set");
            }else{
                ROS_ERROR("Failed to set region boundary");
                as_.setAborted();
                return;
            }
        }

        //loop until all frontiers are explored
        ros::Rate rate(frequency_);
        while(ros::ok() && as_.isActive()){
            ROS_INFO("next iteration");

            frontier_exploration::GetNextFrontier srv;

            //placeholder for next goal to be sent to move base
            geometry_msgs::PoseStamped goal_pose;

            //get current robot pose in frame of exploration boundary
            tf::Stamped<tf::Pose> robot_pose;
            explore_costmap_ros_->getRobotPose(robot_pose);

            //provide current robot pose to the frontier search service request
            tf::poseStampedTFToMsg(robot_pose,srv.request.start_pose);

            //evaluate if robot is within exploration boundary using robot_pose in boundary frame
            geometry_msgs::PoseStamped eval_pose = srv.request.start_pose;
            if(eval_pose.header.frame_id != goal->explore_boundary.header.frame_id){
                tf_listener_.transformPose(goal->explore_boundary.header.frame_id, srv.request.start_pose, eval_pose);
            }

            //check if robot is not within exploration boundary and needs to return to center of search area
            if(goal->explore_boundary.polygon.points.size() > 0 && !pointInPolygon(eval_pose.pose.position,goal->explore_boundary.polygon)){
                
                //check if robot has explored at least one frontier, and promote debug message to warning
                if(success_){
                    ROS_WARN("Robot left exploration boundary, returning to center");
                }else{
                    ROS_DEBUG("Robot not initially in exploration boundary, traveling to center");
                }
                //get current robot position in frame of exploration center
                geometry_msgs::PointStamped eval_point;
                eval_point.header = eval_pose.header;
                eval_point.point = eval_pose.pose.position;
                if(eval_point.header.frame_id != goal->explore_center.header.frame_id){
                    geometry_msgs::PointStamped temp = eval_point;
                    tf_listener_.transformPoint(goal->explore_center.header.frame_id, temp, eval_point);
                }

                //set goal pose to exploration center
                goal_pose.header = goal->explore_center.header;
                goal_pose.pose.position = goal->explore_center.point;
                goal_pose.pose.orientation = tf::createQuaternionMsgFromYaw( yawOfVector(eval_point.point, goal->explore_center.point) );

            }else if(getNextFrontier.call(srv)){ //if in boundary, try to find next frontier to search

                ROS_INFO("Found frontier to explore");
                success_ = true;
                goal_pose = feedback_.next_frontier = srv.response.next_frontier;
                retry_ = 5;

            }else{ //if no frontier found, check if search is successful
                ROS_DEBUG("Couldn't find a frontier");

                //search is succesful
                if(retry_ == 0 && success_){
                  if (method == "frontier") {
                    ROS_WARN("Finished exploring room");
                    finishedTask(true);
                    return;
                  } else if (method == "frontier_plus"){
                    ROS_WARN("Finished exploring room .. switching to information gain method");
                    nh_.setParam("switch", true);
                  }                    

                }else if(retry_ == 0 || !ros::ok()){ //search is not successful

                    ROS_ERROR("Failed exploration");
                    as_.setAborted();
                    return;
                }

                ROS_DEBUG("Retrying...");
                retry_--;
                //try to find frontier again, without moving robot
                continue;
            }
            //if above conditional does not escape this loop step, search has a valid goal_pose

            //check if new goal is close to old goal, hence no need to resend
            if(!moving_ || !pointsNearby(move_client_goal_.target_pose.pose.position,goal_pose.pose.position,goal_aliasing_*0.5)){
                ROS_DEBUG("New exploration goal");
                move_client_goal_.target_pose = goal_pose;
                boost::unique_lock<boost::mutex> lock(move_client_lock_);
                if(as_.isActive()){
                    move_client_.sendGoal(move_client_goal_, boost::bind(&FrontierExplorationServer::doneMovingCb, this, _1, _2),0,boost::bind(&FrontierExplorationServer::feedbackMovingCb, this, _1));
                    moving_ = true;
                }
                lock.unlock();
            } else {
                ROS_WARN("New goal too close to previous goal");
            }

            //check if continuous goal updating is enabled
            if(frequency_ > 0){
                ROS_INFO("Continuous goal updating is enabled");
                //sleep for specified frequency and then continue searching
                rate.sleep();
            }else{
                //wait for movement to finish before continuing
                while(ros::ok() && as_.isActive() && moving_){
                    ros::WallDuration(0.1).sleep();
                }
            }
            while(ros::ok() && as_.isActive() && moving_){
                ros::WallDuration(0.1).sleep();
            }

            ros::Duration(delay_time).sleep();
            
            image_transport::ImageTransport it = image_transport::ImageTransport(nh_);
            image_transport::Publisher image_pub = it.advertise("/candidate",1000);
            
            // check if time is up before we do any further processing
            double current_time = ros::Time::now().toSec();
            double used_time = current_time - starting_time;
            if (used_time > available_time) {
              ROS_WARN("Time is up!");
              finishedTask(true);
              return;
            } else {
              ROS_INFO_STREAM("Remaining time " << available_time - used_time);
            }
            
            // Call service to get snapshot
            ros::ServiceClient snapshot_client = nh_.serviceClient<object_candidates::Snapshot>("get_snapshot");
            object_candidates::Snapshot snap_srv;
            
            ROS_INFO("Requesting snapshot");
            if (snapshot_client.call(snap_srv))
            {
              ROS_INFO_STREAM("Received snapshot, " << snap_srv.response.candidates.data.size() << " candidates");
              
              // Publish first of the received candidates
              ros::WallDuration(0.1).sleep();
              image_pub.publish(snap_srv.response.candidates.data[0]);
              ros::WallDuration(0.1).sleep();

              // Call service to perform 3D location on the candidates in the snapshot
              // TODO: Trim candidates so we don't get annoying "shadows"?
              ros::ServiceClient locator_client = nh_.serviceClient<candidate_locator::LocateCandidates>("locate_candidates");
              candidate_locator::LocateCandidates locator_srv;
              locator_srv.request.depth_image = snap_srv.response.depth_image;
              locator_srv.request.rgb_image = snap_srv.response.rgb_image;
              locator_srv.request.rgb_info = snap_srv.response.rgb_info;
              locator_srv.request.candidates = snap_srv.response.candidates;
              locator_srv.request.publish = true;

              ROS_INFO("Locating candidates");
              if (locator_client.call(locator_srv) && locator_srv.response.candidates.data.size() > 0)
              {
                ros::ServiceClient octomap_merge_client = nh_.serviceClient<octomap_msgs::MergeCandidates>("octomap_server/merge_candidates");
                octomap_msgs::MergeCandidates octomap_merge_srv;
                octomap_merge_srv.request.candidates = locator_srv.response.candidates;

                ROS_INFO("Merging candidates");
                if (octomap_merge_client.call(octomap_merge_srv))
                {
                  ROS_INFO("Candidates merged");
                  finishedTask(false);
                }
                else
                {
                  ROS_ERROR("Failed to call octomap candidate merging service");     
                }
              }
              else
              {
                if (locator_srv.response.candidates.data.size() <= 0)
                  ROS_ERROR("Candidate locator service returned no point clouds");
                else
                  ROS_ERROR("Failed to call candidate locator service");
              }
            }
            else
            {
              ROS_ERROR("Failed to call snapshot service");
            }
        }

        //goal should never be active at this point
        ROS_ASSERT(!as_.isActive());

    }


    /**
     * @brief Preempt callback for the server, cancels the current running goal and all associated movement actions.
     */
    void preemptCb(){

        boost::unique_lock<boost::mutex> lock(move_client_lock_);
        move_client_.cancelGoalsAtAndBeforeTime(ros::Time::now());
        ROS_WARN("Current exploration task cancelled");

        if(as_.isActive()){
            as_.setPreempted();
        }

    }

    /**
     * @brief Feedback callback for the move_base client, republishes as feedback for the exploration server
     * @param feedback Feedback from the move_base client
     */
    void feedbackMovingCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback){

        feedback_.base_position = feedback->base_position;
        as_.publishFeedback(feedback_);

    }

    /**
     * @brief Done callback for the move_base client, checks for errors and aborts exploration task if necessary
     * @param state State from the move_base client
     * @param result Result from the move_base client
     */
    void doneMovingCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result){

        if (state == actionlib::SimpleClientGoalState::ABORTED){
            ROS_ERROR("Failed to move");
            as_.setAborted();
            finishedTask(true);
        }else if(state == actionlib::SimpleClientGoalState::SUCCEEDED){
            moving_ = false;
        }

    }

};

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "explore_server");

    frontier_exploration::FrontierExplorationServer server(ros::this_node::getName());
    ros::spin();
    return 0;
}
