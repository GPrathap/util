#include <ros/ros.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <mav_planning_msgs/PolynomialSegment4D.h>
#include <mav_msgs/default_topics.h>
#include <nav_msgs/Path.h>

using namespace mav_trajectory_generation;

ros::Publisher traj_pub;
ros::Publisher viz_pub;

void planCallback(const nav_msgs::Path::ConstPtr& msg) {

    std::cout<< "======segment_times==============>>>" << std::endl;

    // int i=0; 
    std::vector<geometry_msgs::PoseStamped> data = msg->poses; 
   
    //Create a list of three (x,y,z) vertices to fly through, e.g. (0,0,1) -> (1,2,3) -> (2,1,5), and define some parameters. The dimension variable denotes the spatial dimension of the path (here, 3D). The derivative_to_optimize should usually be set to the last derivative that should be continuous (here, snap).
    mav_trajectory_generation::Vertex::Vector vertices;
    std::cout<< "----1" << std::endl;
    const int dimension = 4;
    const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
    mav_trajectory_generation::Vertex start(dimension), middle1(dimension), end(dimension);
    //Add constraints to the vertices.
    std::cout<< "----2" << std::endl;
    auto start_pos = Eigen::Vector4d(data[0].pose.position.x, data[0].pose.position.y, data[0].pose.position.z, 1.0);
    start.makeStartOrEnd(start_pos, derivative_to_optimize);
    std::cout<< "----3" << std::endl;
    vertices.push_back(start);
    double r = 100;
    std::cout<< "----4" << std::endl;
    auto previous_pos = start_pos;
    for(int a = 1; a < data.size()-1; a++)
    {
      auto current_pos = Eigen::Vector4d(data[a].pose.position.x, data[a].pose.position.y, data[a].pose.position.z, 1.0);
      // std::cout<< current_pos.transpose() << std::endl;
      std::cout<< "----5" << std::endl;
      if((current_pos-previous_pos).norm()> 0.02){
        middle1.addConstraint(mav_trajectory_generation::derivative_order::POSITION, current_pos);
        vertices.push_back(middle1);
        previous_pos = current_pos;
      }
    }
    std::cout<< "----6" << std::endl;
    auto back_pose = data.back();
    end.makeStartOrEnd(Eigen::Vector4d(back_pose.pose.position.x, back_pose.pose.position.y, back_pose.pose.position.z, 1.0), derivative_to_optimize);
    vertices.push_back(end);
    std::cout<< "---7" << std::endl;
    //Compute the segment times
    std::cout<< "======vertices==============>>>" << vertices.size() << std::endl;
    std::vector<double> segment_times;
    const double v_max = 10.0;
    const double a_max = 2.5;
    segment_times = estimateSegmentTimes(vertices, v_max, a_max);
    // for(auto const& seg : segment_times){
    //   std::cout<< seg << std::endl;
    // }
    std::cout<< "======segment_times==============>>>" << segment_times.size() << std::endl;
    //Linear Optimization
    const int N = 10;
    mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
    opt.solveLinear();
    //  Obtain the polynomial segments.
    //  mav_trajectory_generation::Segment::Vector segments;
    //  opt.getPolynomialOptimizationRef().getSegments(&segments);
    //  Creating Trajectories
    mav_trajectory_generation::Trajectory trajectory;
    opt.getTrajectory(&trajectory);


    //    Sampling Trajectories



    //    Visualizing Trajectories
    visualization_msgs::MarkerArray markers;
    double distance = 1.0; // Distance by which to seperate additional markers. Set 0.0 to disable.
    std::string frame_id = "world";
    std::cout<< "----5" << std::endl;

    // From Trajectory class:
    mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);

    mav_planning_msgs::PolynomialTrajectory4D msg4d;
    trajectoryToPolynomialTrajectoryMsg(trajectory, &msg4d);
    std::cout<< "----6" << std::endl;
    
    
    
    std::cout<< "----7" << std::endl;

    // ros::Rate rate(0.5); rate.sleep();

    viz_pub.publish(markers);
    traj_pub.publish(msg4d);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "DJIAttitudeControllerNode");
    ros::NodeHandle nh;
    traj_pub = nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("path_segments", 1, true);
    viz_pub = nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 1, true);

    std::cout<< "---------" << std::endl;
    ros::Subscriber sub = nh.subscribe("/hagen/proposed_trajectory", 10, planCallback, ros::TransportHints().tcpNoDelay(true));
    ros::spin();
    return 0;
}
