// ROS headers
#include <ros/ros.h>

// message headers
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <mbz2020_common/MotionPoint.h>
#include <mbz2020_common/MotionTrajectory.h>
#include <mbz2020_common/MotionPointStamped.h>
#include <mbz2020_common/MotionPointStampedArray.h>

// planner headers
#include <target_planner.hpp>

using nav_msgs::Path;
using geometry_msgs::PoseStamped;
using mbz2020_common::MotionPoint;
using mbz2020_common::MotionTrajectory;
using mbz2020_common::MotionPointStamped;
using mbz2020_common::MotionPointStampedArray;

using namespace Mbz2020Planner;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planning_server");

    ROS_INFO("Planner Demo Node starting up...");

    ros::NodeHandle nh;

    // private node handle has a specific namespace that allows us to easily
    // encapsulate parameters
    ros::NodeHandle private_nh ("~");

    // PARAMETERS
    // TODO : make planner and node params pull from rosparam
    double update_frequency = 50;

    // Wait for a valid time in case we are using simulated time (not wall time)
    while (ros::ok() && ros::Time::now() == ros::Time(0)) {
        // wait
        ros::spinOnce();
        ros::Duration(.005).sleep();
    }

    ros::Publisher plan_pub = nh.advertise<nav_msgs::Path>("path", 1000);
    ros::Publisher point_pub = nh.advertise<MotionPointStampedArray>("plan", 1000);

    // update frequency of the node
    ros::Rate rate(update_frequency);

    MotionTrajectory targetTraj;
    targetTraj.start_time = ros::Time::now();
    targetTraj.dt = .1;

    double vel = 1.0; // m/s

    double origin = 10;

    // go straight 5 meters
    // or 50 dts at 1 m/s
    for (int i = 0; i < 50; i++) {
        MotionPoint mp;
        mp.twist.linear.x = vel;
        mp.pose.position.x = i * .1 * vel + origin;
        mp.pose.position.y = 0.0 + origin;
        mp.pose.position.z = 5.0;

        targetTraj.motion_points.push_back(mp);
    }

    // go right
    for (int i = 0; i < 50; i++) {
        MotionPoint mp;
        mp.twist.linear.y = vel;
        mp.pose.position.x = 5.0 + origin;
        mp.pose.position.y = i * .1 * vel + origin;
        mp.pose.position.z = 5.0;

        targetTraj.motion_points.push_back(mp);
    }

    // go down
    for (int i = 0; i < 50; i++) {
        MotionPoint mp;
        mp.twist.linear.x = -vel;
        mp.pose.position.x = 5.0 - i * .1 * vel + origin;
        mp.pose.position.y = 5.0 + origin;
        mp.pose.position.z = 5.0;

        targetTraj.motion_points.push_back(mp);
    }

    // go left
    for (int i = 0; i < 50; i++) {
        MotionPoint mp;
        mp.twist.linear.y = -vel;
        mp.pose.position.x = 0.0 + origin;
        mp.pose.position.y = 5.0 - i * .1 * vel + origin;
        mp.pose.position.z = 5.0;

        targetTraj.motion_points.push_back(mp);
    }

    std::shared_ptr<Target> target_ = std::make_shared<Target>(targetTraj);
    std::shared_ptr<Map> map_ = std::make_shared<Map>(private_nh, .1);
    std::shared_ptr<Limits> limits_ = std::make_shared<Limits>();

    limits_->maxX = 20.0;
    limits_->maxY = 20.0;
    limits_->maxZ = 20.0;

    limits_->minX = -20.0;
    limits_->minY = -20.0;
    limits_->minZ = 0.0;

    limits_->maxVel = 10.0;
    limits_->maxAccel = 5.0;

    limits_->maxTime = 200.0;

    double eps = 3.5;

    private_nh.getParam("/eps", eps);

    double poseTol = 1.0; // in meters
    double velTol = 3.0; // in m/s
    double dt = .1; // in seconds
    double hDt = .05;

    std::shared_ptr<Domain> env_ = std::make_shared<Domain>(map_,
                                                            target_,
                                                            limits_,
                                                            eps,
                                                            poseTol,
                                                            velTol,
                                                            dt,
                                                            hDt);

    std::shared_ptr<State> start_ = std::make_shared<State>();
    start_->numX = 0;
    start_->numY = 0;
    start_->numZ = 0;
    start_->numXVel = 0;
    start_->numYVel = 0;
    start_->numZVel = 0;
    start_->time = 0;

    std::shared_ptr<TargetPlanner> planner_ = std::make_shared<TargetPlanner>(env_,
                                                                              start_,
                                                                              eps);
    while (ros::ok()) {
        bool success = false;
        MotionPointStampedArray plan;
        Path path;
        if (env_->canPlan()) {
            success = planner_->plan();
            if (success) {
                plan = planner_->getPlanTrajectory();
                point_pub.publish(plan);

                path.header.stamp = ros::Time::now();
                path.header.frame_id = "map";

                for (MotionPointStamped mp : plan.motion_points) {
                    PoseStamped pose;
                    pose.header = mp.header;
                    pose.pose = mp.motion_point.pose;
                    path.poses.push_back(pose);
                }
                plan_pub.publish(path);
            }

        } else {
            ROS_WARN_THROTTLE(1, "Planner Demo: waiting to plan...");
        }

        while (ros::ok() && success) {
            point_pub.publish(plan);
            plan_pub.publish(path);
            ros::spinOnce();
            rate.sleep();
        }


        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();

    // All is good.
    return 0;
}
