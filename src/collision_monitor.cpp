#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <map>
#include <cmath> 
#include <vector>
#include <std_msgs/Empty.h>
#include <time.h>
#include <fstream>
#include <sstream>
#include <string>


// ros::Publisher marker_pub; // パブリッシャーをグローバルで宣言

//---------------------------自己位置がodomになってる--------------------------------
class CollisionMonitor
{
public:
    CollisionMonitor(){
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("collision_monitor", 1);
        collision_state_pub_ = nh_.advertise<std_msgs::Int16>("collision_state", 1000, this);
        sub_waypoint_state_pub_ = nh_.advertise<std_msgs::Int16>("sub_waypoint_state", 1000, this);

        waypoint_sub_ = nh_.subscribe("waypoint", 1000, &CollisionMonitor::waypointCallback, this);
        scan_sub_ = nh_.subscribe("/scan", 10, &CollisionMonitor::scanCallback, this);
        odom_sub_ = nh_.subscribe("ypspur_ros/odom", 10, &CollisionMonitor::odomCallback, this);
        timer_callback_ = nh_.createTimer(ros::Duration(1.0), &CollisionMonitor::timerCallback, this);
        
        robot_x_ = 0.0;
        robot_y_ = 0.0;
        robot_odom_x_ = 0.0;
        robot_odom_y_ = 0.0;
        robot_r_.x = 0.0;
        robot_r_.y = 0.0;
        robot_r_.z = 0.0;
        robot_r_.w = 1.0;

        x_min = 0.0;
        x_max = 1.5;
        y_min = -0.5;
        y_max = 0.5;
        // void detect_obstacle();
        collision_state_msg_.data = 0;
        sub_waypoint_state_msg_.data = 0;

        }
    void detect_obstacle();

private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_, odom_sub_, waypoint_sub_;
    ros::Publisher marker_pub_, robot_speed_pub_, collision_state_pub_, sub_waypoint_state_pub_;
    ros::Timer timer_callback_;
    ros::Time timer_start_;
    ros::Time timer_now_;
    std_msgs::Int16 collision_state_msg_;
    std_msgs::Int16 sub_waypoint_state_msg_;
    geometry_msgs::Point p_;
    geometry_msgs::PoseStamped goal_;  // 目標地点goal_judge 
    geometry_msgs::Quaternion robot_r_;
    sensor_msgs::LaserScan::ConstPtr scan_;


    double roll_, pitch_, yaw_;
    double theta_;
    double angle_rad_, angle_deg_;
    double distance_, distance_judge_;
    double robot_odom_x_, robot_odom_y_;
    double robot_x_, robot_y_;
    double x_min, x_max, y_min, y_max;

    double calcAngle(geometry_msgs::PoseStamped goal);
    void timer();
    void geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void timerCallback(const ros::TimerEvent&);
    void visualize(double x_min, double x_max, double y_min, double y_max);
    void waypointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

};

    void CollisionMonitor::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan) {
        scan_ = scan;
    }

    void CollisionMonitor::timerCallback(const ros::TimerEvent&) {

    }

    void CollisionMonitor::detect_obstacle(){//calcAngleをつかって目的地との間にある障害物を検知するようにする！！！！！！！！！！！！！！！！！！！！！
        if (!scan_) {
            // scan_が初期化されていない場合は処理をスキップ
            ROS_WARN("Laser scan data is not yet received.");
            return;
        }
        double waypoint_angle_rad = std::atan2(goal_.pose.position.y - robot_odom_y_ , goal_.pose.position.x - robot_odom_x_); // ラジアン
        std::cout << "waypoint_angle_rad(度数法):" << waypoint_angle_rad * (180.0 / M_PI) << std::endl;
        for (size_t i = 0; i < scan_->ranges.size(); i++) {
            angle_rad_ = scan_->angle_min + i * scan_->angle_increment;
            angle_rad_ = angle_rad_ - waypoint_angle_rad;//次のwaypointを加味できてる？？？
            angle_deg_ = angle_rad_ * (180.0 / M_PI); // ラジアンを度数法に変換
            distance_ = scan_->ranges[i];
            distance_judge_ = 100;

            // float angle_rad = scan->angle_min + i * scan->angle_increment;
            // float angle_deg = angle_rad * (180.0 / M_PI); // ラジアンを度数法に変換
            // float distance = scan->ranges[i];
            // float distance_judge_ = 100;

            // geometry_msgs::Point p;
            p_.x = distance_ * std::cos(angle_rad_); // X座標
            p_.y = distance_ * std::sin(angle_rad_); // Y座標
            p_.z = 0; // Z座標は0に設定

            distance_judge_ = scan_->ranges[i] * std::cos(angle_rad_); // std::cosの入力はrad
            // std::cout << "distance_judge_:" << distance_judge_ << std::endl;

            if (-1 < angle_deg_ && angle_deg_ < 1) { //±10度に戻す
                // distance_judge_ = scan->ranges[i] * std::cos(angle_rad); // std::cosの入力はrad
                if ((-1.5 < distance_judge_ && distance_judge_ < -0.5) || (0.5 < distance_judge_ && distance_judge_ < 1.5)) {
            // std::cout << "distance_judge_:" << distance_judge_ << std::endl;

                    // std::cout << "1_0.5~1.5" << std::endl;
                    collision_state_msg_.data = 1;
                    sub_waypoint_state_msg_.data = 0;


                }else if(-0.5 <= distance_judge_ && distance_judge_ <= 0.5){
            // std::cout << "distance_judge_:" << distance_judge_ << std::endl;

                    // std::cout << " 1_~0.5" <<  std::endl;
                    collision_state_msg_.data = 2;
                    sub_waypoint_state_msg_.data = 1;

                    timer();//detect_obstacleがずっと呼ばれていると抜け出せない

                }else{
            // std::cout << "distance_judge_:" << distance_judge_ << std::endl;

                    collision_state_msg_.data = 0;
                    sub_waypoint_state_msg_.data = 0;

                }

            visualize(robot_odom_x_ + x_min, robot_odom_x_ + x_max, robot_odom_y_ + y_min, robot_odom_y_ + y_max);
            
            }
            
            // else if ((angle_deg_ >= -90 && angle_deg_ <= -10) || (angle_deg_ >= 10 && angle_deg_ <= 90)) {
            //     // distance_judge = scan_->ranges[i] * std::sin(angle_rad); // std::sinの入力はrad
            //     if ((-1.5 < distance_judge_ && distance_judge_ < -0.5) || (0.5 < distance_judge_ && distance_judge_ < 1.5)) {
            //         std::cout << "2_0.5~1.5" << std::endl;
            //         collision_state_msg_.data = 1;

            //     }
            //     else if(-0.5 <= distance_judge_ && distance_judge_ <= 0.5){
            //         std::cout << " 2_~0.5" <<  std::endl;
            //         collision_state_msg_.data = 2;

            //         timer();//detect_obstacleがずっと呼ばれていると抜け出せない

            //     }else{
            //         collision_state_msg_.data = 0;
            //     }
            // }
        }
        std::cout << "collision_state_msg_.data:" << collision_state_msg_.data << std::endl;
        collision_state_pub_.publish(collision_state_msg_);
        sub_waypoint_state_pub_.publish(sub_waypoint_state_msg_);
    }

    void CollisionMonitor::visualize(double x_min, double x_max, double y_min, double y_max) {
        // マーカーの設定
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";  // 必要に応じて座標フレームを設定
        marker.header.stamp = ros::Time::now();
        marker.ns = "collision_area";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        // 短冊状エリアの中心とサイズを計算
        marker.pose.position.x = (x_min + x_max) / 2.0;
        marker.pose.position.y = (y_min + y_max) / 2.0;
        marker.pose.position.z = 0.0;  // 高さ方向は0
        marker.pose.orientation.w = 1.0;

        marker.scale.x = std::abs(x_max - x_min);
        marker.scale.y = std::abs(y_max - y_min);
        marker.scale.z = 0.1;  // 薄い高さを設定

        // 色の設定
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.5;  // 半透明

        // マーカーのライフタイム
        marker.lifetime = ros::Duration(0.1);

        // パブリッシュ
        marker_pub_.publish(marker);
    }



    void CollisionMonitor::geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat)
    {
        tf::Quaternion quat;
        quaternionMsgToTF(geometry_quat, quat);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    }

    double CollisionMonitor::calcAngle(geometry_msgs::PoseStamped goal){
        theta_ = atan2(goal.pose.position.y - robot_odom_y_, goal.pose.position.x - robot_odom_x_);
        while (theta_ <= -M_PI || M_PI <= theta_)
        {
            if (theta_ <= -M_PI)
                theta_ = theta_ + 2 * M_PI;
            else
                theta_ = theta_ - 2 * M_PI;
        }

        geometry_quat_to_rpy(roll_, pitch_, yaw_, robot_r_);

        while (yaw_ <= -M_PI || M_PI <= yaw_)
        {
            if (yaw_ <= -M_PI)
                yaw_ = yaw_ + 2 * M_PI;
            else
                yaw_ = yaw_ - 2 * M_PI;
        }

        theta_ = theta_ - yaw_;
        
        return theta_;
    }

    void CollisionMonitor::timer() {
        // timerの処理をここに記述
    }

    void CollisionMonitor::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {//near_waypointなどはこっちでやる
        robot_odom_x_ = msg->pose.pose.position.x;
        robot_odom_y_ = msg->pose.pose.position.y;
        // std::cout << "robot_odom_x_:" << robot_odom_x_ << "robot_odom_y_;" << robot_odom_y_ << std::endl;
    }

    void CollisionMonitor::waypointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        ROS_INFO("Received waypoint: [x: %f, y: %f]", msg->pose.position.x, msg->pose.position.y);

        // サブスクライブしたウェイポイントを処理
        // 例えば次の目標に設定する
        goal_ = *msg;
        std::cout << "goal_" << goal_ << std::endl;
    }

int main(int argc, char **argv) {
    ros::init(argc, argv, "collision_monitor");

    CollisionMonitor cm;

    ros::Rate loop_rate(100);

    while(ros::ok()) {
        ros::spinOnce();
        // std::cout << "sub_waypoint_flag_:" << sub_waypoint_flag_ << std::endl;
        // std::cout << "state:" << state_ << std::endl;
        // twist_pub.publish(twist);
        cm.detect_obstacle();
        loop_rate.sleep();
    }
    return 0;
}