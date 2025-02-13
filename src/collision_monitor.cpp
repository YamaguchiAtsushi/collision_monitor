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
#include <visualization_msgs/MarkerArray.h>
#include <map>
#include <cmath> 
#include <vector>
#include <std_msgs/Empty.h>
#include <time.h>
#include <fstream>
#include <sstream>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


// ros::Publisher marker_pub; // パブリッシャーをグローバルで宣言

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
        amcl_sub_ = nh_.subscribe("/amcl_pose", 1000, &CollisionMonitor::amclPoseCallback, this);


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
        visualize_obstacle_detection_area();
        
        collision_state_msg_.data = 0;
        sub_waypoint_state_msg_.data = 0;

        }
    void detect_obstacle();

private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_, odom_sub_, waypoint_sub_, amcl_sub_;
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
    double distance_, distance_judge_, distance_judge_width_;
    double robot_odom_x_, robot_odom_y_;
    double robot_x_, robot_y_;
    double x_min, x_max, y_min, y_max;
    double robot_yaw;
    double waypoint_angle_rad;


    double calcAngle(geometry_msgs::PoseStamped goal);
    void timer();
    void geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void timerCallback(const ros::TimerEvent&);
    void visualize(double x_min, double x_max, double y_min, double y_max);
    void waypointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void visualize_obstacle_detection_area();
    
};

    void CollisionMonitor::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan) {
        scan_ = scan;
    }

    void CollisionMonitor::timerCallback(const ros::TimerEvent&) {

    }

    void CollisionMonitor::detect_obstacle(){//calcAngleをつかって目的地との間にある障害物を検知するようにする！！！！！！！！！！！！！！！！！！！！！
        if (!scan_) {
            ROS_WARN("Laser scan data is not yet received.");
            return;
        }

        waypoint_angle_rad = std::atan2(goal_.pose.position.y - robot_y_ , goal_.pose.position.x - robot_x_) - robot_yaw; // ラジアン
        // std::cout << "(goal_.pose.position_x, goal_.pose.position_y):(" << goal_.pose.position.x << "," << goal_.pose.position.y << ")" << std::endl;
        // std::cout << "(robot_x_, robot_y_):(" << robot_x_ << "," << robot_y_ << ")" << std::endl;
        // std::cout << "robot_yaw:" << robot_yaw << std::endl;
        std::cout << "waypoint_angle_rad(度数法):" << waypoint_angle_rad * (180.0 / M_PI) << std::endl;
        for (size_t i = 0; i < scan_->ranges.size(); i++) {
            distance_ = scan_->ranges[i];

            if(distance_ > 0.002){//URGの表面のゴミ？？による障害物検知を消す！！！
                angle_rad_ = scan_->angle_min + i * scan_->angle_increment;
                angle_rad_ = angle_rad_ - waypoint_angle_rad;//次のwaypointを加味できてる？？？
                angle_deg_ = angle_rad_ * (180.0 / M_PI); // ラジアンを度数法に変換
                distance_ = scan_->ranges[i];
                distance_judge_ = 100;



                p_.x = distance_ * std::cos(angle_rad_); // X座標
                p_.y = distance_ * std::sin(angle_rad_); // Y座標
                p_.z = 0; // Z座標は0に設定

                distance_judge_ = scan_->ranges[i] * std::cos(angle_rad_); // std::cosの入力はrad

                if (-7.0 < angle_deg_ && angle_deg_ < 7.0) { //distance_judge_はマイナスになりえないかも？？
                    // distance_judge_ = scan->ranges[i] * std::cos(angle_rad); // std::cosの入力はrad
                    if ((-2.0 < distance_judge_ && distance_judge_ < -1.0) || (1.0 < distance_judge_ && distance_judge_ < 2.0)) {


                        //減速したいときはコメントアウトを入れ替える！！！
                        collision_state_msg_.data = 1;
                        sub_waypoint_state_msg_.data = 0;
                        // collision_state_msg_.data = 2;
                        // sub_waypoint_state_msg_.data = 1;



                    }else if(-1.0 <= distance_judge_ && distance_judge_ <= 1.0){
                        collision_state_msg_.data = 2;
                        sub_waypoint_state_msg_.data = 1;

                        timer();//detect_obstacleがずっと呼ばれていると抜け出せない

                    }else{
                        collision_state_msg_.data = 0;
                        sub_waypoint_state_msg_.data = 0;

                    }

                
                }
                
                else if ((angle_deg_ >= -30 && angle_deg_ <= -7) || (angle_deg_ >= 7 && angle_deg_ <= 30)) {//90度にすると回避せずに止まる
                    distance_judge_width_ = scan_->ranges[i] * std::sin(angle_rad_); // std::sinの入力はrad
                    if(-0.25 < distance_judge_width_ && distance_judge_width_ < 0.25){
                        // std::cout << "(x,y):(" << p_.x << "," << p_.y << ")" << std::endl;
                        // std::cout << "distance_" << distance_ << std::endl;
                        if ((-2.0 < distance_judge_ && distance_judge_ < -1.0) || (1.0 < distance_judge_ && distance_judge_ < 2.0)) {
                            // std::cout << "2_0.5~1.5" << std::endl;
                            collision_state_msg_.data = 1;
                            sub_waypoint_state_msg_.data = 0;


                        }
                        else if(-1.0 <= distance_judge_ && distance_judge_ <= 1.0){
                            // std::cout << " 2_~0.5" <<  std::endl;
                            collision_state_msg_.data = 2;
                            sub_waypoint_state_msg_.data = 1;

                            // std::cout << "distance_judge_:" << distance_judge_ << std::endl;
                            // std::cout << "distance_judge_width_:" << distance_judge_width_ << std::endl;

                            timer();//detect_obstacleがずっと呼ばれていると抜け出せない

                        }else{
                            collision_state_msg_.data = 0;
                            sub_waypoint_state_msg_.data = 0;
                        }
                    }
                }
            }
        }
        std::cout << "collision_state_msg_.data:" << collision_state_msg_.data << std::endl;
        visualize_obstacle_detection_area();
        collision_state_pub_.publish(collision_state_msg_);
        sub_waypoint_state_pub_.publish(sub_waypoint_state_msg_);
    }

    void CollisionMonitor::visualize_obstacle_detection_area() {
        visualization_msgs::Marker rectangle_marker;
        rectangle_marker.header.frame_id = "map";  // 基準座標系
        rectangle_marker.header.stamp = ros::Time::now();
        rectangle_marker.ns = "rectangle";
        rectangle_marker.id = 0;
        rectangle_marker.type = visualization_msgs::Marker::CUBE;
        rectangle_marker.action = visualization_msgs::Marker::ADD;

        // 平面上に配置
        rectangle_marker.pose.position.x = robot_x_ + 1.5 * std::cos(std::atan2(goal_.pose.position.y - robot_y_ , goal_.pose.position.x - robot_x_));
        rectangle_marker.pose.position.y = robot_y_ + 1.5 * std::sin(std::atan2(goal_.pose.position.y - robot_y_ , goal_.pose.position.x - robot_x_));
        rectangle_marker.pose.position.z = 0.0;  // 高さ0 (地面に配置)
        rectangle_marker.pose.orientation.w = 1.0;

        // サイズ設定 (横2m, 縦1m, 高さ0.01m)
        rectangle_marker.scale.x = 0.3;
        rectangle_marker.scale.y = 0.25;
        rectangle_marker.scale.z = 0.01;  // 厚さを小さくして平面に見せる

        // 色設定 (赤, 透明度 50%)
        rectangle_marker.color.r = 1.0;
        rectangle_marker.color.g = 0.0;
        rectangle_marker.color.b = 0.0;
        rectangle_marker.color.a = 0.9;  // 半透明

        rectangle_marker.lifetime = ros::Duration();

        marker_pub_.publish(rectangle_marker);
    }



    void CollisionMonitor::geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat)
    {
        tf::Quaternion quat;
        quaternionMsgToTF(geometry_quat, quat);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    }

    double CollisionMonitor::calcAngle(geometry_msgs::PoseStamped goal){
        // theta_ = atan2(goal.pose.position.y - robot_odom_y_, goal.pose.position.x - robot_odom_x_);
        theta_ = atan2(goal.pose.position.y - robot_y_, goal.pose.position.x - robot_x_);

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

    void CollisionMonitor::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        robot_r_ = msg->pose.pose.orientation;
        // クォータニオンをオイラー角に変換
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);

        double roll, pitch, yaw;
        
        m.getRPY(roll, pitch, yaw);  // オイラー角（rad）を取得
        // robot_yaw = yaw * 180.0 / M_PI;
        robot_yaw = yaw ;
        // std::cout << "robot_x_:" << robot_x_ << "robot_y_;" << robot_y_ << std::endl;
    }

    void CollisionMonitor::waypointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        ROS_INFO("Received waypoint: [x: %f, y: %f]", msg->pose.position.x, msg->pose.position.y);

        // サブスクライブしたウェイポイントを処理
        // 例えば次の目標に設定する
        goal_ = *msg;
        // std::cout << "goal_" << goal_ << std::endl;
    }

int main(int argc, char **argv) {
    ros::init(argc, argv, "collision_monitor");

    CollisionMonitor cm;

    ros::Rate loop_rate(1000);

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