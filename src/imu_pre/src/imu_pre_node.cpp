#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <se3.hpp>
#include <so3.hpp>
#include <unsupported/Eigen/MatrixFunctions>
// 角速度
int angular_velocity = 10;
// 线速度
int linear_velocity = 5;

int main(int argc, char **argv)
{

    // 初始化ROS节点
    ros::init(argc, argv, "sophus_publisher_node");

    // 创建ROS节点句柄
    ros::NodeHandle nh;

    // 创建ROS发布者，发布geometry_msgs::PoseStamped消息
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/point_pose", 10);

    // 创建ROS发布者，发布nav_msgs::Path消息
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/point_path", 10);

    double angular_velocity_rad = angular_velocity * 3.1415926 / 180;

    // 创建一个Sophus::SE3位姿对象（如果使用sophus）
    Sophus::SE3d pose_se;
    // 使用Matrix表示矩阵
    Eigen::Matrix<double, 4, 4> pose = Eigen::Matrix<double, 4, 4>::Identity(); // 位姿

    Eigen::Vector3d omega(0, 0, angular_velocity_rad);
    Eigen::Vector3d v_body(linear_velocity, 0, 0);
    const double dt = 0.05;

    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "world";

    // 发布消息
    ros::Rate loop_rate(10); // 发布频率1Hz
    double time = 0.0;
    while (ros::ok())
    {
        // Eigen::Matrix朴素实现
        Eigen::Vector3d v_world = pose.block<3, 3>(0, 0) * v_body;
        pose.block<3, 1>(0, 3) += v_world * dt;
        
        // SE3实现
        // Eigen::Vector3d v_world = pose_se.so3() * v_body;
        // std::cout << "v_world: " << v_world << std::endl;
        // pose_se.translation() += v_world * dt;

        // 更新自身位姿
        // 使用exp进行更新
        if (0)
        {
            double theta = omega.norm();         // theat是旋转向量对应的旋转角度
            Eigen::Vector3d n_w = omega / theta; // n_w是旋转向量对应的旋转轴
            Eigen::Matrix3d n_w_skew;            // n_w_skew旋转轴的反对称矩阵

            n_w_skew << 0, -n_w(2), n_w(1),
                n_w(2), 0, -n_w(0),
                -n_w(1), n_w(0), 0;

            // Rodrigues's formula
            Eigen::Matrix3d R_w_exp = (theta * n_w_skew * dt).exp();

            std::cout << "R_w_exp: \n"
                      << R_w_exp << "\n";

            pose.block<3, 3>(0, 0) = pose.block<3, 3>(0, 0) * R_w_exp;
        }
        // 使用罗德里格斯公式替代exp
        if (0)
        {
            Eigen::Vector3d omega_theta = omega * dt;
            double theta = omega_theta.norm();         // theat是旋转向量对应的旋转角度
            Eigen::Vector3d n_w = omega_theta / theta; // n_w是旋转向量对应的旋转轴
            Eigen::Matrix3d n_w_skew;                  // n_w_skew旋转轴的反对称矩阵

            n_w_skew << 0, -n_w(2), n_w(1),
                n_w(2), 0, -n_w(0),
                -n_w(1), n_w(0), 0;

            Eigen::Matrix3d R_w = cos(theta) * Eigen::Matrix3d::Identity() + (1 - cos(theta)) * n_w * n_w.transpose() + sin(theta) * n_w_skew; // Rodrigues's formula

            std::cout << "R_w \n"
                      << R_w;

            pose.block<3, 3>(0, 0) = pose.block<3, 3>(0, 0) * R_w;
        }
        // 使用Eigen中的四元数进行更新
        if (1)
        {
            Eigen::Quaterniond qq(pose.block<3, 3>(0, 0));
            Eigen::Quaterniond q_w(1, 0.5 * omega(0) * dt, 0.5 * omega(1) * dt, 0.5 * omega(2) * dt); // 小量角速度对应的四元数
            q_w.normalize();// 单位四元数才可以表示三维旋转，所以必须归一化
            Eigen::Quaterniond q_update = qq * q_w;
            q_update.normalize(); // 单位四元数才可以表示三维旋转，所以必须归一化
            std::cout << "更新后的四元数：\n";
            std::cout << q_update.coeffs().transpose() << "\n";
            pose.block<3, 3>(0, 0) = q_update.toRotationMatrix();
        }
        // 使用sophus中的四元数更新
        if (0)
        {
            Eigen::Quaterniond q = pose_se.unit_quaternion() * Eigen::Quaterniond(1, 0.5 * omega[0] * dt, 0.5 * omega[1] * dt, 0.5 * omega[2] * dt);
            q.normalize();
            pose_se.so3() = Sophus::SO3d(q);
        }
        // 使用sophus中的exp进行更新
        if (0)
        {
            pose_se.so3() = pose_se.so3() * Sophus::SO3d::exp(omega * dt);
        }

        // 获取当前时间戳
        ros::Time current_time = ros::Time::now();

        // 创建并填充geometry_msgs::PoseStamped消息
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = current_time;
        pose_msg.header.frame_id = "world";
        // 提取平移部分
        Eigen::Vector3d translation = pose.block<3, 1>(0, 3);
        pose_msg.pose.position.x = translation.x();
        pose_msg.pose.position.y = translation.y();
        pose_msg.pose.position.z = translation.z();
        // 提取旋转部分并转换为四元数
        Eigen::Matrix3d rotation = pose.block<3, 3>(0, 0);
        Eigen::Quaterniond quat(rotation);
        pose_msg.pose.orientation.x = quat.x();
        pose_msg.pose.orientation.y = quat.y();
        pose_msg.pose.orientation.z = quat.z();
        pose_msg.pose.orientation.w = quat.w();

        /******************** 使用 sophus ***********************/
        // 创建并填充nav_msgs::Path消息
        // geometry_msgs::PoseStamped pose_msg;
        // pose_msg.header.stamp = current_time;
        // pose_msg.header.frame_id = "world";
        // pose_msg.pose.position.x = pose_se.translation().x();
        // pose_msg.pose.position.y = pose_se.translation().y();
        // pose_msg.pose.position.z = pose_se.translation().z();
        // Eigen::Quaterniond quat = pose_se.unit_quaternion();
        // pose_msg.pose.orientation.x = quat.x();
        // pose_msg.pose.orientation.y = quat.y();
        // pose_msg.pose.orientation.z = quat.z();
        // pose_msg.pose.orientation.w = quat.w();
        
        path_msg.poses.push_back(pose_msg);
        // 发布消息
        pose_pub.publish(pose_msg);
        path_pub.publish(path_msg);
        time += 0.1;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
