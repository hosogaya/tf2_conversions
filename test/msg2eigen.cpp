#include <tf2_conversions/msg2eigen.hpp>

#include <iostream>
using namespace tf2_conversions;

int main()
{
    geometry_msgs::msg::PoseStamped msg;
    msg.pose.position.x = 0.0;
    msg.pose.position.y = 1.0;
    msg.pose.position.z = 2.0;
    msg.pose.orientation.w = 0.0;
    msg.pose.orientation.x = 1.0;
    msg.pose.orientation.y = 2.0;
    msg.pose.orientation.z = 3.0;

    auto pos = convertPositionToEigen<double>(msg);
    auto quat = convertOrientationToEigen<double>(msg);

    std::cout << "pos: " << pos.transpose() << std::endl;
    std::cout << "quat: " << quat << std::endl;


    geometry_msgs::msg::TransformStamped tf;
    tf.transform.translation.x = 2.0;
    tf.transform.translation.y = 1.0;
    tf.transform.translation.z = 0.0;

    tf.transform.rotation.w = 1.0;
    tf.transform.rotation.x = 3.0;
    tf.transform.rotation.y = 2.0;
    tf.transform.rotation.z = 0.0;

    auto tf_pos = convertTranslationToEigen<float>(tf);
    auto tf_quat = convertRotationToEigen<float>(tf);

    std::cout << "tf pos: " << tf_pos.transpose() << std::endl;
    std::cout << "tf quat: " << tf_quat << std::endl;

    return 0;
}