#include <tf2_conversions/eigen2msg.hpp>

#include <iostream>
using namespace tf2_conversions;

int main()
{
    Eigen::Vector3<double> v{0.0, 1.0, 2.0};
    Eigen::Quaternion<double> q{1.0, 0.0, 2.0, 3.0};

    geometry_msgs::msg::TransformStamped::UniquePtr tf = std::make_unique<geometry_msgs::msg::TransformStamped>();
    convertEigenToTranslation(v, tf.get());
    convertEigenToRotation(q.toRotationMatrix(), tf.get());


    std::cout << "pos: " << tf->transform.translation.x << ", "
                         << tf->transform.translation.y << ", "
                         << tf->transform.translation.z << std::endl;

    std::cout << "quat: " << tf->transform.rotation.w << ", "
                          << tf->transform.rotation.x << ", "
                          << tf->transform.rotation.y << ", "
                          << tf->transform.rotation.z << std::endl;
    return 0;
}