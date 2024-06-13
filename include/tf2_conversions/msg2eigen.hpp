#pragma once

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace tf2_conversions
{
// conversion for translation or position
template <typename Scalar_t>
inline Eigen::Vector3<Scalar_t> convertPointToEigen(const geometry_msgs::msg::Point& msg)
{
    return Eigen::Vector3<Scalar_t>(
        msg.x,
        msg.y, 
        msg.z
    );
}

template <typename Scalar_t>
inline Eigen::Vector3<Scalar_t> convertVector3ToEigen(const geometry_msgs::msg::Vector3& msg)
{
    return Eigen::Vector3<Scalar_t>(
        msg.x,
        msg.y, 
        msg.z
    );
}

template <typename Scalar_t>
inline Eigen::Vector3<Scalar_t> convertVector3ToEigen(const geometry_msgs::msg::Vector3Stamped& msg)
{
    return convertVector3ToEigen<Scalar_t>(msg.vector);
}

template <typename Scalar_t>
inline Eigen::Vector3<Scalar_t> convertPointToEigen(const geometry_msgs::msg::PointStamped& msg)
{
    return convertPointToEigen<Scalar_t>(msg.point);
}

template <typename Scalar_t>
inline Eigen::Vector3<Scalar_t> convertTranslationToEigen(const geometry_msgs::msg::Transform& msg)
{
    return convertVector3ToEigen<Scalar_t>(msg.translation);
}

template <typename Scalar_t>
inline Eigen::Vector3<Scalar_t> convertTranslationToEigen(const geometry_msgs::msg::TransformStamped& msg)
{
    return convertTranslationToEigen<Scalar_t>(msg.transform);
}

template <typename Scalar_t>
inline Eigen::Vector3<Scalar_t> convertPositionToEigen(const geometry_msgs::msg::Pose& msg)
{
    return convertPointToEigen<Scalar_t>(msg.position);
}

template <typename Scalar_t>
inline Eigen::Vector3<Scalar_t> convertPositionToEigen(const geometry_msgs::msg::PoseStamped& msg)
{
    return convertPositionToEigen<Scalar_t>(msg.pose);
}

// conversion for rotation or orientation
template <typename Scalar_t>
inline Eigen::Quaternion<Scalar_t> convertQuaternionToEigen(const geometry_msgs::msg::Quaternion& msg)
{
    return Eigen::Quaternion<Scalar_t>(
        msg.w,
        msg.x,
        msg.y,
        msg.z
    );
}

template <typename Scalar_t>
inline Eigen::Quaternion<Scalar_t> convertQuaternionToEigen(const geometry_msgs::msg::QuaternionStamped& msg)
{
    return convertQuaternionToEigen<Scalar_t>(msg.quaternion);
}

template <typename Scalar_t>
inline Eigen::Quaternion<Scalar_t> convertRotationToEigen(const geometry_msgs::msg::Transform& msg)
{
    return convertQuaternionToEigen<Scalar_t>(msg.rotation);
}

template <typename Scalar_t>
inline Eigen::Quaternion<Scalar_t> convertRotationToEigen(const geometry_msgs::msg::TransformStamped& msg)
{
    return convertRotationToEigen<Scalar_t>(msg.transform);
}

template <typename Scalar_t>
inline Eigen::Quaternion<Scalar_t> convertOrientationToEigen(const geometry_msgs::msg::Pose& msg)
{
    return convertQuaternionToEigen<Scalar_t>(msg.orientation);
}

template <typename Scalar_t>
inline Eigen::Quaternion<Scalar_t> convertOrientationToEigen(const geometry_msgs::msg::PoseStamped& msg)
{
    return convertOrientationToEigen<Scalar_t>(msg.pose);
}
}