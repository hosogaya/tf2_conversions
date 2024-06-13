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
template <typename Scalar_t>
inline void convertEigenToPoint(const Eigen::Vector3<Scalar_t>& v, geometry_msgs::msg::Point* msg)
{
    msg->x = v.x();
    msg->y = v.y();
    msg->z = v.z();
}

template <typename Scalar_t>
inline void convertEigenToPoint(const Eigen::Vector3<Scalar_t>& v, geometry_msgs::msg::PointStamped* msg)
{
    convertEigenToPoint<Scalar_t>(v, &(msg->point));
}

template <typename Scalar_t>
inline void convertEigenToVector3(const Eigen::Vector3<Scalar_t>& v, geometry_msgs::msg::Vector3* msg)
{
    msg->x = v.x();
    msg->y = v.y();
    msg->z = v.z();
}

template <typename Scalar_t>
inline void convertEigenToVector3(const Eigen::Vector3<Scalar_t>& v, geometry_msgs::msg::Vector3Stamped* msg)
{
    convertEigenToVector3<Scalar_t>(v, &(msg->vector));
}

template <typename Scalar_t>
inline void convertEigenToTranslation(const Eigen::Vector3<Scalar_t>& v, geometry_msgs::msg::Transform* msg)
{
    convertEigenToVector3<Scalar_t>(v, &(msg->translation));
}

template <typename Scalar_t>
inline void convertEigenToTranslation(const Eigen::Vector3<Scalar_t>& v, geometry_msgs::msg::TransformStamped* msg)
{
    convertEigenToVector3<Scalar_t>(v, &(msg->transform.translation));
}

template <typename Scalar_t>
inline void convertEigenToPosition(const Eigen::Vector3<Scalar_t>& v, geometry_msgs::msg::Pose* msg)
{
    convertEigenToPoint<Scalar_t>(v, &(msg->position));
}

template <typename Scalar_t>
inline void convertEigenToPosition(const Eigen::Vector3<Scalar_t>& v, geometry_msgs::msg::PoseStamped* msg)
{
    convertEigenToPoint<Scalar_t>(v, &(msg->pose.position));
}

template <typename Scalar_t>
inline void convertEigenToQuaternion(const Eigen::Quaternion<Scalar_t>& q, geometry_msgs::msg::Quaternion* msg)
{
    msg->w = q.w();
    msg->x = q.x();
    msg->y = q.y();
    msg->z = q.z();
}

template <typename Scalar_t>
inline void convertEigenToQuaternion(const Eigen::Matrix3<Scalar_t>& r, geometry_msgs::msg::Quaternion* msg)
{
    convertEigenToQuaternion<Scalar_t>(Eigen::Quaternion<Scalar_t>(r), msg);
}

template <typename Scalar_t>
inline void convertEigenToQuaternion(const Eigen::Quaternion<Scalar_t>& q, geometry_msgs::msg::QuaternionStamped* msg)
{
    convertEigenToQuaternion<Scalar_t>(q, &(msg->quaternion));
}

template <typename Scalar_t>
inline void convertEigenToQuaternion(const Eigen::Matrix3<Scalar_t>& r, geometry_msgs::msg::QuaternionStamped* msg)
{
    convertEigenToQuaternion<Scalar_t>(Eigen::Quaternion<Scalar_t>(r), msg);
}

template <typename Scalar_t>
inline void convertEigenToRotation(const Eigen::Quaternion<Scalar_t>& q, geometry_msgs::msg::Transform* msg)
{
    convertEigenToQuaternion<Scalar_t>(q, &(msg->rotation));
}

template <typename Scalar_t>
inline void convertEigenToRotation(const Eigen::Matrix3<Scalar_t>& r, geometry_msgs::msg::Transform* msg)
{
    convertEigenToRotation<Scalar_t>(Eigen::Quaternion<Scalar_t>(r), msg);
}

template <typename Scalar_t>
inline void convertEigenToRotation(const Eigen::Quaternion<Scalar_t>& q, geometry_msgs::msg::TransformStamped* msg)
{
    convertEigenToRotation<Scalar_t>(q, &(msg->transform));
}

template <typename Scalar_t>
inline void convertEigenToRotation(const Eigen::Matrix3<Scalar_t>& r, geometry_msgs::msg::TransformStamped* msg)
{
    convertEigenToRotation<Scalar_t>(Eigen::Quaternion<Scalar_t>(r), msg);
}

template <typename Scalar_t>
inline void convertEigenToOrientation(const Eigen::Quaternion<Scalar_t>& q, geometry_msgs::msg::Pose* msg)
{
    convertEigenToQuaternion<Scalar_t>(q, &(msg->orientation));
}

template <typename Scalar_t>
inline void convertEigenToOrientation(const Eigen::Matrix3<Scalar_t>& r, geometry_msgs::msg::Pose* msg)
{
    convertEigenToOrientation<Scalar_t>(Eigen::Quaternion<Scalar_t>(r), msg);
}

template <typename Scalar_t>
inline void convertEigenToOrientation(const Eigen::Quaternion<Scalar_t>& q, geometry_msgs::msg::PoseStamped* msg)
{
    convertEigenToOrientation<Scalar_t>(q, &(msg->pose));
}

template <typename Scalar_t>
inline void convertEigenToOrientation(const Eigen::Matrix3<Scalar_t>& r, geometry_msgs::msg::PoseStamped* msg)
{
    convertEigenToOrientation<Scalar_t>(Eigen::Quaternion<Scalar_t>(r), msg);
}

}