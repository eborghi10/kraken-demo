/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "KrakenWheelOdometryComponent.h"

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>
#include <ROS2/Clock/ROS2ClockRequestBus.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2NamesBus.h>
#include <ROS2/Utilities/ROS2Conversions.h>

#include <cmath>
#include <array>

namespace AppleKraken
{
    namespace
    {
        const char* OdometryMsgType = "nav_msgs::msg::Odometry";

        std::array<double, 36> DiagonalCovariance(const AZ::Vector3& linear, const AZ::Vector3& angular)
        {
            std::array<double, 36> covariance{};
            covariance[0] = linear.GetX();
            covariance[7] = linear.GetY();
            covariance[14] = linear.GetZ();
            covariance[21] = angular.GetX();
            covariance[28] = angular.GetY();
            covariance[35] = angular.GetZ();
            return covariance;
        }
    } // namespace

    KrakenWheelOdometryComponent::KrakenWheelOdometryComponent()
    {
        ROS2::TopicConfiguration topicConfiguration;
        topicConfiguration.m_type = OdometryMsgType;
        topicConfiguration.m_topic = "wheel/odom";
        m_sensorConfiguration.m_frequency = 50.0f;
        m_sensorConfiguration.m_publishersConfigurations.insert(
            AZStd::make_pair(AZStd::string(OdometryMsgType), topicConfiguration));

        // A zero covariance would tell a filter that dead reckoning is exact, which is the one
        // thing it is not. These are placeholders for a measured value, not a measurement.
        m_poseLinearVariance = AZ::Vector3(0.05f, 0.05f, 1e-6f);
        m_poseAngularVariance = AZ::Vector3(1e-6f, 1e-6f, 0.05f);
        m_twistLinearVariance = AZ::Vector3(0.01f, 0.01f, 1e-6f);
        m_twistAngularVariance = AZ::Vector3(1e-6f, 1e-6f, 0.01f);
    }

    void KrakenWheelOdometryComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<KrakenWheelOdometryComponent, SensorBaseType>()
                ->Version(1)
                ->Field("Vehicle configuration", &KrakenWheelOdometryComponent::m_vehicleConfiguration)
                ->Field("Pose linear variance", &KrakenWheelOdometryComponent::m_poseLinearVariance)
                ->Field("Pose angular variance", &KrakenWheelOdometryComponent::m_poseAngularVariance)
                ->Field("Twist linear variance", &KrakenWheelOdometryComponent::m_twistLinearVariance)
                ->Field("Twist angular variance", &KrakenWheelOdometryComponent::m_twistAngularVariance);

            if (auto* editContext = serialize->GetEditContext())
            {
                editContext
                    ->Class<KrakenWheelOdometryComponent>(
                        "Kraken Wheel Odometry", "Dead reckoning from measured wheel rotation and steering angle")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROSConDemo")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &KrakenWheelOdometryComponent::m_vehicleConfiguration,
                        "Vehicle configuration",
                        "Axles, wheel radius and wheelbase this odometry is computed from")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &KrakenWheelOdometryComponent::m_poseLinearVariance,
                        "Pose linear variance",
                        "Confidence claimed for the integrated position")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &KrakenWheelOdometryComponent::m_poseAngularVariance,
                        "Pose angular variance",
                        "Confidence claimed for the integrated heading")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &KrakenWheelOdometryComponent::m_twistLinearVariance,
                        "Twist linear variance",
                        "Confidence claimed for the reported speed")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &KrakenWheelOdometryComponent::m_twistAngularVariance,
                        "Twist angular variance",
                        "Confidence claimed for the reported yaw rate");
            }
        }
    }

    void KrakenWheelOdometryComponent::Activate()
    {
        SensorBaseType::Activate();

        m_position = AZ::Vector3::CreateZero();
        m_orientation = AZ::Quaternion::CreateIdentity();
        m_linearSpeed = 0.0f;
        m_yawRate = 0.0f;
        m_vehicleCached = false;
        m_lastIntegrationTime = 0.0;

        AZStd::string odomFrame;
        ROS2::ROS2NamesRequestBus::BroadcastResult(
            odomFrame, &ROS2::ROS2NamesRequestBus::Events::GetNamespacedName, GetNamespace(), "odom");
        m_odometryMsg.header.frame_id = odomFrame.c_str();
        m_odometryMsg.child_frame_id = GetNamespacedFrameID().c_str();

        const auto& publisherConfig = m_sensorConfiguration.m_publishersConfigurations[OdometryMsgType];
        AZStd::string fullTopic;
        ROS2::ROS2NamesRequestBus::BroadcastResult(
            fullTopic, &ROS2::ROS2NamesRequestBus::Events::GetNamespacedName, GetNamespace(), publisherConfig.m_topic);

        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        m_odometryPublisher = ros2Node->create_publisher<nav_msgs::msg::Odometry>(fullTopic.data(), publisherConfig.GetQoS());

        StartSensor(
            m_sensorConfiguration.m_frequency,
            [this]([[maybe_unused]] auto&&... args)
            {
                Integrate();
                if (m_sensorConfiguration.m_publishingEnabled)
                {
                    Publish();
                }
            });
    }

    void KrakenWheelOdometryComponent::Deactivate()
    {
        StopSensor();
        m_odometryPublisher.reset();
        m_driveWheels.clear();
        m_steeringKnuckles.clear();
        SensorBaseType::Deactivate();
    }

    void KrakenWheelOdometryComponent::CacheVehicle()
    {
        AZ::Transform baseTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(baseTransform, GetEntityId(), &AZ::TransformInterface::GetWorldTM);
        const AZ::Quaternion baseInverse = baseTransform.GetRotation().GetInverseFull();

        for (const auto& axle : m_vehicleConfiguration.m_axles)
        {
            for (const auto& wheelId : axle.m_axleWheels)
            {
                if (axle.m_isDrive)
                {
                    m_driveWheels.push_back({ wheelId, axle.m_wheelRadius });
                }

                if (!axle.m_isSteering)
                {
                    continue;
                }

                // A steered wheel hangs off a knuckle, and it is the knuckle that carries the angle.
                AZ::EntityId knuckleId;
                AZ::TransformBus::EventResult(knuckleId, wheelId, &AZ::TransformInterface::GetParentId);
                if (!knuckleId.IsValid())
                {
                    continue;
                }

                AZ::Transform knuckleTransform = AZ::Transform::CreateIdentity();
                AZ::TransformBus::EventResult(knuckleTransform, knuckleId, &AZ::TransformInterface::GetWorldTM);
                m_steeringKnuckles.push_back({ knuckleId, baseInverse * knuckleTransform.GetRotation() });
            }
        }

        AZ_Warning("KrakenWheelOdometry", !m_driveWheels.empty(), "No drive wheels configured, odometry will report zero");
        AZ_Warning("KrakenWheelOdometry", m_vehicleConfiguration.m_wheelbase > 0.0f, "Wheelbase must be positive");
    }

    float KrakenWheelOdometryComponent::MeasureForwardSpeed() const
    {
        float total = 0.0f;
        int counted = 0;

        for (const auto& wheel : m_driveWheels)
        {
            AzPhysics::RigidBody* body = nullptr;
            Physics::RigidBodyRequestBus::EventResult(body, wheel.m_entityId, &Physics::RigidBodyRequests::GetRigidBody);
            if (body == nullptr)
            {
                continue;
            }

            const AZ::Vector3 spin =
                body->GetTransform().GetRotation().GetInverseFull().TransformVector(body->GetAngularVelocity());
            // The wheel meshes are authored with their axle along the wheel's own Z, so rolling
            // forward is a positive rotation about it. Measured, not assumed: driving at a ground
            // truth 0.849 m/s reads 2.86 rad/s on Z and nothing on the other two axes.
            total += spin.GetZ() * wheel.m_radius;
            ++counted;
        }

        return counted > 0 ? total / static_cast<float>(counted) : 0.0f;
    }

    float KrakenWheelOdometryComponent::MeasureSteeringAngle() const
    {
        AZ::Transform baseTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(baseTransform, GetEntityId(), &AZ::TransformInterface::GetWorldTM);
        const AZ::Quaternion baseInverse = baseTransform.GetRotation().GetInverseFull();

        float total = 0.0f;
        int counted = 0;

        for (const auto& knuckle : m_steeringKnuckles)
        {
            AZ::Transform knuckleTransform = AZ::Transform::CreateIdentity();
            AZ::TransformBus::EventResult(knuckleTransform, knuckle.m_entityId, &AZ::TransformInterface::GetWorldTM);

            const AZ::Quaternion steered =
                knuckle.m_restRotation.GetInverseFull() * (baseInverse * knuckleTransform.GetRotation());
            total += steered.GetEulerRadians().GetZ();
            ++counted;
        }

        // Averaging the two knuckles collapses the Ackermann pair onto the bicycle model's single
        // virtual wheel, which is what the wheelbase below assumes.
        return counted > 0 ? total / static_cast<float>(counted) : 0.0f;
    }

    void KrakenWheelOdometryComponent::Integrate()
    {
        if (!m_vehicleCached)
        {
            CacheVehicle();
            m_vehicleCached = true;
        }

        m_linearSpeed = MeasureForwardSpeed();

        const float wheelbase = m_vehicleConfiguration.m_wheelbase;
        m_yawRate = wheelbase > 0.0f ? m_linearSpeed * std::tan(MeasureSteeringAngle()) / wheelbase : 0.0f;

        // The physics tick hands out a delta time that does not sum to elapsed simulation time, so
        // the step is taken from the same clock that stamps the message. That keeps the integrated
        // pose consistent with the stamp a filter will use to align it.
        builtin_interfaces::msg::Time now;
        ROS2::ROS2ClockRequestBus::BroadcastResult(now, &ROS2::ROS2ClockRequests::GetROSTimestamp);
        const double nowSeconds = static_cast<double>(now.sec) + 1e-9 * static_cast<double>(now.nanosec);

        const double deltaTime = nowSeconds - m_lastIntegrationTime;
        m_lastIntegrationTime = nowSeconds;

        // The first call has no previous stamp, and a long stall would integrate a stale speed over
        // a gap the robot did not drive at that speed.
        if (deltaTime <= 0.0 || deltaTime > 1.0)
        {
            return;
        }

        const float step = static_cast<float>(deltaTime);
        m_position += m_orientation.TransformVector(AZ::Vector3(m_linearSpeed * step, 0.0f, 0.0f));
        m_orientation *= AZ::Quaternion::CreateFromScaledAxisAngle(AZ::Vector3(0.0f, 0.0f, m_yawRate * step));
        m_orientation.Normalize();
    }

    void KrakenWheelOdometryComponent::Publish()
    {
        builtin_interfaces::msg::Time timestamp;
        ROS2::ROS2ClockRequestBus::BroadcastResult(timestamp, &ROS2::ROS2ClockRequests::GetROSTimestamp);
        m_odometryMsg.header.stamp = timestamp;

        m_odometryMsg.pose.pose.position = ROS2::ROS2Conversions::ToROS2Point(m_position);
        m_odometryMsg.pose.pose.orientation = ROS2::ROS2Conversions::ToROS2Quaternion(m_orientation);
        m_odometryMsg.pose.covariance = DiagonalCovariance(m_poseLinearVariance, m_poseAngularVariance);

        m_odometryMsg.twist.twist.linear = ROS2::ROS2Conversions::ToROS2Vector3(AZ::Vector3(m_linearSpeed, 0.0f, 0.0f));
        m_odometryMsg.twist.twist.angular = ROS2::ROS2Conversions::ToROS2Vector3(AZ::Vector3(0.0f, 0.0f, m_yawRate));
        m_odometryMsg.twist.covariance = DiagonalCovariance(m_twistLinearVariance, m_twistAngularVariance);

        m_odometryPublisher->publish(m_odometryMsg);
    }
} // namespace AppleKraken
