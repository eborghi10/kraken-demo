/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/Math/Quaternion.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/std/containers/vector.h>
#include <ROS2/Sensor/Events/PhysicsBasedSource.h>
#include <ROS2/Sensor/ROS2SensorComponentBase.h>
#include <ROS2Controllers/VehicleDynamics/VehicleConfiguration.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/publisher.hpp>

namespace AppleKraken
{
    //! Wheel odometry for an Ackermann vehicle, dead reckoned from the wheels the robot actually
    //! has rather than from the speed it was asked for.
    //!
    //! The ROS 2 Gem ships a wheel odometry sensor, but it requires a skid steering model: the
    //! Ackermann drive model leaves GetVelocityFromModel unimplemented, so on this robot the gem's
    //! component would never activate. This one reads the drive wheels' measured angular velocity
    //! out of PhysX and the steered knuckles' measured angle out of their transforms, and
    //! integrates a bicycle model over them.
    //!
    //! Measured, not commanded, is the whole point. When a wheel spins on wet ground the encoder
    //! keeps counting while the robot stays put, so this estimate walks away from the truth - which
    //! is the failure the localisation stack is here to survive.
    class KrakenWheelOdometryComponent : public ROS2::ROS2SensorComponentBase<ROS2::PhysicsBasedSource>
    {
    public:
        using SensorBaseType = ROS2::ROS2SensorComponentBase<ROS2::PhysicsBasedSource>;

        AZ_COMPONENT(KrakenWheelOdometryComponent, "{3D0B84E1-2E4F-4D7A-9F5C-1C8A6B2F7D31}", SensorBaseType);

        KrakenWheelOdometryComponent();
        ~KrakenWheelOdometryComponent() = default;

        static void Reflect(AZ::ReflectContext* context);

        void Activate() override;
        void Deactivate() override;

    private:
        struct DriveWheel
        {
            AZ::EntityId m_entityId;
            float m_radius{ 0.0f };
        };

        struct SteeringKnuckle
        {
            AZ::EntityId m_entityId;
            AZ::Quaternion m_restRotation{ AZ::Quaternion::CreateIdentity() };
        };

        //! Resolve wheels and knuckles from the vehicle configuration. Deferred to the first
        //! physics tick because the entities it looks up are siblings, not yet activated in Activate.
        void CacheVehicle();

        void Integrate();
        void Publish();

        float MeasureForwardSpeed() const;
        float MeasureSteeringAngle() const;

        ROS2Controllers::VehicleDynamics::VehicleConfiguration m_vehicleConfiguration;
        AZ::Vector3 m_poseLinearVariance;
        AZ::Vector3 m_poseAngularVariance;
        AZ::Vector3 m_twistLinearVariance;
        AZ::Vector3 m_twistAngularVariance;

        AZStd::vector<DriveWheel> m_driveWheels;
        AZStd::vector<SteeringKnuckle> m_steeringKnuckles;
        bool m_vehicleCached{ false };

        AZ::Vector3 m_position{ AZ::Vector3::CreateZero() };
        AZ::Quaternion m_orientation{ AZ::Quaternion::CreateIdentity() };
        float m_linearSpeed{ 0.0f };
        float m_yawRate{ 0.0f };
        double m_lastIntegrationTime{ 0.0 };

        nav_msgs::msg::Odometry m_odometryMsg;
        std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> m_odometryPublisher;
    };
} // namespace AppleKraken
