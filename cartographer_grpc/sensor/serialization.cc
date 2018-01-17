/*
 * Copyright 2018 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "serialization.h"

namespace cartographer_grpc {
namespace sensor {

void CreateSensorMetadata(const std::string& sensor_id, const int trajectory_id,
                          proto::SensorMetadata* proto) {
  proto->set_sensor_id(sensor_id);
  proto->set_trajectory_id(trajectory_id);
}

void CreateAddFixedFramePoseDataRequest(
    const std::string& sensor_id, int trajectory_id,
    const cartographer::sensor::proto::FixedFramePoseData&
        fixed_frame_pose_data,
    proto::AddFixedFramePoseDataRequest* proto) {
  CreateSensorMetadata(sensor_id, trajectory_id,
                       proto->mutable_sensor_metadata());
  *proto->mutable_fixed_frame_pose_data() = fixed_frame_pose_data;
}

void CreateAddImuDataRequest(
    const std::string& sensor_id, const int trajectory_id,
    const cartographer::sensor::proto::ImuData& imu_data,
    proto::AddImuDataRequest* proto) {
  CreateSensorMetadata(sensor_id, trajectory_id,
                       proto->mutable_sensor_metadata());
  *proto->mutable_imu_data() = imu_data;
}

void CreateAddOdometryDataRequest(
    const std::string& sensor_id, int trajectory_id,
    const cartographer::sensor::proto::OdometryData& odometry_data,
    proto::AddOdometryDataRequest* proto) {
  CreateSensorMetadata(sensor_id, trajectory_id,
                       proto->mutable_sensor_metadata());
  *proto->mutable_odometry_data() = odometry_data;
}

void CreateAddRangeFinderDataRequest(
    const std::string& sensor_id, int trajectory_id,
    const cartographer::sensor::proto::TimedPointCloudData&
        timed_point_cloud_data,
    proto::AddRangefinderDataRequest* proto) {
  CreateSensorMetadata(sensor_id, trajectory_id,
                       proto->mutable_sensor_metadata());
  *proto->mutable_timed_point_cloud_data() = timed_point_cloud_data;
}

void CreateAddLocalSlamResultDataRequest(
    const std::string& sensor_id, int trajectory_id,
    cartographer::common::Time time,
    const cartographer::mapping::TrajectoryBuilderInterface::InsertionResult&
        insertion_result,
    proto::AddLocalSlamResultDataRequest* proto) {
  sensor::CreateSensorMetadata(sensor_id, trajectory_id,
                               proto->mutable_sensor_metadata());
  proto->mutable_local_slam_result_data()->set_timestamp(
      cartographer::common::ToUniversal(time));
  *proto->mutable_local_slam_result_data()->mutable_node_data() =
      cartographer::mapping::ToProto(*insertion_result.constant_data);
  for (const auto& insertion_submap : insertion_result.insertion_submaps) {
    // We only send the probability grid up if the submap is finished.
    insertion_submap->ToProto(
        proto->mutable_local_slam_result_data()->add_submaps(),
        insertion_submap->finished());
  }
}

}  // namespace sensor
}  // namespace cartographer_grpc
