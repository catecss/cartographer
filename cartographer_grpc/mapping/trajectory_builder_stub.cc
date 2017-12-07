/*
 * Copyright 2017 The Cartographer Authors
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

#include "cartographer_grpc/mapping/trajectory_builder_stub.h"

#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "glog/logging.h"

namespace cartographer_grpc {
namespace mapping {

TrajectoryBuilderStub::TrajectoryBuilderStub(
    int trajectory_id, std::shared_ptr<grpc::Channel> client_channel)
    : trajectory_id_(trajectory_id), client_channel_(client_channel) {
  LOG(INFO) << "TrajectoryBuilderStub created.";
  stub_ = proto::MapBuilderService::NewStub(client_channel_);
  CHECK(stub_) << "Failed to create stub.";
}

void TrajectoryBuilderStub::AddRangefinderData(
    const std::string& sensor_id, cartographer::common::Time time,
    const Eigen::Vector3f& origin,
    const cartographer::sensor::TimedPointCloud& ranges) {
  if (!range_finder_writer) {
    range_finder_response_ =
        cartographer::common::make_unique<google::protobuf::Empty>();
    range_finder_writer = stub_->AddRangefinderData(
        &range_finder_client_context_, range_finder_response_.get());
  }
  proto::AddRangefinderDataRequest range_finder_data_request;
  proto::SensorMetadata sensor_metadata;
  sensor_metadata.set_trajectory_id(trajectory_id_);
  sensor_metadata.set_sensor_id(sensor_id);
  *range_finder_data_request.mutable_sensor_metadata() = sensor_metadata;
  *range_finder_data_request.mutable_timed_point_cloud_data() =
      cartographer::sensor::ToProto(
          cartographer::sensor::TimedPointCloudData{time, origin, ranges});
  range_finder_writer->Write(range_finder_data_request);
}

void TrajectoryBuilderStub::AddImuData(
    const std::string& sensor_id, cartographer::common::Time time,
    const Eigen::Vector3d& linear_acceleration,
    const Eigen::Vector3d& angular_velocity) {
  if (!imu_writer) {
    imu_response_ =
        cartographer::common::make_unique<google::protobuf::Empty>();
    imu_writer = stub_->AddImuData(&imu_client_context_, imu_response_.get());
  }
  proto::AddImuDataRequest imu_data_request;
  proto::SensorMetadata sensor_metadata;
  sensor_metadata.set_trajectory_id(trajectory_id_);
  sensor_metadata.set_sensor_id(sensor_id);
  *imu_data_request.mutable_sensor_metadata() = sensor_metadata;
  *imu_data_request.mutable_imu_data() =
      cartographer::sensor::ToProto(cartographer::sensor::ImuData{
          time, linear_acceleration, angular_velocity});

  imu_writer->Write(imu_data_request);
}

void TrajectoryBuilderStub::AddOdometerData(
    const std::string& sensor_id, cartographer::common::Time time,
    const cartographer::transform::Rigid3d& odometer_pose) {
  LOG(FATAL) << "Not implemented.";
}

void TrajectoryBuilderStub::AddFixedFramePoseData(
    const std::string& sensor_id, cartographer::common::Time time,
    const cartographer::transform::Rigid3d& fixed_frame_pose) {
  LOG(FATAL) << "Not implemented.";
}

}  // namespace mapping
}  // namespace cartographer_grpc