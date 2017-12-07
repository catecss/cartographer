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

#ifndef CARTOGRAPHER_GRPC_MAPPING_TRAJECTORY_BUILDER_STUB_H
#define CARTOGRAPHER_GRPC_MAPPING_TRAJECTORY_BUILDER_STUB_H

#include "cartographer/mapping/trajectory_builder.h"
#include "cartographer_grpc/proto/map_builder_service.grpc.pb.h"
#include "cartographer_grpc/proto/map_builder_service.pb.h"
#include "grpc++/grpc++.h"

namespace cartographer_grpc {
namespace mapping {

class MapBuilderStub;
class TrajectoryBuilderStub : public cartographer::mapping::TrajectoryBuilder {
 public:
  TrajectoryBuilderStub(int trajectory_id,
                        std::shared_ptr<grpc::Channel> client_channel);

  void AddRangefinderData(
      const std::string& sensor_id, cartographer::common::Time time,
      const Eigen::Vector3f& origin,
      const cartographer::sensor::TimedPointCloud& ranges) override;
  void AddImuData(const std::string& sensor_id, cartographer::common::Time time,
                  const Eigen::Vector3d& linear_acceleration,
                  const Eigen::Vector3d& angular_velocity) override;
  void AddOdometerData(
      const std::string& sensor_id, cartographer::common::Time time,
      const cartographer::transform::Rigid3d& odometer_pose) override;
  void AddFixedFramePoseData(
      const std::string& sensor_id, cartographer::common::Time time,
      const cartographer::transform::Rigid3d& fixed_frame_pose) override;

 private:
  int trajectory_id_;
  std::shared_ptr<grpc::Channel> client_channel_;
  std::unique_ptr<proto::MapBuilderService::Stub> stub_;

  grpc::ClientContext range_finder_client_context_;
  std::unique_ptr<grpc::ClientWriter<proto::AddRangefinderDataRequest> >
      range_finder_writer;
  std::unique_ptr<google::protobuf::Empty> range_finder_response_;

  grpc::ClientContext imu_client_context_;
  std::unique_ptr<grpc::ClientWriter<proto::AddImuDataRequest> > imu_writer;
  std::unique_ptr<google::protobuf::Empty> imu_response_;
};

}  // namespace mapping
}  // namespace cartographer_grpc

#endif  // CARTOGRAPHER_GRPC_MAPPING_TRAJECTORY_BUILDER_STUB_H
