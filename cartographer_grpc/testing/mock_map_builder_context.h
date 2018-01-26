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

#ifndef CARTOGRAPHER_GRPC_TESTING_MOCK_MAP_BUILDER_CONTEXT_H
#define CARTOGRAPHER_GRPC_TESTING_MOCK_MAP_BUILDER_CONTEXT_H

#include "cartographer_grpc/map_builder_server.h"

namespace cartographer_grpc {
namespace testing {

class MockMapBuilderContext : public MapBuilderServer::MapBuilderContext {
public:
  MockMapBuilderContext() : MapBuilderServer::MapBuilderContext(nullptr) {}
  MOCK_METHOD0(map_builder, cartographer::mapping::MapBuilderInterface &());

  /*
   *
   * cartographer::mapping::MapBuilderInterface& map_builder();
      cartographer::common::BlockingQueue<std::unique_ptr<Data>>&
      sensor_data_queue();
      cartographer::mapping::TrajectoryBuilderInterface::LocalSlamResultCallback
      GetLocalSlamResultCallbackForSubscriptions();
      void AddSensorDataToTrajectory(const Data& sensor_data);
      SubscriptionId SubscribeLocalSlamResults(
          int trajectory_id, LocalSlamSubscriptionCallback callback);
      void UnsubscribeLocalSlamResults(const SubscriptionId& subscription_id);
      void NotifyFinishTrajectory(int trajectory_id);
      std::unique_ptr<cartographer::mapping::LocalSlamResultData>
      ProcessLocalSlamResultData(
          const std::string& sensor_id, cartographer::common::Time time,
          const cartographer::mapping::proto::LocalSlamResultData& proto);
      LocalTrajectoryUploader* local_trajectory_uploader() {
        return map_builder_server_->local_trajectory_uploader_.get();
      }

      template <typename DataType>
      void EnqueueSensorData(int trajectory_id, const std::string& sensor_id,
                             const DataType& data) {
        map_builder_server_->incoming_data_queue_.Push(
            cartographer::common::make_unique<MapBuilderServer::Data>(
                MapBuilderServer::Data{
                    trajectory_id,
                    cartographer::sensor::MakeDispatchable(sensor_id, data)}));
      }

      void EnqueueLocalSlamResultData(
          int trajectory_id, const std::string& sensor_id,
          std::unique_ptr<cartographer::mapping::LocalSlamResultData>
              local_slam_result_data) {
        map_builder_server_->incoming_data_queue_.Push(
            cartographer::common::make_unique<MapBuilderServer::Data>(
                MapBuilderServer::Data{trajectory_id,
                                       std::move(local_slam_result_data)}));
      }
   */
};

} // namespace testing
} // namespace cartographer_grpc

#endif // CARTOGRAPHER_GRPC_TESTING_MOCK_MAP_BUILDER_CONTEXT_H
