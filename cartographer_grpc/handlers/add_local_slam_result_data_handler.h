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

#ifndef CARTOGRAPHER_GRPC_HANDLERS_ADD_LOCAL_SLAM_RESULT_DATA_HANDLER_H
#define CARTOGRAPHER_GRPC_HANDLERS_ADD_LOCAL_SLAM_RESULT_DATA_HANDLER_H

#include "cartographer/common/make_unique.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer_grpc/framework/rpc_handler.h"
#include "cartographer_grpc/map_builder_server.h"
#include "cartographer_grpc/proto/map_builder_service.pb.h"
#include "google/protobuf/empty.pb.h"

namespace cartographer_grpc {
namespace handlers {

class AddLocalSlamResultDataHandler
    : public framework::RpcHandler<
          framework::Stream<proto::AddLocalSlamResultDataRequest>,
          google::protobuf::Empty> {
 public:
  void OnRequest(const proto::AddLocalSlamResultDataRequest& request) override {
    LOG(INFO) << "ProcessLocalSlamResultData";
    auto local_slam_result_data =
        GetContext<MapBuilderServer::MapBuilderContext>()
            ->ProcessLocalSlamResultData(
                request.sensor_metadata().sensor_id(),
                cartographer::common::FromUniversal(
                    request.local_slam_result_data().timestamp()),
                request.local_slam_result_data());
    LOG(INFO) << "!ProcessLocalSlamResultData";
    GetContext<MapBuilderServer::MapBuilderContext>()
        ->EnqueueLocalSlamResultData(request.sensor_metadata().trajectory_id(),
                                     request.sensor_metadata().sensor_id(),
                                     std::move(local_slam_result_data));
  }

  void OnReadsDone() override {
    Send(cartographer::common::make_unique<google::protobuf::Empty>());
  }
};

}  // namespace handlers
}  // namespace cartographer_grpc

#endif  // CARTOGRAPHER_GRPC_HANDLERS_ADD_LOCAL_SLAM_RESULT_DATA_HANDLER_H
