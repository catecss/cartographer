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

#ifndef CARTOGRAPHER_GRPC_HANDLERS_ADD_RANGEFINDER_DATA_HANDLER_H
#define CARTOGRAPHER_GRPC_HANDLERS_ADD_RANGEFINDER_DATA_HANDLER_H

#include "cartographer/common/make_unique.h"
#include "cartographer_grpc/framework/rpc_handler.h"
#include "cartographer_grpc/map_builder_server.h"
#include "cartographer_grpc/proto/map_builder_service.pb.h"
#include "google/protobuf/empty.pb.h"

namespace cartographer_grpc {
namespace handlers {

class AddRangefinderDataHandler
    : public framework::RpcHandler<
          framework::Stream<proto::AddRangefinderDataRequest>,
          google::protobuf::Empty> {
 public:
  std::string method_name() const override {
    return "/cartographer_grpc.proto.MapBuilderService/AddRangefinderData";
  }
  void OnRequest(const proto::AddRangefinderDataRequest &request) override {
    // The 'BlockingQueue' returned by 'sensor_data_queue()' is already
    // thread-safe. Therefore it suffices to get an unsynchronized reference to
    // the 'MapBuilderContext'.
    GetUnsynchronizedContext<MapBuilderContext>()->EnqueueSensorData(
        request.sensor_metadata().trajectory_id(),
        cartographer::sensor::MakeDispatchable(
            request.sensor_metadata().sensor_id(),
            cartographer::sensor::FromProto(request.timed_point_cloud_data())));
  }

  void OnReadsDone() override {
    Send(cartographer::common::make_unique<google::protobuf::Empty>());
  }
};

}  // namespace handlers
}  // namespace cartographer_grpc

#endif  // CARTOGRAPHER_GRPC_HANDLERS_ADD_RANGEFINDER_DATA_HANDLER_H
