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

#ifndef CARTOGRAPHER_GRPC_HANDLERS_GET_ALL_SUBMAP_DATA_HANDLER_H
#define CARTOGRAPHER_GRPC_HANDLERS_GET_ALL_SUBMAP_DATA_HANDLER_H

#include "cartographer_grpc/framework/rpc_handler.h"
#include "cartographer_grpc/map_builder_server.h"
#include "cartographer_grpc/proto/map_builder_service.pb.h"
#include "google/protobuf/empty.pb.h"

namespace cartographer_grpc {
namespace handlers {

class GetAllSubmapDataHandler
    : public framework::RpcHandler<google::protobuf::Empty,
                                   proto::GetAllSubmapDataResponse> {
 public:
  void OnRequest(const google::protobuf::Empty& request) override {
    cartographer::mapping::MapById<cartographer::mapping::SubmapId,
                                   cartographer::mapping::PoseGraphInterface::SubmapData>
        submap_data = GetContext<MapBuilderServer::MapBuilderContext>()
        ->map_builder().pose_graph()->GetAllSubmapData();

    auto response =
        cartographer::common::make_unique<proto::GetAllSubmapDataResponse>();
    for (const auto& submap : submap_data) {
      auto added_submap_data = response->add_submap_data();
      submap.data.submap->ToProto(added_submap_data);
    }
    Send(std::move(response));
  }
};

}  // namespace handlers
}  // namespace cartographer_grpc

#endif  // CARTOGRAPHER_GRPC_HANDLERS_GET_ALL_SUBMAP_DATA_HANDLER_H
