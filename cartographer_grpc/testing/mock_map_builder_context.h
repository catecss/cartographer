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
#include "glog/logging.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

using testing::_;

namespace cartographer_grpc {
namespace testing {

class MockMapBuilderContext
    : public cartographer_grpc::MapBuilderServer::MapBuilderContext {
 public:
  MockMapBuilderContext()
      : cartographer_grpc::MapBuilderServer::MapBuilderContext(nullptr) {}
  MOCK_METHOD0(map_builder, cartographer::mapping::MapBuilderInterface&());
  MOCK_METHOD0(sensor_data_queue,
               cartographer::common::BlockingQueue<
                   std::unique_ptr<MapBuilderServer::Data>>&());
  MOCK_METHOD0(GetLocalSlamResultCallbackForSubscriptions,
               cartographer::mapping::TrajectoryBuilderInterface::
                   LocalSlamResultCallback());
  MOCK_METHOD1(AddSensorDataToTrajectory, void(const MapBuilderServer::Data&));
  MOCK_METHOD2(SubscribeLocalSlamResults,
               MapBuilderServer::SubscriptionId(
                   int, MapBuilderServer::LocalSlamSubscriptionCallback));
  MOCK_METHOD1(UnsubscribeLocalSlamResults,
               void(const MapBuilderServer::SubscriptionId&));
  MOCK_METHOD1(NotifyFinishTrajectory, void(int));

  /*MOCK_METHOD3(ProcessLocalSlamResultData,
     std::unique_ptr<cartographer::mapping::LocalSlamResultData>(const
     std::string&, cartographer::common::Time, const
     cartographer::mapping::proto::LocalSlamResultData&));*/
};

}  // namespace testing
}  // namespace cartographer_grpc

#endif  // CARTOGRAPHER_GRPC_TESTING_MOCK_MAP_BUILDER_CONTEXT_H