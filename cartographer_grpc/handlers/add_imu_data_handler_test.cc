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

#include "cartographer_grpc/handlers/add_imu_data_handler.h"
#include "cartographer_grpc/framework/testing/test_server.h"
#include "glog/logging.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace cartographer_grpc {
namespace handlers {
namespace {

using framework::testing::MakeMsg;

TEST(AddImuDataHandlerTest, Test) {
  framework::testing::TestServer<AddImuDataHandler> test_server;
  test_server.Start();
  test_server.Write(
      MakeMsg<proto::SensorMetadata>(0, "lala"),
      MakeMsg<cartographer::sensor::proto::ImuData>(
          (int64_t)0,
          MakeMsg<cartographer::transform::proto::Vector3d>(3.0, 4.0, 5.0),
          MakeMsg<cartographer::transform::proto::Vector3d>(3.0, 4.0, 5.0)));
  test_server.WritesDone();
  LOG(INFO) << "Sending finish";
  test_server.Finish();
  test_server.Shutdown();
}

} // namespace
} // namespace handlers
} // namespace cartographer_grpc
