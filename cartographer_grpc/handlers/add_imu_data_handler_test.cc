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
#include "cartographer_grpc/framework/testing/rpc_handler_test_server.h"
#include "cartographer_grpc/testing/mock_map_builder_context.h"
#include "gtest/gtest.h"

namespace cartographer_grpc {
namespace handlers {
namespace {

class AddImuDataHandlerTest : public ::testing::Test {
 protected:
  framework::testing::RpcHandlerTestServer<AddImuDataHandler> test_server_;
};

TEST_F(AddImuDataHandlerTest, NoUploader) {
  auto mock_map_builder =
      cartographer::common::make_unique<testing::MockMapBuilderContext>();
  LOG(INFO) << "Sending write";
  test_server_.SendWrite("");
  LOG(INFO) << "Sending WritesDone";
  test_server_.SendWritesDone();
  LOG(INFO) << "Sending Finish";
  test_server_.SendFinish();
}

}  // namespace
}  // namespace handlers
}  // namespace cartographer_grpc