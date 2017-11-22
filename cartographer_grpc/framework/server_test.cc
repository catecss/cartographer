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

#include "cartographer_grpc/framework/server.h"

#include "cartographer_grpc/framework/proto/math_service.grpc.pb.h"
#include "cartographer_grpc/framework/proto/math_service.pb.h"
#include "cartographer_grpc/framework/rpc_handler.h"
#include "glog/logging.h"
#include "grpc++/grpc++.h"
#include "gtest/gtest.h"

namespace cartographer_grpc {
namespace framework {
namespace {

class GetServerOptionsHandler
    : public RpcHandler<Stream<proto::GetSumRequest>, proto::GetSumResponse> {
 public:
  GetServerOptionsHandler() = default;

  void OnRequest(const proto::GetSumRequest* request) override {
    LOG(INFO) << "Got " << request->input();
    sum_ += request->input();
  }

  void OnReadsDone() override {
    LOG(INFO) << "Reads done.";
    std::unique_ptr<proto::GetSumResponse> response(new proto::GetSumResponse);
    response->set_output(sum_);
    Send(std::move(response));
  }

 private:
  int sum_ = 0;
};

const std::string kServerAddress = "localhost:50051";
const size_t kNumThreads = 1;

class ServerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    Server::Builder server_builder;
    server_builder.SetServerAddress(kServerAddress);
    server_builder.SetNumberOfThreads(kNumThreads);
    server_builder.RegisterHandler<GetServerOptionsHandler, proto::Math>(
        "GetSum");
    server_ = server_builder.Build();
  }

  std::unique_ptr<Server> server_;
};

TEST_F(ServerTest, StartAndStopServerTest) {
  server_->Start();
  server_->Shutdown();
}

TEST_F(ServerTest, ProcessRpcStreamTest) {
  server_->Start();

  auto channel =
      grpc::CreateChannel(kServerAddress, grpc::InsecureChannelCredentials());
  std::unique_ptr<proto::Math::Stub> stub_(proto::Math::NewStub(channel));
  grpc::ClientContext context;
  proto::GetSumResponse result;
  std::unique_ptr<grpc::ClientWriter<proto::GetSumRequest> > writer(
      stub_->GetSum(&context, &result));
  for (int i = 0; i < 3; ++i) {
    proto::GetSumRequest request;
    request.set_input(i);
    CHECK(writer->Write(request));
  }
  writer->WritesDone();
  grpc::Status status = writer->Finish();
  CHECK(status.ok());
  LOG(INFO) << "Received " << result.DebugString();

  server_->Shutdown();
}

}  // namespace
}  // namespace framework
}  // namespace cartographer_grpc

