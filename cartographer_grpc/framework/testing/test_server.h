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

#ifndef CARTOGRAPHER_GRPC_FRAMEWORK_TESTING_SERVER_H
#define CARTOGRAPHER_GRPC_FRAMEWORK_TESTING_SERVER_H

#include <string>

#include "cartographer/common/blocking_queue.h"
#include "cartographer/common/make_unique.h"
#include "cartographer_grpc/framework/rpc_handler_interface.h"
#include "cartographer_grpc/framework/server.h"
#include "cartographer_grpc/framework/testing/msg_helpers.h"
#include "glog/logging.h"

namespace cartographer_grpc {
namespace framework {
namespace testing {

namespace {
const std::string kMethodName = "MethodUnderTest";
const std::string kServiceName = "ServiceUnderTest";
const std::string kFullyQualifiedMethodName =
    "/" + kServiceName + "/" + kMethodName;
const std::string kServerAddress = "localhost:50051";
} // namespace

enum Event { ON_REQUEST, ON_READS_DONE, ON_FINISH };

template <typename RpcHandlerType> class TestServer : public Server {
public:
  TestServer();

  template <typename... Values> void Write(Values... values) {
    InstantiateReadersAndWriters();
    auto msg = MakeMsg<typename RpcHandlerType::RequestType>(values...);
    CHECK(rpc_type_ == ::grpc::internal::RpcMethod::CLIENT_STREAMING ||
          rpc_type_ == ::grpc::internal::RpcMethod::BIDI_STREAMING);
    CHECK(client_writer_->Write(*msg)) << "Write failed.";
    Event event = server_event_queue_.Pop();
    CHECK_EQ(event, ON_REQUEST);
  }

  void WritesDone() {
    InstantiateReadersAndWriters();
    client_writer_->WritesDone();
    Event event = server_event_queue_.Pop();
    CHECK_EQ(event, ON_READS_DONE);
  }

  void Finish() {
    InstantiateReadersAndWriters();
    LOG(INFO) << "client_writer_->Finish();";
    client_writer_->Finish();
    LOG(INFO) << "Finish()";
    Event event = server_event_queue_.Pop();
    CHECK_EQ(event, ON_FINISH);
  }

  void AddEvent(Event event) { server_event_queue_.Push(event); }

private:
  void InstantiateReadersAndWriters() {
    switch (rpc_type_) {
    case ::grpc::internal::RpcMethod::CLIENT_STREAMING:
      if (!response_) {
        response_ = cartographer::common::make_unique<
            typename RpcHandlerType::ResponseType>();
      }
      if (!client_writer_) {
        client_writer_ = std::unique_ptr<
            ::grpc::ClientWriter<typename RpcHandlerType::RequestType>>(
            ::grpc::internal::ClientWriterFactory<
                typename RpcHandlerType::RequestType>::Create(channel_.get(),
                                                              rpc_method_,
                                                              &context_,
                                                              response_.get()));
      }
      break;
    case ::grpc::internal::RpcMethod::SERVER_STREAMING:
    case ::grpc::internal::RpcMethod::NORMAL_RPC:
    case ::grpc::internal::RpcMethod::BIDI_STREAMING:
      LOG(FATAL) << "Not implemented";
      break;
    }
  }

  ::grpc::internal::RpcMethod::RpcType rpc_type_;
  ::grpc::ClientContext context_;
  std::shared_ptr<::grpc::ChannelInterface> channel_;
  ::grpc::internal::RpcMethod rpc_method_;

  std::unique_ptr<::grpc::ClientWriter<typename RpcHandlerType::RequestType>>
      client_writer_;
  std::unique_ptr<typename RpcHandlerType::ResponseType> response_;
  cartographer::common::BlockingQueue<Event> server_event_queue_;
};

template <typename RpcHandlerType>
class HandlerWrapper : public RpcHandlerType {
public:
  HandlerWrapper(TestServer<RpcHandlerType> *);
  void OnRequest(const typename RpcHandlerType::RequestType &request) override {
    LOG(INFO) << "OnRequest!";
    // RpcHandlerType::OnRequest(request);
    LOG(INFO) << "OnRequest!";
    test_server_->AddEvent(ON_REQUEST);
  }

  void OnReadsDone() override {
    LOG(INFO) << "OnReadsDone!";
    test_server_->AddEvent(ON_READS_DONE);
    LOG(INFO) << "OnReadsDone!";
    this->Send(cartographer::common::make_unique<
               typename RpcHandlerType::ResponseType>());
  }

  void OnFinish() override {
    LOG(INFO) << "OnFinish!";
    test_server_->AddEvent(ON_FINISH);
    LOG(INFO) << "OnFinish!";
  }

private:
  TestServer<RpcHandlerType> *test_server_;
};

template <typename RpcHandlerType>
TestServer<RpcHandlerType>::TestServer()
    : Server(Options{1, 1, kServerAddress}),
      rpc_type_(RpcType<typename RpcHandlerType::IncomingType,
                        typename RpcHandlerType::OutgoingType>::value),
      channel_(grpc::CreateChannel(kServerAddress,
                                   grpc::InsecureChannelCredentials())),
      rpc_method_(kFullyQualifiedMethodName.c_str(), rpc_type_, channel_)

{
  this->AddService(
      kServiceName,
      {{kMethodName,
        RpcHandlerInfo{
            RpcHandlerType::RequestType::default_instance().GetDescriptor(),
            RpcHandlerType::ResponseType::default_instance().GetDescriptor(),
            [this](Rpc *const rpc, ExecutionContext *const execution_context) {
              std::unique_ptr<RpcHandlerInterface> rpc_handler =
                  cartographer::common::make_unique<
                      HandlerWrapper<RpcHandlerType>>(this);
              rpc_handler->SetRpc(rpc);
              rpc_handler->SetExecutionContext(execution_context);
              return rpc_handler;
            },
            rpc_type_, kFullyQualifiedMethodName}}});
}

template <typename RpcHandlerType>
HandlerWrapper<RpcHandlerType>::HandlerWrapper(
    TestServer<RpcHandlerType> *test_server)
    : test_server_(test_server) {
}
} // namespace testing
} // namespace framework
} // namespace cartographer_grpc

#endif // CARTOGRAPHER_GRPC_FRAMEWORK_SERVER_H
