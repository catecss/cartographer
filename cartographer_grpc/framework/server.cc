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

#include "glog/logging.h"

namespace cartographer_grpc {
namespace framework {

void Server::Builder::SetNumGrpcThreads(const size_t num_grpc_threads) {
  options_.num_grpc_threads = num_grpc_threads;
}

void Server::Builder::SetNumEventThreads(const std::size_t num_event_threads) {
  options_.num_event_threads = num_event_threads;
}

void Server::Builder::SetServerAddress(const std::string& server_address) {
  options_.server_address = server_address;
}

std::unique_ptr<Server> Server::Builder::Build() {
  std::unique_ptr<Server> server(new Server(options_));
  for (const auto& service_handlers : rpc_handlers_) {
    server->AddService(service_handlers.first, service_handlers.second);
  }
  return server;
}

Server::Server(const Options& options) : options_(options) {
  server_builder_.AddListeningPort(options_.server_address,
                                   grpc::InsecureServerCredentials());

  // Set up event queue threads.
  for (size_t i = 0; i < options_.num_event_threads; ++i) {
    event_queue_threads_.emplace_back();
  }

  // Set up completion queues threads.
  for (size_t i = 0; i < options_.num_grpc_threads; ++i) {
    completion_queue_threads_.emplace_back(
        server_builder_.AddCompletionQueue());
  }
}

void Server::AddService(
    const std::string& service_name,
    const std::map<std::string, RpcHandlerInfo>& rpc_handler_infos) {
  // Instantiate and register service.
  const auto result =
      services_.emplace(std::piecewise_construct, std::make_tuple(service_name),
                        std::make_tuple(service_name, rpc_handler_infos,
                                        options_.num_event_threads,
                                        GetEventInserter()));
  CHECK(result.second) << "A service named " << service_name
                       << " already exists.";
  server_builder_.RegisterService(&result.first->second);
}

void Server::RunCompletionQueue(
    ::grpc::ServerCompletionQueue* completion_queue) {
  bool ok;
  void* tag;
  while (completion_queue->Next(&tag, &ok)) {
    auto* rpc_event = static_cast<Rpc::RpcEvent*>(tag);
    rpc_event->ok = ok;
    event_queue_threads_.at(rpc_event->event_queue_id).event_queue()->Push(rpc_event);
  }
}

void Server::ProcessRpcEvent(Rpc::RpcEvent* rpc_event) {
  if (auto rpc = rpc_event->rpc.lock()) {
        rpc->service()->HandleEvent(rpc_event->event, rpc.get(), rpc_event->ok);
      } else {
        LOG(WARNING) << "Ignoring stale event.";
      }
      delete rpc_event;
}

EventInserter Server::GetEventInserter() {
  return [this](int event_queue_id, Rpc::RpcEvent* rpc_event) {
    event_queue_threads_.at(event_queue_id).event_queue()->Push(rpc_event);
  };
}

void Server::RunEventQueue(EventQueue* event_queue) {
  while(!shutting_down_) {
    Rpc::RpcEvent* rpc_event = event_queue->PopWithTimeout(cartographer::common::FromMilliseconds(100));
    if (rpc_event) {
      ProcessRpcEvent(rpc_event);
    }
  }

  // Finish processing the rest of the items.
  while(Rpc::RpcEvent* rpc_event = event_queue->PopWithTimeout(cartographer::common::FromMilliseconds(100))) {
    ProcessRpcEvent(rpc_event);
  }
}

void Server::Start() {
  // Start the gRPC server process.
  server_ = server_builder_.BuildAndStart();

  // Start serving all services on all completion queues.
  for (auto& service : services_) {
    service.second.StartServing(completion_queue_threads_,
                                execution_context_.get());
  }

  // Start threads to process all event queues.
  for (auto& event_queue_thread : event_queue_threads_) {
    event_queue_thread.Start([this](EventQueue* event_queue){
      RunEventQueue(event_queue);
    });
  }

  // Start threads to process all completion queues.
  for (auto& completion_queue_threads : completion_queue_threads_) {
    completion_queue_threads.Start([this](::grpc::ServerCompletionQueue* completion_queue) {
      RunCompletionQueue(completion_queue);
    });
  }
}

void Server::WaitForShutdown() {
  if (!server_) {
    return;
  }

  server_->Wait();
}

void Server::Shutdown() {
  LOG(INFO) << "Shutting down server.";
  shutting_down_ = true;

  // Tell the services to stop serving RPCs.
  for (auto& service : services_) {
    service.second.StopServing();
  }

  // Shut down the gRPC server waiting for RPCs to finish until the hard
  // deadline; then force a shutdown.
  server_->Shutdown();

  // Shut down the server completion queues and wait for the processing threads
  // to join.
  for (auto& completion_queue_threads : completion_queue_threads_) {
    completion_queue_threads.Shutdown();
  }

  for (auto& event_queue_thread : event_queue_threads_) {
    event_queue_thread.Shutdown();
  }

  LOG(INFO) << "Shutdown complete.";
}

void Server::SetExecutionContext(
    std::unique_ptr<ExecutionContext> execution_context) {
  // After the server has been started the 'ExecutionHandle' cannot be changed
  // anymore.
  CHECK(!server_);
  execution_context_ = std::move(execution_context);
}

}  // namespace framework
}  // namespace cartographer_grpc
