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
#include "grpc++/impl/codegen/proto_utils.h"

namespace cartographer_grpc {
namespace framework {

Service::Service(const std::string& service_name,
                 const std::map<std::string, RpcHandlerInfo>& rpc_handler_infos)
    : rpc_handler_infos_(rpc_handler_infos) {
  for (const auto& rpc_handler_info : rpc_handler_infos_) {
    std::string fully_qualified_method_name =
        "/" + service_name + "/" + rpc_handler_info.first;
    // The 'handler' below is set to 'nullptr' indicating that we want to
    // handle this method asynchronously.
    this->AddMethod(new grpc::internal::RpcServiceMethod(
        fully_qualified_method_name.c_str(), rpc_handler_info.second.rpc_type,
        nullptr /* handler */));
  }
}

void Service::StartServing(
    std::vector<CompletionQueueThread>& completion_queue_threads) {
  int i = 0;
  for (const auto& rpc_handler_info : rpc_handler_infos_) {
    for (auto& completion_queue_thread : completion_queue_threads) {
      Rpc* rpc = active_rpcs_.Add(cartographer::common::make_unique<Rpc>(
          i, completion_queue_thread.completion_queue(),
          rpc_handler_info.second, this));
      rpc->RequestNextMethodInvocation();
    }
    ++i;
  }
}

void Service::StopServing() { shutting_down_ = true; }

void Service::HandleEvent(Rpc::State state, Rpc* rpc, bool ok) {
  rpc->GetRpcState(state)->pending = false;
  switch (state) {
    case Rpc::State::NEW_CONNECTION:
      HandleNewConnection(rpc, ok);
      break;
    case Rpc::State::READ:
      HandleRead(rpc, ok);
      break;
    case Rpc::State::WRITE:
      HandleWrite(rpc, ok);
      break;
    case Rpc::State::DONE:
      HandleDone(rpc, ok);
      break;
  }
}

void Service::HandleNewConnection(Rpc* rpc, bool ok) {
  if (shutting_down_) {
    if (ok) {
      LOG(WARNING) << "Server shutting down. Refusing to handle new RPCs.";
    }
    active_rpcs_.Remove(rpc);
    return;
  }

  if (!ok) {
    LOG(ERROR) << "Failed to establish connection for unknown reason.";
    active_rpcs_.Remove(rpc);
  }

  if (ok) {
    // For request-streaming RPCs ask the client to start sending requests.
    rpc->RequestStreamingReadIfNeeded();
  }

  // Create new active rpc to handle next connection and register it for the
  // incoming connection.
  active_rpcs_.Add(rpc->Clone())->RequestNextMethodInvocation();
}

void Service::HandleRead(Rpc* rpc, bool ok) {
  if (ok) {
    rpc->OnRequest();
    rpc->RequestStreamingReadIfNeeded();
    return;
  }

  // Reads completed.
  rpc->OnReadsDone();
}

void Service::HandleWrite(Rpc* rpc, bool ok) {
  if (!ok) {
    LOG(ERROR) << "Write failed";
  }

  RemoveIfNotPending(rpc);
}

void Service::HandleDone(Rpc* rpc, bool ok) { RemoveIfNotPending(rpc); }

void Service::RemoveIfNotPending(Rpc* rpc) {
  if (!rpc->GetRpcState(Rpc::State::DONE)->pending &&
      !rpc->GetRpcState(Rpc::State::READ)->pending &&
      !rpc->GetRpcState(Rpc::State::WRITE)->pending) {
    active_rpcs_.Remove(rpc);
  }
}

}  // namespace framework
}  // namespace cartographer_grpc
