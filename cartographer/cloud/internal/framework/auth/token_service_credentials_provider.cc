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

#include "cartographer/cloud/internal/framework/auth/token_service_credentials_provider.h"
#include "cartographer/cloud/internal/framework/auth/proto/token.pb.h"
#include "glog/logging.h"
#include "google/protobuf/empty.pb.h"
#include "grpc++/impl/codegen/client_unary_call.h"
#include "grpc++/impl/codegen/proto_utils.h"

namespace cartographer {
namespace cloud {
namespace framework {
namespace auth {

TokenServiceCredentialsProvider::TokenServiceCredentialsProvider(
    const std::string &token_server_address)
    : channel_(::grpc::CreateChannel(token_server_address,
                                     ::grpc::InsecureChannelCredentials())) {}

void TokenServiceCredentialsProvider::SetCredentials(
    ::grpc::ClientContext *client_context) {
  ::grpc::ClientContext context;
  google::protobuf::Empty request;
  proto::GetTokenResponse response;
  ::grpc::internal::RpcMethod rpc_method(
      "/cartographer.cloud.framework.auth.proto.Token/GetToken",
      ::grpc::internal::RpcMethod::NORMAL_RPC, channel_);
  CHECK(::grpc::internal::BlockingUnaryCall(channel_.get(), rpc_method,
                                            &context, request, &response)
            .ok());
  client_context->set_credentials(
      ::grpc::AccessTokenCredentials(response.access_token()));
}

} // namespace auth
} // namespace framework
} // namespace cloud
} // namespace cartographer
