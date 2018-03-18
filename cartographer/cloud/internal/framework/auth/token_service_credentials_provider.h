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

#ifndef CARTOGRAPHER_CLOUD_INTERNAL_FRAMEWORK_AUTH_TOKEN_SERVICE_CREDENTIALS_PROVIDER_H
#define CARTOGRAPHER_CLOUD_INTERNAL_FRAMEWORK_AUTH_TOKEN_SERVICE_CREDENTIALS_PROVIDER_H

#include <string>

#include "cartographer/cloud/internal/framework/auth/credentials_provider.h"
#include "grpc++/grpc++.h"

namespace cartographer {
namespace cloud {
namespace framework {
namespace auth {

class TokenServiceCredentialsProvider : public CredentialsProvider {
public:
  TokenServiceCredentialsProvider(const std::string &token_server_address);

  TokenServiceCredentialsProvider(const TokenServiceCredentialsProvider &) =
      delete;
  TokenServiceCredentialsProvider &
  operator=(const TokenServiceCredentialsProvider &) = delete;

  void SetCredentials(::grpc::ClientContext *client_context) override;

private:
  std::shared_ptr<::grpc::Channel> channel_;
};

} // namespace auth
} // namespace framework
} // namespace cloud
} // namespace cartographer

#endif // CARTOGRAPHER_CLOUD_INTERNAL_FRAMEWORK_AUTH_TOKEN_SERVICE_CREDENTIALS_PROVIDER_H
