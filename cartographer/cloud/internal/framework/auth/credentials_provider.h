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

#ifndef CARTOGRAPHER_CLOUD_INTERNAL_FRAMEWORK_AUTH_CREDENTIALS_PROVIDER_H
#define CARTOGRAPHER_CLOUD_INTERNAL_FRAMEWORK_AUTH_CREDENTIALS_PROVIDER_H

#include "grpc++/grpc++.h"

namespace cartographer {
namespace cloud {
namespace framework {
namespace auth {

class CredentialsProvider {
public:
  CredentialsProvider() {}
  virtual ~CredentialsProvider() {}

  CredentialsProvider(const CredentialsProvider &) = delete;
  CredentialsProvider &operator=(const CredentialsProvider &) = delete;

  virtual void SetCredentials(::grpc::ClientContext *client_context) = 0;
};

} // namespace auth
} // namespace framework
} // namespace cloud
} // namespace cartographer

#endif // CARTOGRAPHER_CLOUD_INTERNAL_FRAMEWORK_AUTH_CREDENTIALS_PROVIDER_H
