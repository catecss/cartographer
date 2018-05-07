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

#include "cartographer/cloud/internal/pose_uploader.h"

namespace cartographer {
namespace cloud {
namespace {

class PoseUploader : public PoseUploaderInterface {
public:
  PoseUploader(const std::string &uplink_server_address,
               bool enable_ssl_encryption);
  ~PoseUploader();

  // Starts the upload thread.
  void Start() final;

  // Shuts down the upload thread. This method blocks until the shutdown is
  // complete.
  void Shutdown() final;

  void
  EnqueueLocalToGlobalPose(common::Time time,
                           const transform::Rigid3d &local_to_global) override;
  void EnqueueTrackingToLocalPose(
      common::Time time, const transform::Rigid3d &tracking_to_local) override;
};

PoseUploader::PoseUploader(const std::string &uplink_server_address,
                           bool enable_ssl_encryption) {}

PoseUploader::~PoseUploader() {}

void PoseUploader::Start() {}

void PoseUploader::Shutdown() {}

void PoseUploader::EnqueueLocalToGlobalPose(
    common::Time time, const transform::Rigid3d &local_to_global) {}

void PoseUploader::EnqueueTrackingToLocalPose(
    common::Time time, const transform::Rigid3d &local_to_global) {}

} // namespace
} // namespace cloud
} // namespace cartographer
