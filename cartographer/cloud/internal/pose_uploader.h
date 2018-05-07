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

#ifndef CARTOGRAPHER_CLOUD_INTERNAL_POSE_UPLOADER_H
#define CARTOGRAPHER_CLOUD_INTERNAL_POSE_UPLOADER_H

#include <memory>
#include <set>
#include <string>

#include "cartographer/cloud/proto/map_builder_service.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/trajectory_builder_interface.h"

namespace cartographer {
namespace cloud {

class PoseUploaderInterface {
public:
  virtual ~PoseUploaderInterface() = default;

  // Starts the upload thread.
  virtual void Start() = 0;

  // Shuts down the upload thread. This method blocks until the shutdown is
  // complete.
  virtual void Shutdown() = 0;

  virtual void
  EnqueueLocalToGlobalPose(common::Time time,
                           const transform::Rigid3d &local_to_global) = 0;
  virtual void
  EnqueueTrackingToLocalPose(common::Time time,
                             const transform::Rigid3d &tracking_to_local) = 0;
};

// Returns PoseUploader with the actual implementation.
std::unique_ptr<PoseUploaderInterface>
CreatePoseUploader(const std::string &uplink_server_address,
                   bool enable_ssl_encryption);

} // namespace cloud
} // namespace cartographer

#endif // CARTOGRAPHER_CLOUD_INTERNAL_POSE_UPLOADER_H
