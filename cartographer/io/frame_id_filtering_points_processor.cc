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

#include "cartographer/io/frame_id_filtering_points_processor.h"

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/io/points_batch.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {

std::unique_ptr<FrameIdFilteringPointsProcessor>
FrameIdFilteringPointsProcessor::FromDictionary(
    common::LuaParameterDictionary* dictionary,
    PointsProcessor* next) {
  return common::make_unique<FrameIdFilteringPointsProcessor>(
      dictionary->GetDictionary("keep_frames")->GetArrayValuesAsStrings(),
      dictionary->GetDictionary("drop_frames")->GetArrayValuesAsStrings(),
      next);
}

FrameIdFilteringPointsProcessor::FrameIdFilteringPointsProcessor(
    const std::vector<string>& keep_frame_ids,
    const std::vector<string>& drop_frame_ids,
    PointsProcessor* next)
    : keep_frame_ids_(keep_frame_ids),
      drop_frame_ids_(drop_frame_ids),
      next_(next) {
  CHECK(keep_frame_ids.empty() || drop_frame_ids.empty())
      << "You may either specify the `keep_frames` property or the "
      << "`drop_frames` property, but not both at the same time.";
}

void FrameIdFilteringPointsProcessor::Process(
    std::unique_ptr<PointsBatch> batch) {
  if (std::find(keep_frame_ids_.begin(), keep_frame_ids_.end(),
                batch->frame_id) != keep_frame_ids_.end() ||
      std::find(drop_frame_ids_.begin(), drop_frame_ids_.end(),
                batch->frame_id) == drop_frame_ids_.end()) {
    next_->Process(std::move(batch));
  }
}

PointsProcessor::FlushResult FrameIdFilteringPointsProcessor::Flush() {
  return next_->Flush();
}

}  // namespace io
}  // namespace cartographer
