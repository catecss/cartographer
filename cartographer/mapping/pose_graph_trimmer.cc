/*
 * Copyright 2016 The Cartographer Authors
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

#include "cartographer/mapping/pose_graph_trimmer.h"
#include "cartographer/mapping_2d/pose_graph.h"

#include "glog/logging.h"

namespace cartographer {
namespace mapping {
/*
transform::Rigid3d ComputeRelativePose(const mapping::NodeId& first_node_id,
const mapping::NodeId& second_node_id, const mapping::MapById<mapping::NodeId,
PoseGraphInterface::MyConstraint>& my_constraints, const
mapping::MapById<mapping::NodeId, mapping::TrajectoryNode>& trajectory_nodes_) {
  if (second_node_id.node_index == first_node_id.node_index + 1) {
    // next to each other
  } else {
    // one missing
  }
}*/

void MyNodeTrimmer::Trim(Trimmable* pose_graph) {
  static int last_trimmed_node_id = 0;
  auto* handle =
      dynamic_cast<mapping_2d::PoseGraph::TrimmingHandle*>(pose_graph);
  auto* p = handle->pose_graph();

  LOG(INFO) << "LAST NODE "
            << p->trajectory_nodes_.EndOfTrajectory(0).operator--()->id;
  LOG(INFO) << "LAST NODE "
            << p->optimization_problem_.node_data_.EndOfTrajectory(0)
                   .
                   operator--()
                   ->id;

  return;

  // Make a copy of the old nodes.
  mapping::MapById<mapping::NodeId, mapping::TrajectoryNode> old_nodes =
      p->trajectory_nodes_;

  std::vector<mapping::NodeId> nodes_to_remove;
  for (const auto& node : p->trajectory_nodes_) {
    if (node.id.node_index < last_trimmed_node_id) {
      continue;
    }

    mapping::NodeId node_id = node.id;
    mapping::NodeId prev_node_id = node.id;
    --prev_node_id.node_index;
    mapping::NodeId next_node_id = node.id;
    ++next_node_id.node_index;

    if (!p->trajectory_nodes_.Contains(prev_node_id) ||
        !p->trajectory_nodes_.Contains(next_node_id) ||
        !p->optimization_problem_.node_data_.Contains(prev_node_id) ||
        !p->optimization_problem_.node_data_.Contains(next_node_id)) {
      continue;
    }
    last_trimmed_node_id = node.id.node_index;

    const auto& prev_node_data = p->trajectory_nodes_.at(prev_node_id);
    const auto& next_node_data = p->trajectory_nodes_.at(next_node_id);

    if (rand() % 3 == 0) {
      // Node is to be removed.

      // step 1 => find all constraints for this node.
      auto& c = p->constraints_;
      c.erase(
          std::remove_if(
              c.begin(), c.end(),
              [node_id,
               p](const mapping::PoseGraphInterface::Constraint& constraint) {
                bool shouldRemove = constraint.node_id == node_id;
                if (shouldRemove) {
                  auto submap_id = constraint.submap_id;
                  // auto submap_it = p->submap_data_.find(submap_id);
                  // CHECK(submap_it != p->submap_data_.end());
                  p->submap_data_.at(submap_id).node_ids.erase(node_id);
                }
                return shouldRemove;
              }),
          c.end());

      // step 2 => remove node
      nodes_to_remove.push_back(node_id);
    }  // rand % 3
  }    // nodes
  for (const auto& node_id : nodes_to_remove) {
    p->trajectory_nodes_.Trim(node_id);
    p->optimization_problem_.TrimTrajectoryNode(node_id);
  }

  // Now construct replacement constraints covering the holes in the trajectory.
  // plan: iterate over p->trajectory_nodes_ with prev and next.

  LOG(INFO) << "Trimming done.";
  LOG(INFO) << "LAST TRIMMED NODE: " << last_trimmed_node_id;
}

bool MyNodeTrimmer::IsFinished() { return false; }

PureLocalizationTrimmer::PureLocalizationTrimmer(const int trajectory_id,
                                                 const int num_submaps_to_keep)
    : trajectory_id_(trajectory_id), num_submaps_to_keep_(num_submaps_to_keep) {
  CHECK_GE(num_submaps_to_keep, 3);
}

void PureLocalizationTrimmer::Trim(Trimmable* const pose_graph) {
  if (pose_graph->IsFinished(trajectory_id_)) {
    num_submaps_to_keep_ = 0;
  }

  while (pose_graph->num_submaps(trajectory_id_) > num_submaps_to_keep_) {
    const int submap_index_to_trim_next = num_submaps_trimmed_;
    pose_graph->MarkSubmapAsTrimmed(
        SubmapId{trajectory_id_, submap_index_to_trim_next});
    ++num_submaps_trimmed_;
  }

  if (num_submaps_to_keep_ == 0) {
    finished_ = true;
  }
}

bool PureLocalizationTrimmer::IsFinished() { return finished_; }

}  // namespace mapping
}  // namespace cartographer
