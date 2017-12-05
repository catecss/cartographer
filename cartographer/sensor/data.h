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

#ifndef CARTOGRAPHER_MAPPING_DATA_H_
#define CARTOGRAPHER_MAPPING_DATA_H_

#include "cartographer/common/make_unique.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/global_trajectory_builder_interface.h"
#include "cartographer/mapping/trajectory_builder.h"
#include "cartographer/sensor/fixed_frame_pose_data.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace sensor {

class Data {
 public:
   Data(const std::string &sensor_id) : sensor_id_(sensor_id) {}
   virtual ~Data() {}

   virtual common::Time GetTime() const = 0;
   const std::string &GetSensorId() const { return sensor_id_; }
   virtual void AddToTrajectoryBuilder(
       mapping::GlobalTrajectoryBuilderInterface *trajectory_builder) = 0;

 protected:
   const std::string sensor_id_;
};

template <typename DataType>
class Dispatchable : public Data {
 public:
   Dispatchable(const std::string &sensor_id, const DataType &data)
       : Data(sensor_id), data_(data) {}

   common::Time GetTime() const override { return data_.time; }
   void AddToTrajectoryBuilder(mapping::GlobalTrajectoryBuilderInterface
                                   *const trajectory_builder) override {
     trajectory_builder->AddSensorData(sensor_id_, data_);
  }
  void AddToTrajectoryBuilder(
      mapping::TrajectoryBuilder* const trajectory_builder,
      const std::string& sensor_id) override;

 private:
  const DataType data_;
};

template <typename DataType>
std::unique_ptr<Dispatchable<DataType>>
MakeDispatchable(const std::string &sensor_id, const DataType &data) {
  return common::make_unique<Dispatchable<DataType>>(sensor_id, data);
}

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_DATA_H_
