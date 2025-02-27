/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/fusion/app/obstacle_multi_sensor_fusion.h"

namespace apollo {
namespace perception {
namespace fusion {

bool ObstacleMultiSensorFusion::Init(
    const ObstacleMultiSensorFusionParam& param) {
  if (fusion_ != nullptr) {
    AINFO << "Already inited";
    return true;
  }
  // 获取 ProbabilisticFusion 类的实例指针
  BaseFusionSystem* fusion =
      BaseFusionSystemRegisterer::GetInstanceByName(param.fusion_method);
  fusion_.reset(fusion);

  FusionInitOptions init_options;
  init_options.main_sensor = param.main_sensor;

  // 多态调用 ProbabilisticFusion::Init 方法
  if (fusion_ == nullptr || !fusion_->Init(init_options)) {
    AINFO << "Failed to Get Instance or Initialize " << param.fusion_method;
    return false;
  }
  return true;
}

bool ObstacleMultiSensorFusion::Process(const base::FrameConstPtr& frame,
                                        std::vector<base::ObjectPtr>* objects) {
  FusionOptions options;
  return fusion_->Fuse(options, frame, objects);
}

PERCEPTION_REGISTER_MULTISENSORFUSION(ObstacleMultiSensorFusion);

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
