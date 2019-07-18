#pragma once

#include "Image.h"
#include "MatUtils.h"
#include "ImageIntrinsics.h"

namespace loo
{

template<typename T, typename Target = TargetDevice, typename Management = DontManage>
struct ImageKeyframe : public ImageTransformProject
{
    Image<T, loo::TargetDevice> img;
};

} // namespace roo
