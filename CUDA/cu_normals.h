#pragma once

#include "Image.h"

namespace loo
{
	__declspec(dllexport) void NormalsFromVbo(Image<float4> dN, const Image<float4> dV);
}
