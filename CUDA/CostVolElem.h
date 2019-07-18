#pragma once

#include <cuda_runtime.h>

namespace loo
{

	struct __align__(8) CostVolElem
	{
		inline __host__ __device__
			operator float() {
				return n > 0 ? sum / n : 1E30;
		}

		int n;
		float sum;
	};

}
