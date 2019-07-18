#pragma once
#include <cuda_runtime.h>
#include "cutil_math.h"

namespace loo
{

#define COLOR_SDF

	struct __align__(8) SDF_t {
		inline __host__ __device__ SDF_t() {}
		inline __host__ __device__ SDF_t(float v) : val(v), w(1) {}
		inline __host__ __device__ SDF_t(float v, float w) : val(v), w(w) {}

		inline __host__ __device__ operator float() const {
			return val;
		}
		inline __host__ __device__ void Clamp(float minval, float maxval) {
			val = clamp(val, minval, maxval);
		}
		inline __host__ __device__ void LimitWeight(float max_weight) {
			w = fminf(w, max_weight);
		}
		inline __host__ __device__ void operator+=(const SDF_t& rhs)
		{
			if(rhs.w > 0) {
				val = (w * val + rhs.w * rhs.val);
				w += rhs.w;
				val /= w;
			}
		}

		float val;
		float w;
	};

	inline __host__ __device__ SDF_t operator+(const SDF_t& lhs, const SDF_t& rhs)
	{
		SDF_t res = lhs;
		res += rhs;
		return res;
	}

}
