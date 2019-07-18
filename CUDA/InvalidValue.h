#pragma once

#include <cuda_runtime.h>

#ifdef __CUDACC__
#  include <math_constants.h>
#else
#  include <limits>
# include <math.h>
#endif

namespace loo
{


#ifdef _MSC_VER
#define INFINITY (DBL_MAX+DBL_MAX)
#define NAN (INFINITY-INFINITY)
#endif
	template<typename T>
	struct InvalidValue;

	template<>
	struct InvalidValue<float> {

		inline __host__ __device__ static float Value() {
#ifdef _MSC_VER
			return NAN;
#else
			return 0.0f / 0.0f;
#endif // _MSC_VER
		}

		/*
		#ifdef __CUDACC__
		inline __device__ static float Value() {
		return CUDART_NAN_F;
		}
		#else
		inline __host__ static float Value() {
		return std::numeric_limits<float>::quiet_NaN();
		}
		#endif
		*/
		inline __host__ __device__ static bool isfinite(float arg)
		{
			return arg == arg && 
				arg != std::numeric_limits<float>::infinity() &&
				arg != -std::numeric_limits<float>::infinity();
		}

		inline __host__ __device__ static bool IsValid(float val) {

			return isfinite(val);
		}
	};

	template<>
	struct InvalidValue<char> {
		inline __host__ __device__ static char Value() {
			return 0;
		}

		inline __host__ __device__ static bool IsValid(unsigned char val) {
			return !val;
		}
	};


	template<>
	struct InvalidValue<unsigned char> {
		inline __host__ __device__ static unsigned char Value() {
			return 0;
		}

		inline __host__ __device__ static bool IsValid(unsigned char val) {
			return !val;
		}
	};

	template<>
	struct InvalidValue<int> {
		inline __host__ __device__ static int Value() {
			return -1;
		}

		inline __host__ __device__ static bool IsValid(int val) {
			return val >= 0;
		}
	};

} // namespace
