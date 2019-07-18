#pragma once 
#include "Image.h"
typedef unsigned int uint;
namespace loo
{
	template<typename To, typename Ti>
	 __declspec(dllexport)
	void MediumFilter( Image<To> dOut, const Image<Ti> dIn,  uint size, float near, float far);
}
