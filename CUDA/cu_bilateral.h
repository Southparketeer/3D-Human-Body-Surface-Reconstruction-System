#pragma once 
#include "Image.h"
typedef unsigned int uint;
namespace loo
{
	template<typename To, typename Ti>
	 __declspec(dllexport)
	void BilateralFilter( Image<To> dOut, const Image<Ti> dIn, float gs, float gr, uint size);
}
