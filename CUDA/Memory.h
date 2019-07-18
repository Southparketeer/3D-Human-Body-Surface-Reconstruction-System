#ifndef CUDAMEMORY_H
#define CUDAMEMORY_H

#include <cuda_runtime.h>
#include <thrust/device_vector.h>
#include <sstream>
#include "config.h"
namespace loo
{
	struct CudaException : public std::exception
	{
		CudaException(const std::string& what, cudaError err = cudaSuccess) : mWhat(what), mErr(err) { }
		virtual ~CudaException() throw() {}
		virtual const char* what() const throw() {
			std::stringstream ss;
			ss << "CudaException: " << mWhat << std::endl;
			if(mErr != cudaSuccess) {
				ss << "cudaError code: " << cudaGetErrorString(mErr);
				ss << " (" << mErr << ")" << std::endl;
			}
			return ss.str().c_str();
		}
		std::string mWhat;
		cudaError mErr;
	};

	struct TargetHost
	{
		template<typename T> inline static
			void AllocatePitchedMem(T** hostPtr, size_t *pitch, size_t w, size_t h){
				*pitch = w*sizeof(T);
				const cudaError err = cudaMallocHost(hostPtr, *pitch * h);
				if( err != cudaSuccess ) {
					throw CudaException("Unable to cudaMallocHost", err);
				}
		}

		template<typename T> inline static
			void AllocatePitchedMem(T** hostPtr, size_t *pitch, size_t *img_pitch, size_t w, size_t h, size_t d){
				*pitch = w*sizeof(T);
				*img_pitch = *pitch*h;
				const cudaError err = cudaMallocHost(hostPtr, *pitch * h * d);
				if( err != cudaSuccess ) {
					throw CudaException("Unable to cudaMallocHost", err);
				}
		}

		template<typename T> inline static
			void DeallocatePitchedMem(T* hostPtr){
				cudaFreeHost(hostPtr);
		}
	};

	struct TargetDevice
	{
		template<typename T> inline static
			void AllocatePitchedMem(T** devPtr, size_t *pitch, size_t w, size_t h)
		{
			const cudaError err = cudaMallocPitch(devPtr,pitch,w*sizeof(T),h);
			if( err != cudaSuccess ) {
				throw CudaException("Unable to cudaMallocPitch", err);
			}
		}

		template<typename T> inline static
			void AllocatePitchedMem(T** devPtr, size_t *pitch, size_t *img_pitch, size_t w, size_t h, size_t d)
		{
			const cudaError err = cudaMallocPitch(devPtr,pitch,w*sizeof(T),h*d);
			if( err != cudaSuccess ) {
				throw CudaException("Unable to cudaMallocPitch", err);
			}
			*img_pitch = *pitch * h;
		}

		template<typename T> inline static
			void DeallocatePitchedMem(T* devPtr){
				cudaFree(devPtr);
		}
	};

#if CUDA_VERSION_MAJOR >= 6
	struct TargetManaged
	{
		template<typename T> inline static
			void AllocatePitchedMem(T** mgdPtr, size_t *pitch, size_t w, size_t h){
				*pitch = w*sizeof(T);
				const cudaError err = cudaMallocManaged(mgdPtr, *pitch * h);
				if( err != cudaSuccess ) {
					throw CudaException("Unable to cudaMallocManaged", err);
				}
		}

		template<typename T> inline static
			void AllocatePitchedMem(T** mgdPtr, size_t *pitch, size_t *img_pitch, size_t w, size_t h, size_t d){
				*pitch = w*sizeof(T);
				*img_pitch = *pitch*h;
				const cudaError err = cudaMallocManaged(mgdPtr, *pitch * h * d);
				if( err != cudaSuccess ) {
					throw CudaException("Unable to cudaMallocManaged", err);
				}
		}

		template<typename T> inline static
			void DeallocatePitchedMem(T* mgdPtr){
				cudaFree(mgdPtr);
		}
	};
#endif // CUDA_VERSION_MAJOR >= 6
	template<typename TargetTo, typename TargetFrom>
	cudaMemcpyKind TargetCopyKind() { return cudaMemcpyDefault; }

	template<> inline cudaMemcpyKind TargetCopyKind<TargetHost,TargetHost>() { return cudaMemcpyHostToHost;}
	template<> inline cudaMemcpyKind TargetCopyKind<TargetDevice,TargetHost>() { return cudaMemcpyHostToDevice;}
	template<> inline cudaMemcpyKind TargetCopyKind<TargetHost,TargetDevice>() { return cudaMemcpyDeviceToHost;}
	template<> inline cudaMemcpyKind TargetCopyKind<TargetDevice,TargetDevice>() { return cudaMemcpyDeviceToDevice;}

	#ifdef HAVE_THRUST
	template<typename T, typename Target> struct ThrustType;
	template<typename T> struct ThrustType<T,TargetHost> { typedef T* Ptr; };
	template<typename T> struct ThrustType<T,TargetDevice> { typedef thrust::device_ptr<T> Ptr; };

#if CUDA_VERSION_MAJOR >= 6
	// TODO: Check this.
	template<typename T> struct ThrustType<T,TargetManaged> { typedef thrust::device_ptr<T> Ptr; };
#endif

#endif // HAVE_THRUST

	struct Manage
	{
		inline static __host__
			void AllocateCheck()
		{
		}

		template<typename T, typename Target> inline static __host__
			void Cleanup(T* ptr)
		{
			if(ptr) {
				Target::template DeallocatePitchedMem<T>(ptr);
				ptr = 0;
			}
		}
	};

	struct DontManage
	{
		inline static __host__
			void AllocateCheck()
		{
			throw CudaException("Image that doesn't own data should not call this constructor");
		}

		template<typename T, typename Target> inline static __device__ __host__
			void Cleanup(T* /*ptr*/)
		{
		}
	};

	// Empty definition to trigger compile time error by default.
	template<typename ManagementTo, typename TargetTo, typename TargetFrom>
	__device__ __host__	void AssignmentCheck();

	// Define valid assignments
	template<> inline void AssignmentCheck<DontManage, TargetDevice, TargetDevice>() { }
	template<> inline void AssignmentCheck<DontManage, TargetHost,   TargetHost>() { }

#if CUDA_VERSION_MAJOR >= 6
	template<> inline void AssignmentCheck<DontManage, TargetManaged,TargetManaged>() { }
	template<> inline void AssignmentCheck<DontManage, TargetDevice, TargetManaged>() { }
	template<> inline void AssignmentCheck<DontManage, TargetHost,   TargetManaged>() { }
#endif
}

#endif // CUDAMEMORY_H