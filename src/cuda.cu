#include "cuda_pathtracer.hpp"
#include "cuda_lib.hpp"
#include "cuda_renderer.hpp"
#include "cuda_ray.hpp"
#include "cuda_scene.hpp"
#include "cuda_image.hpp"
#include "cuda_bih.hpp"

#include <cuda_runtime_api.h>
#include <cuda.h>
#include <curand_kernel.h>


/* device and global functions */

template<class accel_t, class accel_cuda_t>
__global__ void kernel_render(CudaRenderer<accel_t, accel_cuda_t>* d_renderer, CudaImage* d_image)
{
  d_renderer->render(d_image);
}

template<class accel_t, class accel_cuda_t>
__device__ void CudaRenderer<accel_t, accel_cuda_t>::render(CudaImage* image)
{
	// initialize random generator for this block/thread
	sampling_init();
	
	Vector3 left, right, bottom, top;
	const float aspect = (float) image->viewResolution.h / image->viewResolution.w;
	scene.camera->calculateFrustumVectors(aspect, &left, &right, &bottom, &top);

	float max_x = (float) image->viewResolution.w;
	float max_y = (float) image->resolution.h;
	
	long local_y = ray_t::dim::h * blockIdx.y;
	long x = ray_t::dim::w * blockIdx.x;

	auto y = image->getGlobalY(local_y);
  
	auto direction = Vector3((top * (1.f - (2 * y + 1) / max_y) + left * (1.f - (2 * x + 1) / max_x) + scene.camera->direction).normalize());
	ray_t ray(scene.camera->position, direction);
	
	Color c = trace(ray);
	image->setPixel(x, local_y, c);
}

template<class accel_t, class accel_cuda_t>
__device__ Color CudaRenderer<accel_t, accel_cuda_t>::trace(CudaRay ray, unsigned int depth)
{
  typename ray_t::distance_t intersection_distance;
  const auto intersected_triangle = accel.intersect(scene, ray, &intersection_distance);
	
  if (intersected_triangle == TriangleIndex_Invalid)
    return *(scene.background_color);

  const auto &triangle = scene.triangles[intersected_triangle];
  const auto material_index = triangle.material_index;
  auto &material = scene.materials[material_index];

  if (material.isEmission)
    return material.color;
  if (depth >= opts.max_depth)
    return Color {0, 0, 0};

  const auto N = ray_t::getNormals(scene, intersected_triangle);
  const auto X = (triangle.vertices[1] - triangle.vertices[0]).normalize();
  const auto Y = N.cross(X).normalize();
  const auto &Z = N;
  const auto P = ray.getIntersectionPoint(intersection_distance) + N * 0.0001f;

  Color value{0, 0, 0};
  for (unsigned i = 0; i < opts.num_samples; ++i) {
    ray_t next(P, sampleHemisphere(X, Y, Z));
    value += trace(next, depth + 1);
  }
  value = material.color * value / opts.num_samples;
  return value;
}

template<class accel_t, class accel_cuda_t>
__device__ Vector3 CudaRenderer<accel_t, accel_cuda_t>::sampleHemisphere(const Vector3 &X, const Vector3 &Y, const Vector3 &Z) 
{
  float u1 = sampling_rand();
  float u2 = sampling_rand();
  float r = std::sqrt(1.f - u1);
  float phi = 2 * std::acos(-1.f) * u2;
  float x = std::cos(phi) * r;
  float y = std::sin(phi) * r;
  float z = std::sqrt(u1);
  return X * x + Y * y + Z * z;
}

template<class accel_t, class accel_cuda_t>
__device__ inline void CudaRenderer<accel_t, accel_cuda_t>::sampling_init()
{
	curand_init(0, /* the seed controls the sequence of random values that are produced */
              blockIdx.x * blockDim.x + blockIdx.y, /* the sequence number is only important with multiple cores */
              0, /* the offset is how much extra we advance in the sequence for each call, can be 0 */
              &randomState);
}

template<class accel_t, class accel_cuda_t>
__device__ inline float CudaRenderer<accel_t, accel_cuda_t>::sampling_rand()
{
	return curand_uniform(&randomState);
}

__device__ typename CudaBih::ray_t::intersect_t CudaBih::intersect(const typename CudaBih::scene_t &scene, const typename CudaBih::ray_t &ray, typename CudaBih::ray_t::distance_t *out_distance) const
{
	// TODO: this is the simple intersect, not using the Bih so far!
	typename ray_t::intersect_t intersected_triangle = ray_t::getNoIntersection();
	auto minimum_distance = ray_t::max_distance();

	for(TriangleIndex triangle_index = 0u; triangle_index < index_cast<TriangleIndex>(scene.triangles.size()); ++triangle_index)
	{
		const Triangle &triangle = scene.triangles[triangle_index];

		typename ray_t::distance_t distance;
		if(ray.intersectTriangle(triangle, &distance))
		{
			ray_t::updateIntersections(&intersected_triangle, triangle_index, &minimum_distance, distance);
		}
	}

	*out_distance = minimum_distance;
	return intersected_triangle;
}



/* host functions */

template<class accel_t, class accel_cuda_t>
CudaPathTracer<accel_t, accel_cuda_t>::CudaPathTracer(const PathScene &scene, const RenderOptions::Path &opts, const accel_t &accel) : renderer(scene, accel.pod, opts)
{
	d_renderer = cuda::create<CudaRenderer<accel_pod_t, accel_cuda_t>>(renderer);
}

template<class accel_t, class accel_cuda_t>
CudaPathTracer<accel_t, accel_cuda_t>::~CudaPathTracer()
{
	cuda::destroy<CudaRenderer<accel_pod_t, accel_cuda_t>>(d_renderer);
}

template<class accel_t, class accel_cuda_t>
void CudaPathTracer<accel_t, accel_cuda_t>::render(ImageView &image)
{
	CudaImage cudaImage(image);
	
	// copy objects to device
	CudaImage* d_image = cuda::create<CudaImage>(cudaImage);
	
	// configure execution
	// max. 1024 Threads per Block (may ask API)
	// Blocks are assigned to GPU processors -> if there are less GPUs than blocks, one GPU has to calculate several blocks
	dim3 dimGrid(image.resolution.w, image.resolution.h); // BLOCKS per grid (size of a grid)
	
	// start kernel
	//      <<< BLOCKS, THREADS >>>
	kernel_render<accel_pod_t, accel_cuda_t><<<dimGrid, 1>>>(d_renderer, d_image);
  cuda::checkForError(__FILE__, __func__, __LINE__);
	
	// destroy objects on device
	cuda::destroy<CudaImage>(d_image);
	
	// copy back resulting image
	cudaImage.copyBack();
}


template class CudaPathTracer<Bih<Ray<Scene<EmissionMaterial> >, Scene<EmissionMaterial> >, CudaBih>;
template class CudaRenderer<BihPOD<Scene<EmissionMaterial> >, CudaBih>;
