#include "cuda_pathtracer.hpp"
#include "cuda_lib.hpp"
#include "cuda_renderer.hpp"
#include "cuda_ray.hpp"
#include "cuda_scene.hpp"
#include "cuda_image.hpp"
#include "cuda_bih.hpp"

#include <cuda_runtime_api.h>
#include <cuda.h>

template<class accel_t, class accel_cuda_t>
__global__ void d_kernel(CudaRenderer<accel_t, accel_cuda_t>* d_renderer, CudaImage* d_image)
{
  d_renderer->render(d_image);
}


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
	d_kernel<accel_pod_t, accel_cuda_t><<<dimGrid, 1>>>(d_renderer, d_image);
  cuda::checkForError(__FILE__, __func__, __LINE__);
	
	// destroy objects on device
	cuda::destroy<CudaImage>(d_image);
	
	// copy back resulting image
	cudaImage.copyBack();
}


template<class accel_t, class accel_cuda_t>
__device__ void CudaRenderer<accel_t, accel_cuda_t>::render(CudaImage* image)
{
	/*
	Vector3 left, right, bottom, top;
	const float aspect = (float) image.resolution.h / image.resolution.w;
	d_camera->calculateFrustumVectors(aspect, &left, &right, &bottom, &top);

	float max_x = (float) image.resolution.w;
	float max_y = (float) image.resolution_org.h;
	*/
	long local_y = ray_t::dim::h * blockIdx.y;
	long x = ray_t::dim::w * blockIdx.x;

	//auto y = image.getGlobalY(local_y);
  
	//auto direction = Vector3((top * (1.f - (2 * y + 1) / max_y) + left * (1.f - (2 * x + 1) / max_x) + d_camera->direction).normalize());
	//ray_t ray(d_camera->position, direction);
	
	//Color c = trace(ray);
  Color c;
	image->setPixel(x, local_y, c);
}

template<class accel_t, class accel_cuda_t>
__device__ Color CudaRenderer<accel_t, accel_cuda_t>::trace(CudaRay ray) // ray_t ray, unsigned depth
{
	/*
  typename ray_t::distance_t intersection_distance;
  const auto intersected_triangle = d_bih_intersect(ray, &intersection_distance);

  if (d_bih_isNoIntersection(intersected_triangle))
    return scene.background_color;

  const auto &triangle = d_triangles[intersected_triangle];
  const auto material_index = triangle.material_index;
  auto &material = d_materials[material_index];

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
    value += trace(scene, next, depth + 1);
  }
  value = material.color * value / opts.num_samples;
  return value;
	*/
  
  return Color();
}

template class CudaPathTracer<Bih<Ray<Scene<EmissionMaterial> >, Scene<EmissionMaterial> >, CudaBih>;
template class CudaRenderer<BihPOD<Scene<EmissionMaterial> >, CudaBih>;
