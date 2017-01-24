#include "cuda_pathtracer.hpp"
#include "cuda_ray.hpp"
#include "cuda.hpp"

#include <cuda_runtime_api.h>
#include <cuda.h>


__global__ void d_kernel(CudaPathTracer* d_tracer, CudaImageView* image)
{
	d_tracer->d_render(*image);
}

__device__ void CudaPathTracer::d_render(CudaImageView image)
{
	Vector3 left, right, bottom, top;
	const float aspect = (float) image.resolution.h / image.resolution.w;
	scene.camera.calculateFrustumVectors(aspect, &left, &right, &bottom, &top);

	float max_x = (float) image.resolution.w;
	float max_y = (float) image.resolution_org.h;

	long local_y = ray_t::dim::h * blockDim.y;
	long x = ray_t::dim::w * blockDim.x;

	auto y = image.getGlobalY(local_y);

	auto direction = Vector3((top * (1.f - (2 * y + 1) / max_y) + left * (1.f - (2 * x + 1) / max_x) + d_camera->direction).normalize());
	ray_t ray(d_camera->position, direction);
	
	Color c = d_trace(ray);
	
	image.setPixel(x, local_y, c);
}

void CudaPathTracer::render(ImageView &image)
{	
	CudaImageView cudaImageView(image);
	cudaImageView.initialize();
	
	// copy objects to device
	CudaPathTracer* d_tracer = cuda::create<CudaPathTracer>(*this);
	CudaImageView* d_image = cuda::create<CudaImageView>(image);
	
	// configure execution
	// max. 1024 Threads per Block (may ask API)
	// Blocks are assigned to GPU processors -> if there are less GPUs than blocks, one GPU has to calculate several blocks
	dim3 dimGrid(image.resolution.w, image.resolution.h); // BLOCKS per grid (size of a grid)
	
	// start kernel
	//      <<< BLOCKS, THREADS >>>
	d_kernel<<<dimGrid, 1>>>(d_tracer, d_image);
	
	// destroy objects on device
	cuda::destroy<CudaPathTracer>(d_tracer);
	cuda::destroy<CudaImageView>(d_image);
	
	// copy back and free image memory
	cudaImageView.finalize();
}

void CudaPathTracer::initialize()
{
	// allocate memory and copy data
	d_nodes = cuda::vector<Node>::create(bih.nodes);
	d_scene_aabb = cuda::create<AABox3>(bih.scene_aabb);
	
	d_triangles = cuda::vector<Triangle>::create(scene.triangles);
	d_materials = cuda::vector<material_t>::create(scene.materials);
	d_camera = cuda::create<Camera>(scene.camera);
}

void CudaPathTracer::finalize()
{
	// free memory
	d_nodes.destroy();
	cuda::destroy<AABox3>(d_scene_aabb);
	
	d_triangles.destroy();
	d_materials.destroy();
	cuda::destroy<Camera>(d_camera);
}
