#include "cuda_pathtracer.hpp"
#include "cuda_ray.hpp"
#include "cuda.hpp"

#include <cuda_runtime_api.h>
#include <cuda.h>


__global__ void d_kernel(CudaPathTracer* d_obj)
{
	d_obj->d_render();
}

__device__ void CudaPathTracer::d_render()
{
	
}

void CudaPathTracer::render(ImageView &image)
{	
	// allocate memory for the image
	uint8_t* d_image;
	uint8_t* pixels_start = image.img.pixels.data() + sizeof(uint8_t) * 3 * image.resolution.w * image.min_y;
	unsigned int pixels_size = sizeof(uint8_t) * 3 * image.resolution.w * image.resolution.h;
	cudaMalloc((void**) d_image, pixels_size);
	
	// copy CudaPathTracer Object to device
	CudaPathTracer* d_obj = cuda::create<CudaPathTracer>(*this);
	
	// configure execution
  // max. 1024 Threads per Block (may ask API)
  // Blocks are assigned to GPU processors -> if there are less GPUs than blocks, one GPU has to calculate several blocks
  dim3 dimGrid(1, 1);           // BLOCKS per grid (size of a grid)
  dim3 dimBlock(image.resolution.w, image.resolution.h); // THREADS per block (size of one block)
  
  // start kernel
	//      <<< BLOCKS, THREADS >>>
	d_kernel<<<dimGrid, dimBlock>>>(d_obj);
	
	// destroy CudaPathTracer copy on device
	cuda::destroy<CudaPathTracer>(d_obj);
	
	// copy back image results
	cudaMemcpy(pixels_start, d_image, pixels_size, cudaMemcpyDeviceToHost);
	
	// free image memory
	cudaFree(d_image);
}

void CudaPathTracer::initialize()
{
	// allocate memory and copy data
	d_nodes = cuda::vector<Node>::create(bih.nodes);
	d_triangles = cuda::vector<Triangle>::create(scene.triangles);
	d_materials = cuda::vector<material_t>::create(scene.materials);
	d_scene_aabb = cuda::create<AABox3>(bih.scene_aabb);
}

void CudaPathTracer::finalize()
{
	// free memory
	d_nodes.destroy();
	d_triangles.destroy();
	d_materials.destroy();
	cuda::destroy<AABox3>(d_scene_aabb);
}
