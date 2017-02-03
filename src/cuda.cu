#include "cuda_pathtracer.hpp"
#include "cuda_lib.hpp"
#include "cuda_renderer.hpp"
#include "cuda_ray.hpp"
#include "cuda_scene.hpp"
#include "cuda_image.hpp"
#include "cuda_bih.hpp"
#include "cuda_dummy_acceleration.hpp"
#include "cuda_kdtree.hpp"

#include <cuda_runtime_api.h>
#include <curand_kernel.h>

using namespace std;

/* device and global functions */

template<class accel_cuda_t>
__global__ void kernel_render(CudaRenderer<accel_cuda_t>* d_renderer, CudaImage* d_image)
{
	// check if out of bounds
	// blockDim = threads per block
	if (blockIdx.x * blockDim.x + threadIdx.x > d_image->viewResolution.w)
		return;
	if (blockIdx.y * blockDim.y + threadIdx.y > d_image->viewResolution.h)
		return;
	
	d_renderer->render(d_image);
}

template<class accel_cuda_t>
__device__ void CudaRenderer<accel_cuda_t>::render(CudaImage* image)
{
	Vector3 left, right, bottom, top;
	const float aspect = (float) image->resolution.h / image->resolution.w;
	scene.camera->calculateFrustumVectors(aspect, &left, &right, &bottom, &top);

	float max_x = (float) image->resolution.w;
	float max_y = (float) image->resolution.h;
	
	long local_y = blockIdx.y * blockDim.y + threadIdx.y;
	long x = blockIdx.x * blockDim.x + threadIdx.x;

	auto y = image->getGlobalY(local_y);
	
	auto direction = Vector3((top * (1.f - (2 * y + 1) / max_y) + left * (1.f - (2 * x + 1) / max_x) + scene.camera->direction).normalize());
	ray_t ray(scene.camera->position, direction);
	
	Color c = trace(ray);
	image->setPixel(x, local_y, c);
}

template<class accel_cuda_t>
__device__ Color CudaRenderer<accel_cuda_t>::trace(CudaRay ray, unsigned int depth, curandState_t* randomState)
{
	if (depth == 0)
	{
		// initialize random generator for this block/thread
		curandState_t randomStatePerBlock;
		randomState = &randomStatePerBlock;
		sampling_init(randomState);
	}
	
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
		ray_t next(P, sampleHemisphere(X, Y, Z, randomState));
		value += trace(next, depth + 1, randomState);
	}
	value = material.color * value / opts.num_samples;
	return value;
}

template<class accel_cuda_t>
__device__ Vector3 CudaRenderer<accel_cuda_t>::sampleHemisphere(const Vector3 &X, const Vector3 &Y, const Vector3 &Z, curandState* randomState) 
{
	float u1 = sampling_rand(randomState);
	float u2 = sampling_rand(randomState);
	float r = sqrt(1.f - u1);
	float phi = 2 * acos(-1.f) * u2;
	float x = cos(phi) * r;
	float y = sin(phi) * r;
	float z = sqrt(u1);
	return X * x + Y * y + Z * z;
}

template<class accel_cuda_t>
__device__ inline void CudaRenderer<accel_cuda_t>::sampling_init(curandState_t* randomState)
{
	curand_init(0, /* the seed controls the sequence of random values that are produced */
							blockIdx.y * blockDim.y + threadIdx.y * blockIdx.x + threadIdx.x, /* the sequence number is only important with multiple cores */
							0, /* the offset is how much extra we advance in the sequence for each call, can be 0 */
							randomState);
}

template<class accel_cuda_t>
__device__ inline float CudaRenderer<accel_cuda_t>::sampling_rand(curandState_t* randomState)
{
	return curand_uniform(randomState);
}

__device__ typename CudaBih::ray_t::intersect_t CudaBih::intersect(const typename CudaBih::scene_t &scene, const typename CudaBih::ray_t &ray, typename CudaBih::ray_t::distance_t *out_distance) const
{
	const auto active_mask = ray.intersectAABB(*scene_aabb);
	if(!active_mask) return ray_t::getNoIntersection();

	const Vector3 direction_sign = ray.getSubrayDirection(ray_t::subrays_count / 2).sign();

	bool direction_sign_equal[3];
	ray.isDirectionSignEqualForAllSubrays(direction_sign, &direction_sign_equal[0]);

	// move to global namespace and use this intead of float *out_distance?
	struct IntersectionResult
	{
		typename ray_t::intersect_t triangle = ray_t::getNoIntersection();
		typename ray_t::distance_t distance = ray_t::max_distance();
	};

	IntersectionResult intersection_result;

	// keen this a VALUE (as opposed to reference)!!!
	const auto triangles_data = scene.triangles.data();

	Stack node_stack;

	Current current(nodes[0u], *scene_aabb, active_mask);

	for(;;)
	{
		if(current.node->getType() == Bih<ray_t, scene_t>::pod_t::Node::Leaf)
		{
			for(unsigned i = 0u; i < current.node->getLeafData().children_count; ++i)
			{
				const TriangleIndex triangle_index = current.node->getChildrenIndex() + i;
				const Triangle &triangle = triangles_data[triangle_index];

				typename ray_t::distance_t distance = ray_t::max_distance();
				const auto intersected = (ray.intersectTriangle(triangle, &distance) && current.active_mask);

				if(intersected)
				{
					// no need to pass current.active_mask here, updateIntersection handles this correctly
					// however, should ray_t ever get wider than one register, adding the mask might improve the performance if used correctly
					ray_t::updateIntersections(&intersection_result.triangle, triangle_index, &intersection_result.distance, distance);
				}
			}
		}
		else
		{
			const auto split_axis = current.node->getType();

			const auto &left_child = nodes[current.node->getChildrenIndex()+0];
			const auto &right_child = nodes[current.node->getChildrenIndex()+1];

			const auto left_plane = current.node->getSplitData().left_plane;
			const auto right_plane = current.node->getSplitData().right_plane;

			auto left_aabb = current.aabb;
			left_aabb.max[split_axis] = left_plane;
			auto right_aabb = current.aabb;
			right_aabb.min[split_axis] = right_plane;

			// Ignore direction_sign_equal[split_axis] here since the code would be the same. This is handeled on pop below.
			// The code in both cases is duplicated but with swapped left/right parameters. This gave a speedup of ~800ms on my machine with the sponza scene, no sse (lukasf)!
			if(direction_sign[split_axis] >= 0.f)
			{
				node_stack.push(StackElement(right_plane, split_axis, current.active_mask, right_child, right_aabb));

				const auto intersect_left = (ray.intersectAABB(left_aabb) && current.active_mask);
				if(intersect_left)
				{
					current = Current(left_child, left_aabb, intersect_left);
					continue;
				}
				else
				{
					//TODO: handle right_child here, don't push it on the stack and pop it directly afterwards down below
				}
			}
			else
			{
				node_stack.push(StackElement(left_plane, split_axis, current.active_mask, left_child, left_aabb));

				const auto intersect_right = (ray.intersectAABB(right_aabb) && current.active_mask);
				if(intersect_right)
				{
					current = Current(right_child, right_aabb, intersect_right);
					continue;
				}
				else
				{
					//TODO: handle left_child here, don't push it on the stack and pop it directly afterwards down below
				}
			}
		}

		for(;;)
		{
			if(node_stack.empty()) goto finish; // this goto is totally valid, I need to jump out of two loops!

			const auto &top = node_stack.pop(); // the reference is valid as long as nothing is pushed on the stack

			const auto smart_test_result = !direction_sign_equal[top.split_axis] || // always handle nodes for which !direction_sign_equal[split_axis]
				// using here that intersection_result.distance is default-initialized with ray_t::max_distance()
				(ray.intersectAxisPlane(top.plane, top.split_axis, intersection_result.distance) && top.active_mask);

			if(smart_test_result)
			{
				const auto intersect_node = (ray.intersectAABB(top.aabb) && top.active_mask);
				if(intersect_node)
				{
					current = Current(*top.node, top.aabb, top.active_mask);
					break;
				}
			}
		}
	}

	finish:

	*out_distance = intersection_result.distance;
	return intersection_result.triangle;
}


__device__ typename CudaDummyAcceleration::ray_t::intersect_t CudaDummyAcceleration::intersect(const typename CudaDummyAcceleration::scene_t &scene, const typename CudaDummyAcceleration::ray_t &ray, typename CudaDummyAcceleration::ray_t::distance_t *out_distance) const
{
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


__device__ typename CudaKdTree::ray_t::intersect_t CudaKdTree::intersect(const typename CudaKdTree::scene_t &scene, const typename CudaKdTree::ray_t &ray, typename CudaKdTree::ray_t::distance_t *out_distance) const
{
	auto active_mask = ray.intersectAABB(*scene_aabb);
	if(!active_mask) return ray_t::getNoIntersection();

	const Vector3 direction_sign = ray.getSubrayDirection(ray_t::subrays_count / 2).sign();
	
	bool direction_sign_equal[3];
	ray.isDirectionSignEqualForAllSubrays(direction_sign, &direction_sign_equal[0]);

	// move to global namespace and use this intead of float *out_distance?
	struct IntersectionResult
	{
		typename ray_t::intersect_t triangle = ray_t::getNoIntersection();
		typename ray_t::distance_t distance = ray_t::max_distance();
	};

	IntersectionResult intersection_result;

	// keep this a VALUE (as opposed to reference)!!!
	const auto triangles_data = scene.triangles.data();
	Stack node_stack;
	Current current(nodes[0u], *scene_aabb);

	for(;;)
	{
		if(current.node->getType() == KdTree<ray_t, scene_t>::pod_t::Node::Leaf)
		{
			const auto old_active_mask = active_mask;

			for(unsigned i = 0u; i < current.node->getLeafData().children_count; ++i)
			{
				const TriangleIndex triangle_index = current.node->getChildrenIndex() + i;
				const Triangle &triangle = triangles_data[triangle_index];

				typename ray_t::distance_t distance = ray_t::max_distance();
				const auto intersected = ray.intersectTriangle(triangle, &distance) && old_active_mask;

				if(intersected)
				{
					// no need to pass old_active_mask here, updateIntersection handles this correctly
					// however, should ray_t ever get wider than one register, adding the mask might improve the performance if used correctly
					ray_t::updateIntersections(&intersection_result.triangle, triangle_index, &intersection_result.distance, distance);

					//TODO: what to do about that?
					if(direction_sign_equal[0] && direction_sign_equal[1] && direction_sign_equal[2] && i < current.node->getLeafData().non_overlap_count)
					{
						active_mask = active_mask && !intersected;
					}
				}
			}
		}
		else
		{
			const auto split_axis = current.node->getType();

			const auto &left_child = nodes[current.node->getChildrenIndex()+0];
			const auto &right_child = nodes[current.node->getChildrenIndex()+1];

			const auto split_plane = current.node->getSplitData().plane;

			auto left_aabb = current.aabb;
			left_aabb.max[split_axis] = split_plane;
			auto right_aabb = current.aabb;
			right_aabb.min[split_axis] = split_plane;

			// Ignore direction_sign_equal[split_axis] here since the code would be the same. This is handeled on pop below.
			// The code in both cases is duplicated but with swapped left/right parameters. This gave a speedup of ~800ms on my machine with the sponza scene, no sse (lukasf)!
			if(direction_sign[split_axis] >= 0.f)
			{
				node_stack.push(StackElement(split_plane, split_axis, right_child, right_aabb));

				const auto intersect_left = ray.intersectAABB(left_aabb) && active_mask;
				if(intersect_left)
				{
					current = Current(left_child, left_aabb);
					continue;
				}
				else
				{
					//TODO: handle right_child here, don't push it on the stack and pop it directly afterwards down below
				}
			}
			else
			{
				node_stack.push(StackElement(split_plane, split_axis, left_child, left_aabb));

				const auto intersect_right = ray.intersectAABB(right_aabb) && active_mask;
				if(intersect_right)
				{
					current = Current(right_child, right_aabb);
					continue;
				}
				else
				{
					//TODO: handle left_child here, don't push it on the stack and pop it directly afterwards down below
				}
			}
		}

		for(;;)
		{
			if(node_stack.empty()) goto finish; // this goto is totally valid, I need to jump out of two loops!

			const auto &top = node_stack.pop(); // the reference is valid as long as nothing is pushed on the stack

			const auto smart_test_result = !direction_sign_equal[top.split_axis] || // always handle nodes for which !direction_sign_equal[split_axis]
				active_mask;

			if(smart_test_result)
			{
				const auto intersect_node = ray.intersectAABB(top.aabb) && active_mask;
				if(intersect_node)
				{
					current = Current(*top.node, top.aabb);
					break;
				}
			}
		}
	}

	finish:

	*out_distance = intersection_result.distance;
	return intersection_result.triangle;
}


/* host functions */
template<class accel_cuda_t>
struct CudaPathTracerImpl : public CudaPathTracer {
  using scene_t = PathScene;
  using material_t = EmissionMaterial;
        
	const CudaRenderer<accel_cuda_t> renderer;  // struct with device pointers
	CudaRenderer<accel_cuda_t>* d_renderer; // device pointer to struct

	template<class accel_pod_t>
  CudaPathTracerImpl(const PathScene &scene, const RenderOptions::Path &opts, const accel_pod_t &accel) : renderer(scene, accel, opts)
	{
		d_renderer = cuda::create<CudaRenderer<accel_cuda_t>>(renderer);
		
#ifdef DEBUG
		// device information output
		int nDevices;

		cudaGetDeviceCount(&nDevices);
		cuda::checkForError(__FILE__, __func__, __LINE__);
		
		for (int i = 0; i < nDevices; i++) {
			cudaDeviceProp prop;
			cudaGetDeviceProperties(&prop, i);
			printf("Device Number: %d\n", i);
			printf("  Device name: %s\n", prop.name);
			printf("  Memory Clock Rate (KHz): %d\n", prop.memoryClockRate);
			printf("  Memory Bus Width (bits): %d\n",prop.memoryBusWidth);
			printf("  Peak Memory Bandwidth (GB/s): %f\n",2.0*prop.memoryClockRate*(prop.memoryBusWidth/8)/1.0e6);
			printf("  Major revision number: %d\n", prop.major);
			printf("  Minor revision number: %d\n", prop.minor);
			printf("  Total global memory: %u bytes\n", prop.totalGlobalMem);
			printf("  Number of multiprocessors: %d\n", prop.multiProcessorCount);
			printf("  Total amount of shared memory per block: %u\n",prop.sharedMemPerBlock);
			printf("  Total registers per block: %d\n", prop.regsPerBlock);
			printf("  Warp size: %d\n", prop.warpSize);
			printf("  Maximum memory pitch: %u\n", prop.memPitch);
			printf("  Total amount of constant memory: %u\n\n", prop.totalConstMem);
		}
#endif
	}
  virtual ~CudaPathTracerImpl() override;

  virtual void render(ImageView &image) override;
};

unique_ptr<CudaPathTracer> CudaPathTracer::Create(const PathScene &scene, const RenderOptions::Path &opts, const DummyAccelerationPOD<PathScene> &accel) {
	using tracer_t = CudaPathTracerImpl<CudaDummyAcceleration>;
	return unique_ptr<CudaPathTracer>(new tracer_t(scene, opts, accel));
}
unique_ptr<CudaPathTracer> CudaPathTracer::Create(const PathScene &scene, const RenderOptions::Path &opts, const BihPOD<PathScene> &accel) {
	using tracer_t = CudaPathTracerImpl<CudaBih>;
	return unique_ptr<CudaPathTracer>(new tracer_t(scene, opts, accel));
}
unique_ptr<CudaPathTracer> CudaPathTracer::Create(const PathScene &scene, const RenderOptions::Path &opts, const KdTreePOD<PathScene> &accel) {
	using tracer_t = CudaPathTracerImpl<CudaKdTree>;
	return unique_ptr<CudaPathTracer>(new tracer_t(scene, opts, accel));
}


template<class accel_cuda_t>
CudaPathTracerImpl<accel_cuda_t>::~CudaPathTracerImpl()
{
	cuda::destroy<CudaRenderer<accel_cuda_t>>(d_renderer);
}

template<class accel_cuda_t>
void CudaPathTracerImpl<accel_cuda_t>::render(ImageView &image)
{
	CudaImage cudaImage(image);
	
	// copy objects to device
	CudaImage* d_image = cuda::create<CudaImage>(cudaImage);
	
	#ifdef DEBUG
		// check free memory on device
		size_t cuda_mem_free;
		size_t cuda_mem_total;
		cudaMemGetInfo(&cuda_mem_free, &cuda_mem_total);
		cout << cuda_mem_free << "/" << cuda_mem_free << " Bytes currently free on device" << endl;
	#endif
	
	// adjust stack size limit according
	size_t stack_size_wish = CudaBih::stack_size + renderer.opts.max_depth * 2048 + 1024; // add some number for the other stack frames
	size_t stack_size_old;
	size_t stack_size_new;
	
	cudaDeviceGetLimit(&stack_size_old, cudaLimitStackSize);
	cuda::checkForError(__FILE__, __func__, __LINE__);
	
	cudaDeviceSetLimit(cudaLimitStackSize, stack_size_wish);
	cuda::checkForError(__FILE__, __func__, __LINE__);
	
	cudaDeviceGetLimit(&stack_size_new, cudaLimitStackSize);
	cuda::checkForError(__FILE__, __func__, __LINE__);
	
	#ifdef DEBUG
		cout << "cudaDeviceSetLimit set from " << stack_size_old << " to " << stack_size_new << " (wanted " << stack_size_wish << ")" << endl;
	#endif
	
	const unsigned int block_length = 8;
	
	// configure execution
	// max. 1024 Threads per Block (may ask API)
	// Blocks are assigned to GPU processors -> if there are less GPUs than blocks, one GPU has to calculate several blocks
	dim3 dimGrid(ceil(image.resolution.w / (float) block_length), ceil(image.resolution.h / (float) block_length)); // BLOCKS per grid (size of a grid)
	dim3 dimBlock(block_length, block_length); // THREADS per block
	
	// start kernel
	// <<< BLOCKS, THREADS >>>
	kernel_render<accel_cuda_t><<<dimGrid, dimBlock>>>(d_renderer, d_image);
	cuda::checkForError(__FILE__, __func__, __LINE__);
	
	// destroy objects on device
	cuda::destroy<CudaImage>(d_image);
	
	// copy back resulting image
	cudaImage.copyBack();
}
