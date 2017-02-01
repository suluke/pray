#include "pray/Config.h" // This should always be first
#include "scene.hpp"
#include "image.hpp"
#include "parallel_worker.hpp"
#include "dummy_acceleration.hpp"
#include "bih.hpp"
#include "kdtree.hpp"
#include "cpu_tracer.hpp"
#include "cpu_pathtracer.hpp"
#include "ray.hpp"
#include "sse_ray.hpp"
#include "sampler.hpp"

#ifdef WITH_CUDA
	#include "cuda_pathtracer.hpp"
	#include "cuda_bih.hpp"
	#include "cuda_dummy_acceleration.hpp"
#endif

#include "logging.hpp" // This should always be last

#include <thread>
#include <vector>
#include <atomic>

template<class scene_t>
struct PrayTypes {
#ifdef WITH_SSE
	using ray_t = SSERay<scene_t>;
#else
	using ray_t = Ray<scene_t>;
#endif
	using accel_t = ACCELERATOR<ray_t, scene_t>;
#ifdef WITH_CUDA
	using accel_cuda_t = ACCELERATOR_CUDA;
#endif
	template <class tracer_t>
	using sampler_t = SAMPLER<scene_t, tracer_t, ray_t>;
};
#ifndef WITH_SSE_PT
template<>
struct PrayTypes<PathScene> {
	using scene_t = PathScene;
	using ray_t = Ray<scene_t>;
	using accel_t = ACCELERATOR<ray_t, scene_t>;
#ifdef WITH_CUDA
	using accel_cuda_t = ACCELERATOR_CUDA;
#endif
	template <class tracer_t>
	using sampler_t = SAMPLER<scene_t, tracer_t, ray_t>;
};
#endif // not WITH_SSE_PT

using WhittedTypes = PrayTypes<WhittedScene>;
using PathTypes = PrayTypes<PathScene>;

static void traceScene(const WhittedScene &scene, Image &image, const WhittedTypes::accel_t &accel, const RenderOptions &opts) {
  ImageView img(image, 0, opts.resolution.h);
  
	auto tracer = CpuTracer<WhittedTypes::ray_t, WhittedTypes::accel_t>(scene, accel);
	using sampler = WhittedTypes::sampler_t<decltype(tracer)>;
	sampler::render(scene, img, tracer);
}

static void traceScene(const PathScene &scene, Image &image, const PathTypes::accel_t &accel, const RenderOptions &opts) {
#ifdef WITH_CUDA
	static const unsigned int view_height = 8 * 8;

	std::vector<ImageView> views;
	std::atomic<size_t> views_idx{0};
	
	for (unsigned int i = 0; i < opts.resolution.h; i += view_height)
	{
		ImageView img(image, i, std::min(i + view_height, opts.resolution.h));
		views.push_back(img);
	}
	
	auto cpuTracer = CpuPathTracer< PathTypes::ray_t, PathTypes::accel_t >(scene, opts.path_opts, accel);
	using sampler = PathTypes::sampler_t<decltype(cpuTracer)>;
	auto cudaTracer  = CudaPathTracer< PathTypes::accel_cuda_t>(scene, opts.path_opts, accel.pod);
	
	int cuda_num = 0;
	int cpu_num = 0;
	
	std::thread cpuThread([&] () {
		while (1)
		{
			size_t idx = views_idx.fetch_add(1, std::memory_order_relaxed);
			if (idx >= views.size())
				break;
				
			ImageView &currentView = views[idx];
			sampler::render(scene, currentView, cpuTracer);
			
			cpu_num++;
		}
	});
	std::thread cudaThread([&] () {
		while (1)
		{
			size_t idx = views_idx.fetch_add(1, std::memory_order_relaxed);
			if (idx >= views.size())
				break;
				
			ImageView &currentView = views[idx];
			cudaTracer.render(currentView);
			
			cuda_num++;
		}
	});
	
	cpuThread.join();
	cudaThread.join();
	
	std::cout << cpu_num << " parts processed by CPU / " << cuda_num << " parts processed by GPU\n";
#else
  ImageView img(image, 0, opts.resolution.h);
	
	auto cpuTracer = CpuPathTracer< PathTypes::ray_t, PathTypes::accel_t >(scene, opts.path_opts, accel);
	using sampler = PathTypes::sampler_t<decltype(cpuTracer)>;
	sampler::render(scene, img, cpuTracer);
#endif
}

template<class scene_t>
static int trace(const char *outpath, RenderOptions &opts, StageLogger &logger) {
	Image image(opts.resolution);
#ifdef WITH_PROGRESS
	logger.image = &image;
#endif
	scene_t scene;
	if (!LoadScene(opts, &scene)) return 1;
	if (scene.triangles.empty()) {
		image.fill(scene.background_color);
		image.save(outpath);
		return 0;
	}

	const auto thread_pool_size = std::max(std::thread::hardware_concurrency(), 1u);
	std::cout << "creating thread pool with " << thread_pool_size << " threads.\n";
	ThreadPool thread_pool(thread_pool_size);

	logger.startPreprocessing();
	typename PrayTypes<scene_t>::accel_t accel;
	accel.build(scene, thread_pool);

	logger.startRendering();
#ifndef DISABLE_RENDERING
	traceScene(scene, image, accel, opts);
#else
	// fix "unused function traceScene"
	// because traceScene is overloaded, the static_cast is needed to specify which overload we want to (void)traceScene
	(void)static_cast<void(*)(const scene_t&, ImageView&, const typename PrayTypes<scene_t>::accel_t&, const RenderOptions&)>(traceScene);
#endif

	logger.startOutput();
#ifndef DISABLE_OUTPUT
	image.save(outpath);
#endif

	logger.finish();
	logger.log();
	return 0;
}

using namespace std;
int main(int argc, char *argv[])
{
	if (argc != 3) {
		cerr << "usage: " << argv[0] << " <input.json> <output.bmp>\n";
		return 1;
	}
	RenderOptions opts;
	StageLogger logger(opts);
	logger.dump_config();
	logger.start();
	if(!LoadJob(argv[1], &opts)) return 1;

	switch (opts.method) {
		case RenderOptions::WHITTED: {
			return trace<WhittedScene>(argv[2], opts, logger);
		}
		case RenderOptions::PATH: {
			return trace<PathScene>(argv[2], opts, logger);
		}
		default:
			return 1;
	}
	return 0;
}
