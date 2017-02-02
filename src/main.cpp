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
	#include "cuda_kdtree.hpp"
#endif

#include "logging.hpp" // This should always be last

#include <thread>
#include <atomic>

template<class scene_t>
struct PrayTypes {
#ifdef WITH_SSE
	using ray_t = SSERay<scene_t>;
#else
	using ray_t = Ray<scene_t>;
#endif
	using accel_t = ACCELERATOR<ray_t, scene_t>;
	using dummy_accel_t = DummyAcceleration<ray_t, scene_t>;
	template <class tracer_t>
	using sampler_t = SAMPLER<scene_t, tracer_t, ray_t>;
};
#ifndef WITH_SSE_PT
template<>
struct PrayTypes<PathScene> {
	using scene_t = PathScene;
	using ray_t = Ray<scene_t>;
	using accel_t = ACCELERATOR<ray_t, scene_t>;
	using dummy_accel_t = DummyAcceleration<ray_t, scene_t>;
	template <class tracer_t>
	using sampler_t = SAMPLER<scene_t, tracer_t, ray_t>;
};
#endif // not WITH_SSE_PT

using WhittedTypes = PrayTypes<WhittedScene>;
using PathTypes = PrayTypes<PathScene>;

template<class accel_t>
static void traceScene(const WhittedScene &scene, Image &image, const accel_t &accel, const RenderOptions &opts) {
  ImageView img(image, 0, opts.resolution.h);
  
	auto tracer = CpuTracer<WhittedTypes::ray_t, accel_t>(scene, accel);
	using sampler = WhittedTypes::sampler_t<decltype(tracer)>;
	sampler::render(scene, img, tracer);
}

template<class accel_t>
static void traceScene(const PathScene &scene, Image &image, const accel_t &accel, const RenderOptions &opts) {
#ifdef WITH_CUDA
	static const unsigned int cpu_block_size = 8 * 8;
	static const unsigned int cuda_block_size = 8 * 16;
	
	std::atomic<Image::dim_t> current_y{0};
	
	auto cpuTracer = CpuPathTracer< PathTypes::ray_t, accel_t >(scene, opts.path_opts, accel);
	using sampler = PathTypes::sampler_t<decltype(cpuTracer)>;
	auto cudaTracer  = CudaPathTracer< typename accel_t::accel_cuda_t>(scene, opts.path_opts, accel.pod);
	
	int cuda_num = 0;
	int cpu_num = 0;
	
	std::thread cpuThread([&] () {
		while (1)
		{
			Image::dim_t y = current_y.fetch_add(cpu_block_size, std::memory_order_relaxed);
			if (y >= image.resolution.h)
				break;
				
			ImageView currentView = ImageView(image, y, std::min(y + cpu_block_size, image.resolution.h));
			sampler::render(scene, currentView, cpuTracer);
			
			cpu_num++;
		}
	});
	std::thread cudaThread([&] () {
		while (1)
		{
			Image::dim_t y = current_y.fetch_add(cuda_block_size, std::memory_order_relaxed);
			if (y >= image.resolution.h)
				break;
				
			ImageView currentView = ImageView(image, y, std::min(y + cuda_block_size, image.resolution.h));
			cudaTracer.render(currentView);
			
			cuda_num++;
		}
	});
	
	cpuThread.join();
	cudaThread.join();
	
	std::cout << cpu_num << " parts processed by CPU / " << cuda_num << " parts processed by GPU\n";
#else
	ImageView img(image, 0, opts.resolution.h);
	
	auto cpuTracer = CpuPathTracer< PathTypes::ray_t, accel_t >(scene, opts.path_opts, accel);
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

	std::cout << scene.triangles.size() << "\n";

	if (scene.triangles.size() <= 256u) {
		typename PrayTypes<scene_t>::dummy_accel_t accel;

		logger.startRendering();
#ifdef DISABLE_RENDERING
		if (false) // avoids "unused function traceScene"
#endif
		traceScene(scene, image, accel, opts);
	} else {
		const auto thread_pool_size = std::max(std::thread::hardware_concurrency(), 1u);
		std::cout << "creating thread pool with " << thread_pool_size << " threads.\n";
		ThreadPool thread_pool(thread_pool_size);

		logger.startPreprocessing();
		typename PrayTypes<scene_t>::accel_t accel;
		accel.build(scene, thread_pool);

		logger.startRendering();
#ifdef DISABLE_RENDERING
		if (false) // avoids "unused function traceScene"
#endif
		traceScene(scene, image, accel, opts);
	}

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
