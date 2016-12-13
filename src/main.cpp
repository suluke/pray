#include "scene.hpp"
#include "image.hpp"
#include "dummy_acceleration.hpp"
#include "bih.hpp"
#include "cpu_tracer.hpp"
#include "math.hpp"

#ifdef WITH_SSE
	#include "sse_ray.hpp"
	using ray_t = SSERay;
#else
	#include "ray.hpp"
	using ray_t = Ray;
#endif
#ifdef WITH_BIH
	using accel_t = Bih<ray_t>;
#else
	using accel_t = DummyAcceleration<ray_t>;
#endif

#ifdef DEBUG
 #include <chrono>
#endif
int main(int argc, char *argv[])
{
	if (argc != 3)
	{
		std::cerr << "usage: " << argv[0] << " <input.json> <output.bmp>\n";
		return 1;
	}

	Scene scene;

	IntDimension2 image_resolution = IntDimension2(1920, 1080);
	if(!scene.load(argv[1], &image_resolution)) return 1;
#ifdef DEBUG
	auto start1 = std::chrono::high_resolution_clock::now();
#endif
	Image image(image_resolution);
	ImageView img(image, 0, image_resolution.h);
	CpuTracer<ray_t, accel_t> tracer(scene);
#ifdef DEBUG
	auto start2 = std::chrono::high_resolution_clock::now();
#endif
	tracer.preprocess();
#ifdef DEBUG
	auto point1 = std::chrono::high_resolution_clock::now();
#endif
	tracer.render(img);
#ifdef DEBUG
	auto end1 = std::chrono::high_resolution_clock::now();
#endif
	image.save(argv[2]);

#ifdef DEBUG
	auto end2 = std::chrono::high_resolution_clock::now();
	std::cout <<"Preprocess Time: "<< std::chrono::duration_cast<std::chrono::nanoseconds>(point1-start2).count() << "ns\n";
	std::cout <<"Render Time: "<< std::chrono::duration_cast<std::chrono::microseconds>(end1-point1).count() << "ms\n";
	std::cout <<"Total Time: "<< std::chrono::duration_cast<std::chrono::microseconds>(end2-start1).count() << "ms\n";
#endif
}
