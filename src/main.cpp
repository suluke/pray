#include "scene.hpp"
#include "image.hpp"
#include "dummy_acceleration.hpp"
#include "bih.hpp"
#include "cpu_tracer.hpp"

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

#ifdef WITH_TIMING
 #include <chrono>
#endif
using namespace std;
int main(int argc, char *argv[])
{
	if (argc != 3)
	{
		cerr << "usage: " << argv[0] << " <input.json> <output.bmp>\n";
		return 1;
	}

#ifdef DEBUG
	cout << "Warning: This is a Debug build and might be very slow!\n";
#endif

	Scene scene;

	cout << "Loading..." << endl;
	IntDimension2 image_resolution = IntDimension2(1920, 1080);
	if(!scene.load(argv[1], &image_resolution)) return 1;

	Image image(image_resolution);

	if(scene.triangles.empty())
	{
		// when the scene is empty, just fill the image with the background color and get outta here, no need to do any ray tracing stuff
		image.fill(scene.background_color);
		image.save(argv[2]);
		return 0;
	}

#ifdef WITH_TIMING
	auto start1 = chrono::high_resolution_clock::now();
#endif

	ImageView img(image, 0, image_resolution.h);

	CpuTracer<ray_t, accel_t> tracer(scene);

#ifdef WITH_TIMING
	auto start2 = chrono::high_resolution_clock::now();
#endif
	cout << "Preprocessing..." << endl;
	tracer.preprocess();
#ifdef WITH_TIMING
	auto point1 = chrono::high_resolution_clock::now();
#endif
	cout << "Rendering..." << endl;
	tracer.render(img);
#ifdef WITH_TIMING
	auto end1 = chrono::high_resolution_clock::now();
#endif
#ifndef DISABLE_OUTPUT
	cout << "Saving..." << endl;
	image.save(argv[2]);
#endif

#ifdef WITH_TIMING
	auto end2 = chrono::high_resolution_clock::now();
	cout <<"Preprocess Time: "<< chrono::duration_cast<chrono::nanoseconds>(point1-start2).count() << "ns\n";
	cout <<"Render Time: "<< chrono::duration_cast<chrono::milliseconds>(end1-point1).count() << "ms\n";
	cout <<"Total Time: "<< chrono::duration_cast<chrono::milliseconds>(end2-start1).count() << "ms\n";
#endif

	return 0;
}
