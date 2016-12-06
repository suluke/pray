#include "scene.hpp"
#include "image.hpp"

#include "naive_tracer.hpp"

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

	Image image(image_resolution);

	// assuming fov is in x direction, otherwise invert this
	const float aspect = (float)image.resolution.h / image.resolution.w;

	Vector3 left, right, bottom, top;
	scene.camera.calculateFrustumVectors(aspect, &left, &right, &bottom, &top);

	NaiveTracer tracer(scene);

#ifdef _MSC_VER // msvc does not support uint32_t as index variable in for loop
	#pragma omp parallel for
	for(intmax_t y = 0; y < image.resolution.h; ++y)
#else
	#pragma omp parallel for
	for(uint32_t y = 0; y < image.resolution.h; ++y)
#endif
	{
		for(uint32_t x = 0; x < image.resolution.w; ++x)
		{
			const float i_x = (float)x / (image.resolution.w-1);
			const float i_y = (float)y / (image.resolution.h-1);

			Ray ray(scene.camera.position, (left * (1.f - i_x) + right * i_x + top * (1.f - i_y) + bottom * i_y).normalize());

			Color c = tracer.trace(ray);
			image.setPixel(x, y, c);
		}
	}

	image.save(argv[2]);
}
