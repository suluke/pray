#include "scene.hpp"

#include "ext.hpp"

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

	std::vector<unsigned char> image(3 * image_resolution.w * image_resolution.h);

	// assuming fov is in x direction, otherwise invert this
	const float aspect = (float)image_resolution.h / image_resolution.w;

	Vector3 left, right, bottom, top;
	scene.camera.calculateFrustumVectors(aspect, &left, &right, &bottom, &top);

	for(uint32_t y = 0; y < image_resolution.h; ++y)
	{
		//#pragma omp parallel for
		for(uint32_t x = 0; x < image_resolution.w; ++x)
		{
			const float i_x = (float)x / (image_resolution.w-1);
			const float i_y = (float)y / (image_resolution.h-1);

			Ray ray(scene.camera.position, (left * (1.f - i_x) + right * i_x + top * (1.f - i_y) + bottom * i_y).normalize());

			Color c = scene.trace(ray);
			image[3 * (y * image_resolution.w + x) + 0] = c.r * 255;
			image[3 * (y * image_resolution.w + x) + 1] = c.g * 255;
			image[3 * (y * image_resolution.w + x) + 2] = c.b * 255;
		}
	}

	//TODO: make Image class and put this into extra function?
	int write_error = stbi_write_bmp(argv[2], image_resolution.w, image_resolution.h, 3, image.data());
	if(write_error == 0)
	{
		std::cout << "could not write output image" << "\n";
	}
}
