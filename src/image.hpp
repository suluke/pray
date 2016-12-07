#ifndef PRAY_IMAGE_H
#define PRAY_IMAGE_H
#pragma once

#include "math.hpp"
#include "ext.hpp"

struct Image
{
	IntDimension2 resolution;
	std::vector<uint8_t> pixels;
	using dim_t = IntDimension2::dim_t;

	Image(const IntDimension2 &res) : resolution(res), pixels(3 * resolution.w * resolution.h) {}

	void setPixel(dim_t x, dim_t y, const Color &c)
	{
		ASSERT(x < resolution.w); ASSERT(y < resolution.h);
		ASSERT(c.r >= 0.f); ASSERT(c.g >= 0.f); ASSERT(c.b >= 0.f);

		pixels[3 * (y * resolution.w + x) + 0] = std::round(std::min(c.r, 1.f) * 255.f);
		pixels[3 * (y * resolution.w + x) + 1] = std::round(std::min(c.g, 1.f) * 255.f);
		pixels[3 * (y * resolution.w + x) + 2] = std::round(std::min(c.b, 1.f) * 255.f);
	}

	bool save(const std::string &filename)
	{
		int write_error = stbi_write_bmp(filename.c_str(), resolution.w, resolution.h, 3, pixels.data());
		if(write_error == 0)
		{
			std::cerr << "could not write output image" << "\n";
			return false;
		}
		return true;
	}
};

/**
 * Defines a sub-view on the image (buffer).
 * In other words: Same as an image, but only operating on a part of a real image
 */
struct ImageView {
	Image &img;
	using dim_t = IntDimension2::dim_t;
	dim_t min_y;
	dim_t max_y;
	IntDimension2 resolution;
	ImageView(Image &img, IntDimension2::dim_t min_y, IntDimension2::dim_t max_y) : img(img), min_y(min_y), max_y(max_y) {
		ASSERT(min_y < max_y); // don't allow empty views
		resolution = {img.resolution.w, max_y - min_y};
	}

	void setPixel(dim_t x, dim_t y, const Color &c)
	{
		ASSERT(y + min_y < max_y);
		img.setPixel(x, y + min_y, c);
	}

	dim_t getGlobalY(dim_t y) {
		return y + min_y;
	}
};

#endif
