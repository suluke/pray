#ifndef PRAY_IMAGE_H
#define PRAY_IMAGE_H
#pragma once

#include "math.hpp"
#include "ext.hpp"

struct Image
{
	IntDimension2 resolution;
	std::vector<uint8_t> pixels;

	Image(const IntDimension2 &res) : resolution(res), pixels(3 * resolution.w * resolution.h) {}

	void setPixel(uint32_t x, uint32_t y, const Color &c)
	{
		ASSERT(x < resolution.w); ASSERT(y < resolution.h);

		pixels[3 * (y * resolution.w + x) + 0] = c.r * 255;
		pixels[3 * (y * resolution.w + x) + 1] = c.g * 255;
		pixels[3 * (y * resolution.w + x) + 2] = c.b * 255;
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

#endif
