#ifndef PRAY_SCENE_H
#define PRAY_SCENE_H
#pragma once

#include "debug.hpp"
#include "math.hpp"
#include "image.hpp"

#include <cstdint>
#include <limits>
#include <array>
#include <vector>

typedef uint32_t TriangleIndex;
typedef uint32_t MaterialIndex;

constexpr auto TriangleIndex_Invalid = std::numeric_limits<TriangleIndex>::max();
constexpr auto MaterialIndex_Invalid = std::numeric_limits<MaterialIndex>::max();

template<class T, class U>
inline T index_cast(U index)
{
	ASSERT(index >= std::numeric_limits<T>::min());
	ASSERT(index <= std::numeric_limits<T>::max());
	return static_cast<T>(index);
}

struct Triangle
{
	std::array<Vector3, 3> vertices;
	MaterialIndex material_index;

	Triangle(const std::array<Vector3, 3> &vertices, MaterialIndex material_index) : vertices(vertices), material_index(material_index) {}

	Vector3 calculateNormal() const
	{
		// clockwise order...
		return (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]).normalize();
	}
};

struct Material
{
	Color color;

	Material(Color color) : color(color) {}
};

struct Light
{
	Vector3 position;
	Color color;

	Light(const Vector3 &position, const Color &color) : position(position), color(color) {}
};

struct Camera
{
	Vector3 position = Vector3(0.f, 0.f, 0.f);
	Vector3 direction = Vector3(0.f, 0.f, -1.f);
	float fov = PI / 2.f;

	void calculateFrustumVectors(float aspect, Vector3 *left, Vector3 *right, Vector3 *bottom, Vector3 *top) const
	{
		const Vector3 global_up(0.f, 1.f, 0.f);
		const Vector3 local_right = global_up.cross(direction).normalize();
		const Vector3 local_up = direction.cross(local_right).normalize();

		*left   = -local_right * tan(fov/2.f);
		*right  = local_right  * tan(fov/2.f);
		*bottom = -local_up    * tan(fov/2.f) * aspect;
		*top    = local_up     * tan(fov/2.f) * aspect;
	}
};

struct Ray
{
	Vector3 origin;
	Vector3 direction;

	Ray(const Vector3 &_origin, const Vector3 &_direction) : origin(_origin), direction(_direction) {}
};

struct Scene
{
	std::vector<Triangle> triangles;
	std::vector<Material> materials;

	std::vector<Light> lights;

	Camera camera;
	Color background_color = Color(0.f, 0.f, 0.f);

	void clear() { *this = Scene(); }

	bool load(const std::string &filename, IntDimension2 *out_image_resolution = nullptr);

	template<class... Args>
	TriangleIndex insertTriangle(Args&&... args)
	{
		triangles.emplace_back(std::forward<Args>(args)...);
		return index_cast<TriangleIndex>(triangles.size() - 1u);
	}

	template<class... Args>
	MaterialIndex insertMaterial(Args&&... args)
	{
		materials.emplace_back(std::forward<Args>(args)...);
		return index_cast<MaterialIndex>(materials.size() - 1u);
	}

	template<class... Args>
	void insertLight(Args&&... args)
	{
		lights.emplace_back(std::forward<Args>(args)...);
	}
};
#endif
