#pragma once

#include "debug.hpp"
#include "math.hpp"

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

	Triangle(const std::array<Vector3, 3> &_vertices, MaterialIndex _material_index) : vertices(_vertices), material_index(_material_index) {}

	Vector3 calculateNormal() const
	{
		return (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]).normalize();
	}
};

struct Material
{
	Color color;

	Material(Color _color) : color(_color) {}
};

struct Light
{
	Vector3 position;
	Color color;

	Light(const Vector3 &_position, const Color &_color) : position(_position), color(_color) {}
};

struct Camera
{
	Vector3 position = Vector3(0.f, 0.f, 0.f);
	Vector3 direction = Vector3(0.f, 0.f, -1.f);
	float fov = 90.f;

	void calculateFrustumVectors(float aspect, Vector3 *left, Vector3 *right, Vector3 *bottom, Vector3 *top)
	{
		const Vector3 global_up(0.f, 1.f, 0.f);
		const Vector3 local_right = direction.cross(global_up).normalize();
		const Vector3 local_up = local_right.cross(direction).normalize();

		*left = direction + -local_right * tan(fov/2.f);
		*right = direction + local_right * tan(fov/2.f);
		*bottom = direction + -local_up * tan(fov/2.f) * aspect;
		*top = direction + local_up * tan(fov/2.f) * aspect;
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

	TriangleIndex insertTriangle(const Triangle &triangle)
	{
		triangles.push_back(triangle);
		return index_cast<TriangleIndex>(triangles.size() - 1u);
	}

	MaterialIndex insertMaterial(const Material &material)
	{
		materials.push_back(material);
		return index_cast<MaterialIndex>(materials.size() - 1u);
	}

	void insertLight(const Light &light)
	{
		lights.push_back(light);
	}

	Color trace(const Ray &ray) const;

	private:
	bool loadObj(const std::string &filename);

	TriangleIndex intersectTriangle(const Ray &ray, float *out_distance) const;
};
