#ifndef PRAY_SCENE_H
#define PRAY_SCENE_H
#pragma once

#include "debug.hpp"
#include "math.hpp"
#include "image.hpp"
#include "cuda.h"

#include <cstdint>
#include <limits>
#include <array>
#include <vector>
#include <memory>

using TriangleIndex = uint32_t;
using MaterialIndex = uint32_t;

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

	Triangle() = default;
	Triangle(const std::array<Vector3, 3> &vertices, MaterialIndex material_index) : vertices(vertices), material_index(material_index) {}

	Vector3 calculateNormal() const
	{
		// clockwise order...
		return (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]).normalize();
	}

	Vector3 calculateCentroid() const
	{
		return (vertices[0] + vertices[1] + vertices[2]) / 3.f;
	}

	AABox3 calculateAabb() const
	{
		return AABox3(vertices.begin(), vertices.end());
	}
};

struct Material
{
	Color color;

	Material(Color color) : color(color) {}
};

struct EmissionMaterial : public Material {
	const bool isEmission;

	EmissionMaterial(Color color, bool isEmission = false) : Material(color), isEmission(isEmission) {}
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
	float fov = acos(-1.f) / 2.f;

	void calculateFrustumVectors(float aspect, Vector3 *left, Vector3 *right, Vector3 *bottom, Vector3 *top) const
	{
		const Vector3 global_up(0.f, 1.f, 0.f);
		const Vector3 local_left = direction.cross(global_up).normalize();
		const Vector3 local_up = local_left.cross(direction).normalize();

		*left   = local_left   * tan(fov/2.f);
		*right  = -local_left  * tan(fov/2.f);
		*top    = local_up     * tan(fov/2.f) * aspect;
		*bottom = -local_up    * tan(fov/2.f) * aspect;
	}
};

struct json_fwd;

struct RenderOptions {
	enum RenderMethod {
		WHITTED, PATH
	};
	struct Whitted {
	};
	struct Path {
		size_t num_samples;
		size_t max_depth;
	};

	IntDimension2 resolution = {1920, 1080};
	RenderMethod method = WHITTED;
	std::unique_ptr<json_fwd> json;
	std::string filename;
	// FIXME use C++17 std::variant so things actually explode if you try to
	// use the incorrect option
	union {
		Whitted whitted_opts;
		Path path_opts;
	};

	RenderOptions();
	~RenderOptions();
};

template<class material_t>
struct Scene
{
	std::vector<Triangle> triangles;
	std::vector<material_t> materials;

	std::vector<Light> lights;

	Camera camera;
	Color background_color = Color(0.f, 0.f, 0.f);

	void clear() { *this = Scene(); }

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

using WhittedScene = Scene<Material>;
using PathScene = Scene<EmissionMaterial>;

bool LoadJob(std::string filename, RenderOptions *out_opts);
bool LoadScene(const RenderOptions &opts, WhittedScene *scene);
bool LoadScene(const RenderOptions &opts, PathScene *scene);

#endif
