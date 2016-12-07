#include "scene.hpp"

#include "ext.hpp"

#include <fstream>
#include <experimental/filesystem>

using namespace std;
namespace fs = std::experimental::filesystem;

static bool loadObj(Scene &scene, const fs::path &file)
{
	string error;
	tinyobj::attrib_t attrib;
	vector<tinyobj::shape_t> shapes;
	vector<tinyobj::material_t> materials;

	//Get directory part of filename
	fs::path dir = file.parent_path();
	dir += fs::path::preferred_separator;

	bool success = tinyobj::LoadObj(&attrib, &shapes, &materials, &error, file.c_str(), dir.c_str());

	if(!error.empty())
	{
		cerr << "error loading obj file: " << error << "\n";
	}

	if(!success) return false;

	for(auto &s : shapes)
	{
		size_t index_offset = 0;

		for(size_t f = 0; f < s.mesh.num_face_vertices.size(); ++f)
		{
			auto fv = s.mesh.num_face_vertices[f];

			// only triangles are supported
			ASSERT(fv == 3);

			//TODO: this is shit, improve please!
			const Color diffuse_color = Color(materials[s.mesh.material_ids[f]].diffuse[0], materials[s.mesh.material_ids[f]].diffuse[1], materials[s.mesh.material_ids[f]].diffuse[2]);
			const MaterialIndex material_index = scene.insertMaterial(diffuse_color);

			array<Vector3, 3> vertices;
			for (size_t v = 0; v < fv; v++) {
				auto idx = s.mesh.indices[index_offset + v];
				vertices[v] = Vector3(attrib.vertices[3 * idx.vertex_index + 0], attrib.vertices[3 * idx.vertex_index + 1], attrib.vertices[3 * idx.vertex_index + 2]);
			}

			scene.insertTriangle(vertices, material_index);

			index_offset += fv;
		}
	}

	return true;
}

bool Scene::load(const string &filename, IntDimension2 *out_image_resolution)
{
	clear();

	json json_input;

	{
		ifstream in(filename);
		if(!in.good())
		{
			cerr << "could not open input file " << filename << "\n";
			return false;
		}

		json_input = json::parse(in);
	}

	if(out_image_resolution)
	{
		*out_image_resolution = IntDimension2(json_input["resolution_x"], json_input["resolution_y"]);
	}

	fs::path file(filename);
	fs::path obj(json_input["obj_file"].get<string>());
	if (obj.is_relative()) {
		obj = file.parent_path() / obj;
	}
	if(!loadObj(*this, obj)) return false;

	for (auto &l : json_input["lights"])
	{
		const auto light_position = Vector3(l["position"][0], l["position"][1], l["position"][2]);
		const auto light_color = Color(l["color"][0], l["color"][1], l["color"][2]);
		insertLight(light_position, light_color);
	}

	camera.position = Vector3(json_input["camera_position"][0], json_input["camera_position"][1], json_input["camera_position"][2]);
	camera.direction = Vector3(json_input["camera_look"][0], json_input["camera_look"][1], json_input["camera_look"][2]);
	camera.fov = json_input["fov"].get<float>() * PI / 180.f; // convert to radians

	background_color = Color(json_input["background"][0], json_input["background"][1], json_input["background"][2]);

	return true;
}
