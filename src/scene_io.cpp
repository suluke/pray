#include "scene.hpp"

#include "ext.hpp"

#include <fstream>

bool Scene::load(const std::string &filename, IntDimension2 *out_image_resolution)
{
	clear();

	json json_input;

	{
		std::ifstream in(filename);
		if(!in.good())
		{
			std::cerr << "could not open input file " << filename << "\n";
			return false;
		}

		json_input = json::parse(in);
	}

	if(out_image_resolution)
	{
		*out_image_resolution = IntDimension2(json_input["resolution_x"], json_input["resolution_y"]);
	}

	if(!loadObj(json_input["obj_file"].get<std::string>())) return false;

	for (auto &l : json_input["lights"])
	{
		const auto light_position = Vector3(l["position"][0], l["position"][1], l["position"][2]);
		const auto light_color = Color(l["color"][0], l["color"][1], l["color"][2]);
		lights.emplace_back(light_position, light_color);
	}

	camera.position = Vector3(json_input["camera_position"][0], json_input["camera_position"][1], json_input["camera_position"][2]);
	camera.direction = Vector3(json_input["camera_look"][0], json_input["camera_look"][1], json_input["camera_look"][2]);
	camera.fov = json_input["fov"].get<float>() * 3.14159265f / 180.f;// convert to radians

	background_color = Color(json_input["background"][0], json_input["background"][1], json_input["background"][2]);

	return true;
}

bool Scene::loadObj(const std::string &filename)
{
	std::string error;
	tinyobj::attrib_t attrib;
	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;

	bool success = tinyobj::LoadObj(&attrib, &shapes, &materials, &error, filename.c_str());

	if(!error.empty())
	{
		std::cerr << "error loading obj file: " << error << "\n";
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
			const MaterialIndex material_index = insertMaterial(diffuse_color);

			std::array<Vector3, 3> vertices;
			for (size_t v = 0; v < fv; v++) {
				auto idx = s.mesh.indices[index_offset + v];
				vertices[v] = Vector3(attrib.vertices[3 * idx.vertex_index + 0], attrib.vertices[3 * idx.vertex_index + 1], attrib.vertices[3 * idx.vertex_index + 2]);
			}

			insertTriangle(vertices, material_index);

			index_offset += fv;
		}
	}

	return true;
}
