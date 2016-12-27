#include "scene.hpp"

#include "ext.hpp"

#include <experimental/filesystem>
#include <fstream>

using namespace std;
namespace fs = std::experimental::filesystem;

struct json_fwd {
	// Cannot forward-declare nlohmann::json itself because it is a typedef
	// to a template with default arguments 
	nlohmann::json json;
};
// This makes std::unique_ptr of forwared-declared json possible
RenderOptions::RenderOptions() = default;
RenderOptions::~RenderOptions() = default;

static inline void addMaterials(WhittedScene &scene, const vector<tinyobj::material_t> &materials) {
	for(auto &m : materials)
	{
		const Color diffuse_color = Color(m.diffuse[0], m.diffuse[1], m.diffuse[2]);
		scene.insertMaterial(diffuse_color);
	}
}
static inline void addMaterials(PathScene &scene, const vector<tinyobj::material_t> &materials) {
	for(auto &m : materials)
	{
		const Color diffuse_color = Color(m.diffuse[0], m.diffuse[1], m.diffuse[2]);
		const Color emission = Color(m.emission[0], m.emission[1], m.emission[2]);
		scene.insertMaterial(diffuse_color, emission);
	}
}

template<class scene_t>
static bool loadObj(scene_t &scene, const fs::path &file)
{
	string error;
	tinyobj::attrib_t attrib;
	vector<tinyobj::shape_t> shapes;
	vector<tinyobj::material_t> materials;

	//Get directory part of filename
	fs::path dir = file.parent_path();
	dir += fs::path::preferred_separator;

	bool success = tinyobj::LoadObj(&attrib, &shapes, &materials, &error, file.u8string().c_str(), dir.u8string().c_str());

	if(!error.empty())
	{
		cerr << "error loading obj file: " << error << "\n";
	}

	if(!success) return false;

	addMaterials(scene, materials);

	for(auto &s : shapes)
	{
		size_t index_offset = 0;

		for(size_t f = 0; f < s.mesh.num_face_vertices.size(); ++f)
		{
			auto fv = s.mesh.num_face_vertices[f];

			// only triangles are supported
			ASSERT(fv == 3);

			const MaterialIndex material_index = s.mesh.material_ids[f];

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

template<class scene_t>
static bool LoadSceneCommon(const RenderOptions &opts, scene_t *scene) {
	scene->clear();
	auto &json_input = opts.json->json;
	fs::path file(opts.filename);
	fs::path obj(json_input["obj_file"].get<string>());
	if (obj.is_relative()) {
		obj = file.parent_path() / obj;
	}
	if(!loadObj(*scene, obj)) return false;

	for (auto &l : json_input["lights"])
	{
		const auto light_position = Vector3(l["position"][0], l["position"][1], l["position"][2]);
		const auto light_color = Color(l["color"][0], l["color"][1], l["color"][2]);
		scene->insertLight(light_position, light_color);
	}

	scene->camera.position = Vector3(json_input["camera_position"][0], json_input["camera_position"][1], json_input["camera_position"][2]);
	scene->camera.direction = Vector3(json_input["camera_look"][0], json_input["camera_look"][1], json_input["camera_look"][2]).normalize();
	scene->camera.fov = json_input["fov"].get<float>() * acos(-1) / 180.f; // convert to radians

	scene->background_color = Color(json_input["background"][0], json_input["background"][1], json_input["background"][2]);
	return true;
}

bool LoadScene(const RenderOptions &opts, WhittedScene *scene) {
	return LoadSceneCommon(opts, scene);
}

bool LoadScene(const RenderOptions &opts, PathScene *scene) {
	return LoadSceneCommon(opts, scene);
}

bool LoadJob(string filename, RenderOptions *out_opts)
{
	out_opts->json = std::make_unique<json_fwd>();
	auto &json_input = out_opts->json->json;

	{
		ifstream in(filename);
		if(!in.good())
		{
			cerr << "could not open input file " << filename << "\n";
			return false;
		}

		json_input = json::parse(in);
	}

	if (json_input.find("resolution_x") != json_input.end() && json_input.find("resolution_y") != json_input.end()) {
		out_opts->resolution = {json_input["resolution_x"], json_input["resolution_y"]};
	}
	if (json_input.find("method") != json_input.end()) {
		auto method = json_input["method"].get<string>();
		if (method == "path_tracing") {
			out_opts->method = RenderOptions::PATH;
			if (json_input.find("num_samples") != json_input.end())
				out_opts->path_opts.num_samples = json_input["num_samples"];
			if (json_input.find("max_depth") != json_input.end())
				out_opts->path_opts.max_depth = json_input["max_depth"];
		} else {
			out_opts->method = RenderOptions::WHITTED;
		}
	}

	out_opts->filename = std::move(filename);
	return true;
}
