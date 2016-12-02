#define STB_IMAGE_WRITE_IMPLEMENTATION
#define TINYOBJLOADER_IMPLEMENTATION
#include "pray.h"

#include <cassert>
#include <experimental/filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace fs = std::experimental::filesystem;

int main(int argc, char *argv[]) {
  using std::uint64_t;
  using vec = pray::Vec3<float>;

  if (argc != 3) {
    std::cout << "usage: " << argv[0] << " <input.json> <output.bmp>\n";
    return 1;
  }

  fs::path inputFileName(argv[1]);
  fs::path outputFileName(argv[2]);
  auto basePath = inputFileName.parent_path();

  std::ifstream fin(inputFileName);
  if (!fin.good()) {
    std::cerr << "Error opening file: " << inputFileName << "\n";
    return 1;
  }
  json json_input = json::parse(fin);
  fin.close();

  std::string error;
  tinyobj::attrib_t attrib;
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;

  // get full path to the obj file
  auto objFileName = basePath / json_input["obj_file"].get<std::string>();
  // base path of the mtl files
  auto mtlDir = basePath.string() + fs::path::preferred_separator;

  bool obj_succ = tinyobj::LoadObj(
      &attrib, &shapes, &materials, &error, objFileName.c_str(),
      (basePath.empty() ? nullptr : mtlDir.c_str()));

  // Warnings might be present even on success
  if (!error.empty())
    std::cerr << error << "\n";
  if (!obj_succ) {
    std::cerr << "Error loading file: " << json_input["obj_file"] << "\n";
    return 1;
  }

  // JSON input
  // Image resolution
  uint64_t resolution_x = json_input["resolution_x"];
  uint64_t resolution_y = json_input["resolution_y"];

  // Field of view is the angle between the rays for the rightmost and leftmost
  // pixels. The vertical fov is determined from the resolution ratio.
  uint64_t fov = json_input["fov"];

  // Position of the camera in the scene
  vec camera_position(json_input["camera_position"][0],
                      json_input["camera_position"][1],
                      json_input["camera_position"][2]);

  // Orientation of the camera in the scene. The y-axis of the camera view is
  // always the "up"-axis wrt. to the image.
  vec camera_look(json_input["camera_look"][0], json_input["camera_look"][1],
                  json_input["camera_look"][2]);

  // Load point lights
  for (auto &l : json_input["lights"]) {
    // TODO: we should probably do something with this:
    l["position"][0];
    l["position"][1];
    l["position"][2];
    l["color"][0];
    l["color"][1];
    l["color"][2];
    std::cout << l.dump() << "\n";
  }
  // Background color of the scene. Colors are given as floating point RGB
  // values in the interval [0.0, 1.0]
  vec backgroundColor(json_input["background"][0], json_input["background"][1],
                      json_input["background"][2]);

  std::vector<unsigned char> image(3 * resolution_x * resolution_y);

  /* tinyobj usage: */
  // Loop over shapes
  for (auto &s : shapes) {
    // Loop over faces(triangles)
    size_t index_offset = 0;
    for (size_t f = 0; f < s.mesh.num_face_vertices.size(); f++) {
      auto fv = s.mesh.num_face_vertices[f];
      assert(fv == 3);

      // Loop over vertices in the face.
      for (size_t v = 0; v < fv; v++) {
        // Access to vertex
        tinyobj::index_t idx = s.mesh.indices[index_offset + v];
        vec vertex(attrib.vertices[3 * idx.vertex_index + 0],
                   attrib.vertices[3 * idx.vertex_index + 1],
                   attrib.vertices[3 * idx.vertex_index + 2]);
      }
      index_offset += fv;

      // Per-face material, diffuse color.
      vec color = materials[s.mesh.material_ids[f]].diffuse;
    }
  }

  /* TODO: Ray trace here */

  int write_error = stbi_write_bmp(outputFileName.c_str(), resolution_x,
                                   resolution_y, 3, image.data());
  if (write_error == 0) {
    std::cerr << "stbi_write_bmp failed\n";
    return 1;
  }

  return 0;
}
