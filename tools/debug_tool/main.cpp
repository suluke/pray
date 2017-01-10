#define FREEGLUT_LIB_PRAGMAS 0
#include <GL/freeglut.h>

#include "scene.hpp"
#include "ray.hpp"
#include "bih.hpp"

#include <functional>
#include <random>

using namespace std;

typedef WhittedScene Scene_t;
static Scene_t scene;

static float camera_pan = 0.f, camera_tilt = 0.f;
static float camera_distance = 5.f;

static string debug_mode;
static string debug_mode_parameter;

// "bih"
typedef Ray<Scene_t> Ray_t;
typedef Bih<Ray_t, Scene_t> Bih_t;
static Bih_t bih;
static int bih_depth = 0; // "draw_all"
static Bih_t::Node *bih_current_node = nullptr; // "draw_single"
static Ray_t bih_ray(Vector3(0.f, 5.f, -5.f), Vector3(0.f, -1.f, 1.f).normalize()); // "intersect"
std::vector<size_t> bih_intersected_nodes;
static bool bih_draw_intersected_nodes = true;

static void bih_rebuild()
{
	auto current_node_index = bih.nodes.empty() ? 0 : bih_current_node - &bih.nodes[0];
	bih = Bih_t();
	bih.build(scene);
	bih_current_node = &bih.nodes[current_node_index];
}

static void init()
{
	glEnable(GL_CULL_FACE);
	glFrontFace(GL_CW);

	if(debug_mode == "bih")
	{
		bih_rebuild();
	}
}

static void keyboard(unsigned char key, int x, int y)
{
	if(key == 'a') camera_pan += 10.f;
	if(key == 'd') camera_pan -= 10.f;
	if(key == 'w') camera_tilt += 10.f;
	if(key == 's') camera_tilt -= 10.f;
	if(key == '+') camera_distance -= 0.3f;
	if(key == '-') camera_distance += 0.3f;
	if(debug_mode == "bih")
	{
		if(debug_mode_parameter == "draw_all")
		{
			if(key == 'i') ++bih_depth;
			if(key == 'k') --bih_depth;
		}
		else if(debug_mode_parameter == "draw_single")
		{
			auto old_current_node = bih_current_node;
			if(key == 'p') bih_current_node = bih_current_node->parent;
			if(key == 'l') bih_current_node = bih_current_node->child1;
			if(key == 'r') bih_current_node = bih_current_node->child2;
			if(key == 'f') ++bih_current_node;
			if(key == 'b') --bih_current_node;

			if(old_current_node != bih_current_node)
			{
				if(bih_current_node->getType() == Bih_t::Node::Leaf)
				{
					std::cout << "current: " << bih_current_node->index << " p: " << bih_current_node->parent->index << " Leaf with " << bih_current_node->getLeafData().children_count << " children" << "\n";
				}
				else
				{
					std::cout << "current: " << bih_current_node->index << " p: " << (bih_current_node->parent ? bih_current_node->parent->index : 13371337) << " l: " << bih_current_node->child1->index << " r: " << bih_current_node->child2->index << "\n";
				}
			}
		}
		else if(debug_mode_parameter == "intersect")
		{
			if(key == 'i') bih_draw_intersected_nodes = !bih_draw_intersected_nodes;
		}

		if(key == 'm')
		{
			static std::vector<std::string> parameters = { "draw_all", "draw_single", "intersect" };
			auto current = std::find(parameters.begin(), parameters.end(), debug_mode_parameter);
			if(current == parameters.end()) --current;
			auto next = ++current;
			if(current == parameters.end()) next = parameters.begin();
			debug_mode_parameter = *next;
		}

		if(key == 'r')
		{
			bih_rebuild();
		}
	}
	glutPostRedisplay();
}

static void reshape(int w, int h)
{
	glViewport(0, 0, w, h);

	const float aspect = (float)w / h;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glFrustum(-1.f * aspect, 1.f * aspect, -1.f, 1.f, 1.f, 1000.f);
	glMatrixMode(GL_MODELVIEW);
}

static void draw_triangle(const Triangle &triangle)
{
	glBegin(GL_TRIANGLES);
	glVertex3fv(&triangle.vertices[0].x);
	glVertex3fv(&triangle.vertices[1].x);
	glVertex3fv(&triangle.vertices[2].x);
	glEnd();
}

static void draw_box(const AABox3 &box)
{
	auto dx = box.max.x - box.min.x;
	auto dy = box.max.y - box.min.y;
	auto dz = box.max.z - box.min.z;
	auto center = box.min + (box.max - box.min) / 2.f;

	glPushMatrix();

	glTranslatef(center.x, center.y, center.z);
	glScalef(dx, dy, dz);
	glutWireCube(1.f);

	glPopMatrix();
}

static void draw_bih_node(const Bih_t::Node &node, const AABox3 &deduced_box, int depth)
{
	if(node.getType() == Bih_t::Node::Leaf)
	{
		if(debug_mode_parameter == "draw_all")
		{
			if(bih_depth == depth)
			{
				glColor3f(0.f, 1.f, 0.f);
				draw_triangle(scene.triangles[bih.triangles[node.getChildrenIndex()]]);
			}
		}
		else if(debug_mode_parameter == "draw_single")
		{
			if(depth == 1 || &node == bih_current_node)
			{
				glColor3f(0.f, 1.f, 0.f);
				draw_triangle(scene.triangles[bih.triangles[node.getChildrenIndex()]]);
			}
		}
		else if(debug_mode_parameter == "intersect")
		{
			if(bih_draw_intersected_nodes && std::find(bih_intersected_nodes.begin(), bih_intersected_nodes.end(), &node - &bih.nodes[0]) != bih_intersected_nodes.end())
			{
				glColor3f(0.f, 1.f, 0.f);
				draw_triangle(scene.triangles[bih.triangles[node.getChildrenIndex()]]);
			}
		}
	}
	else
	{
		auto child1_box = deduced_box;
		child1_box.max[node.getType()] = node.getSplitData().left_plane;
		auto child2_box = deduced_box;
		child2_box.min[node.getType()] = node.getSplitData().right_plane;

		if(debug_mode_parameter == "draw_all")
		{
			if(bih_depth == depth)
			{
				glColor3f(1.f, 0.f, 0.f);
				draw_box(child1_box);
				glColor3f(0.f, 0.f, 1.f);
				draw_box(child2_box);
			}

			draw_bih_node(bih.nodes[node.getChildrenIndex()+0], child1_box, depth+1);
			draw_bih_node(bih.nodes[node.getChildrenIndex()+1], child2_box, depth+2);
		}
		else if(debug_mode_parameter == "draw_single")
		{
			if(&node == bih_current_node)
			{
				glColor3f(1.f, 0.f, 0.f);
				draw_box(child1_box);
				glColor3f(0.f, 0.f, 1.f);
				draw_box(child2_box);

				glColor3f(1.f, 1.f, 1.f);
				glBegin(GL_POINTS);
				Vector3 v(0.f, 0.f, 0.f);
				v[node.getType()] = node.split_point;
				glVertex3fv(&v.x);
				glEnd();

				draw_bih_node(bih.nodes[node.getChildrenIndex()+0], child1_box, 1);
				draw_bih_node(bih.nodes[node.getChildrenIndex()+1], child2_box, 1);
			}
			else
			{
				draw_bih_node(bih.nodes[node.getChildrenIndex()+0], child1_box, depth);
				draw_bih_node(bih.nodes[node.getChildrenIndex()+1], child2_box, depth);
			}
		}
		else if(debug_mode_parameter == "intersect")
		{
			if(bih_draw_intersected_nodes && std::find(bih_intersected_nodes.begin(), bih_intersected_nodes.end(), &node - &bih.nodes[0]) != bih_intersected_nodes.end())
			{
				glColor3f(1.f, 0.f, 0.f);
				draw_box(deduced_box);

				draw_bih_node(bih.nodes[node.getChildrenIndex()+0], child1_box, depth+1);
				draw_bih_node(bih.nodes[node.getChildrenIndex()+1], child2_box, depth+2);
			}
		}
	}
}

static Vector3 sampleHemisphere(const Vector3 &X, const Vector3 &Y, const Vector3 &Z, float u1, float u2)
{
#if 1
	float r = std::sqrt(1.f - u1);
	float phi = 2 * std::acos(-1.f) * u2;
	float x = std::cos(phi) * r;
	float y = std::sin(phi) * r;
	float z = std::sqrt(u1);
#else
	float theta = std::acos(1.f - u1);
	float phi = 2 * std::acos(-1.f) * u2;
	float x = std::cos(phi) * std::sin(theta);
	float y = std::sin(phi) * std::sin(theta);
	float z = std::cos(theta);
#endif
	return X * x + Y * y + Z * z;
}

static void display()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glLoadIdentity();

	glTranslatef(0.f, 0.f, -camera_distance);

	glRotatef(camera_tilt, 1.f, 0.f, 0.f);
	glRotatef(camera_pan, 0.f, 1.f, 0.f);

	glScalef(1.f, 1.f, -1.f);

	if(debug_mode != "sampling")
	{
		glEnable(GL_DEPTH_TEST);
		for(const auto &triangle : scene.triangles)
		{
			glColor3fv(&scene.materials[triangle.material_index].color.r);
			draw_triangle(triangle);
		}
		glDisable(GL_DEPTH_TEST);
	}

	glColor3f(1.f, 1.f, 1.f);

	if(debug_mode == "bih")
	{
		glEnable(GL_BLEND);
		glBlendFunc(GL_ONE, GL_ONE);

		glPointSize(5.f);

		glEnable(GL_DEPTH_TEST);
		glBegin(GL_LINES);
		glColor3f(0.f, 0.f, 1.f);
		glVertex3fv(&bih_ray.origin.x);
		Vector3 e = bih_ray.origin + bih_ray.direction * 1000.f;
		glVertex3fv(&e.x);
		glEnd();
		glDisable(GL_DEPTH_TEST);

		if(debug_mode_parameter == "intersect")
		{
			float distance;
			bih.intersect(scene, bih_ray, &distance);
		}

		draw_bih_node(bih.nodes[0], bih.scene_aabb, 0);

		glDisable(GL_BLEND);
	}

	if(debug_mode == "sampling")
	{
		glPointSize(1.f);

		auto rnd = std::bind(std::uniform_real_distribution<float>(0, 1), std::default_random_engine());

		glBegin(GL_POINTS);
		for(size_t i=0u; i<1000u; ++i)
		{
			const auto p = sampleHemisphere(Vector3(1, 0, 0), Vector3(0, 0, -1), Vector3(0, 1, 0), rnd(), rnd());
			glVertex3fv(&p.x);
		}
		glEnd();
	}

	glutSwapBuffers();
}

int main(int argc, char *argv[])
{
	if(argc != 3 && argc != 4)
	{
		std::cerr << "usage: " << argv[0] << " <input.json> <debug_mode> [<debug_mode_parameter>]\n";
		return 1;
	}

	RenderOptions opts;
	LoadJob(argv[1], &opts);
	LoadScene(opts, &scene);
	debug_mode = argv[2];
	if(argc == 4) debug_mode_parameter = argv[3];

	glutInit(&argc, argv);

	glutInitWindowSize(512, 512);
	glutInitDisplayMode(GLUT_DOUBLE);
	glutCreateWindow("pray debug tool");

	init();
	glutKeyboardFunc(keyboard);
	glutReshapeFunc(reshape);
	glutDisplayFunc(display);
	glutMainLoop();

	return 0;
}
