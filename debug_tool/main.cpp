#define FREEGLUT_LIB_PRAGMAS 0
#include <GL/freeglut.h>

#include "scene.hpp"
#include "bih.hpp"

using namespace std;

static Scene scene;
static float camera_pan = 0.f, camera_tilt = 0.f;

static string debug_mode;
static string debug_mode_parameter;

// "bih"
static Bih bih;
static int bih_depth = 0; // "all"
static Bih::Node *bih_current_node = nullptr;

static void bih_rebuild()
{
	auto current_node_index = bih.nodes.empty() ? 0 : bih_current_node - &bih.nodes[0];
	bih = Bih();
	bih.build(scene);
	bih_current_node = &bih.nodes[current_node_index];
}

static void init()
{
	glEnable(GL_CULL_FACE);
	glFrontFace(GL_CW);

	glPointSize(5.f);

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
	if(debug_mode == "bih")
	{
		if(debug_mode_parameter == "all")
		{
			if(key == 'i') ++bih_depth;
			if(key == 'k') --bih_depth;
		}
		else if(debug_mode_parameter == "single")
		{
			auto old_current_node = bih_current_node;
			if(key == 'p') bih_current_node = bih_current_node->parent;
			if(key == 'l') bih_current_node = bih_current_node->child1;
			if(key == 'r') bih_current_node = bih_current_node->child2;

			if(old_current_node != bih_current_node)
			{
				std::cout << "current: " << bih_current_node->index << " p: " << bih_current_node->parent->index << " l: " << bih_current_node->child1->index << " r: " << bih_current_node->child2->index << "\n";
			}
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

static void draw_bih_node(const Bih::Node &node, const AABox3 &deduced_box, int depth)
{
	if(node.type == Bih::Node::Leaf)
	{
		if(debug_mode_parameter == "all")
		{
			if(bih_depth == depth)
			{
				glColor3f(0.f, 1.f, 0.f);
				draw_triangle(scene.triangles[bih.triangles[node.data.leaf.children_index]]);
			}
		}
		else if(debug_mode_parameter == "single")
		{
			if(depth == 1)
			{
				glColor3f(0.f, 1.f, 0.f);
				draw_triangle(scene.triangles[bih.triangles[node.data.leaf.children_index]]);
			}
		}
	}
	else
	{
		auto child1_box = deduced_box;
		child1_box.max[node.type] = node.data.split.left_plane;
		auto child2_box = deduced_box;
		child2_box.min[node.type] = node.data.split.right_plane;

		if(debug_mode_parameter == "all")
		{
			if(bih_depth == depth)
			{
				glColor3f(1.f, 0.f, 0.f);
				draw_box(child1_box);
				glColor3f(0.f, 0.f, 1.f);
				draw_box(child2_box);

				draw_bih_node(bih.nodes[node.data.split.children_index+0], child1_box, depth+1);
				draw_bih_node(bih.nodes[node.data.split.children_index+1], child2_box, depth+2);
			}
		}
		else if(debug_mode_parameter == "single")
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
				v[node.type] = node.split_point;
				glVertex3fv(&v.x);
				glEnd();

				draw_bih_node(bih.nodes[node.data.split.children_index+0], child1_box, 1);
				draw_bih_node(bih.nodes[node.data.split.children_index+1], child2_box, 1);
			}
			else
			{
				draw_bih_node(bih.nodes[node.data.split.children_index+0], child1_box, depth);
				draw_bih_node(bih.nodes[node.data.split.children_index+1], child2_box, depth);
			}
		}
	}
}

static void display()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glLoadIdentity();

	glTranslatef(0.f, 0.f, -5.f);

	glRotatef(camera_tilt, 1.f, 0.f, 0.f);
	glRotatef(camera_pan, 0.f, 1.f, 0.f);

	glScalef(1.f, 1.f, -1.f);

	glEnable(GL_DEPTH_TEST);
	for(const auto &triangle : scene.triangles)
	{
		glColor3fv(&scene.materials[triangle.material_index].color.r);
		draw_triangle(triangle);
	}
	glDisable(GL_DEPTH_TEST);

	glColor3f(1.f, 1.f, 1.f);

	if(debug_mode == "bih")
	{
		glEnable(GL_BLEND);
		glBlendFunc(GL_ONE, GL_ONE);

		draw_bih_node(bih.nodes[0], bih.scene_aabb, 0);

		glDisable(GL_BLEND);
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

	scene.load(argv[1], nullptr);
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
