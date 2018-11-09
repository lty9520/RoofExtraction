//ʹ��CGAL���ʵ�ֵ㼯��Delaunay�����ʷ֣�voronoiͼ
//�����Delaunay�㷨�����ע����ο�CGAL��Delaunay�����ʷֵ�ʵ�֣�CGAL��Delaunay�����ʷֵ�ʵ���������㷨��Incremental��
//��������ص����ڶ�CGAL��Delaunay���ݽṹ�ķ��ʣ���ο�����points_triangulation()

//SudoLeo 2010/7/20
//CGAL required, GLUT required

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
//#include <CGAL/Triangulation_euclidean_traits_xy_3.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include "GL/glut.h"
#include <iostream>
#include <cmath>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_2<K> Delaunay;
typedef Delaunay::Vertex_handle Vertex_handle;

typedef K::Point_2 Point;

std::vector<Point> vertices;

int global_w, global_h;
int tri_state = 0;

void points_draw()
{
	glClear(GL_COLOR_BUFFER_BIT);
	glPushMatrix();

	std::vector <Point>::iterator iter;
	glColor3f(1.0, 1.0, 1.0);
	glPointSize(5);
	glBegin(GL_POINTS);
	for (iter = vertices.begin(); iter != vertices.end(); iter++)
		glVertex2i(iter->hx(), iter->hy());
	glEnd();

	glPopMatrix();
	glutSwapBuffers();
}

void points_add_point(int x, int y)
{
	vertices.push_back(Point(x, global_h - y));
}


void points_clear()
{
	glClear(GL_COLOR_BUFFER_BIT);
	glPushMatrix();
	glPopMatrix();
	glutSwapBuffers();

	vertices.clear();
	tri_state = 0;
}

void read_file()//���ļ��ж���㼯���ݣ�����ʱ��ʹ��
{
	FILE* f;
	f = freopen("data.txt", "r", stdin);

	int a, b;
	while (std::cin >> a >> b)
	{
		vertices.push_back(Point(a, b));
	}

	fclose(f);
}

void points_triangulation()
{
	Delaunay dt;//Delaunay���ݽṹ������ǰ���ݵ�һ���ҽ���һ���������ʷ֣�������ο�CGAL_manual

	dt.insert(vertices.begin(), vertices.end());//��������

	points_draw();//points_draw()�������Ѿ�����һ��glutSwapBuffers()����������һ�ε���glutSwapBuffers()
	//��һ֡�Ļ��������ε���glutSwapBuffers()����Ա�����Ӱ�죬������һЩ���⣬�ⲻ�Ǳ��ĵ��ص㣬�����Һ���֮

	glPushMatrix();

	Delaunay::Finite_faces_iterator fit;//����Delaunay�������棨�����棩����ÿ����ı߻�����
	glColor3f(0.0, 0.0, 1.0);
	for (fit = dt.finite_faces_begin(); fit != dt.finite_faces_end(); fit++)
	{
		glBegin(GL_LINE_LOOP);
		glVertex2i(fit->vertex(0)->point().hx(), fit->vertex(0)->point().hy());
		glVertex2i(fit->vertex(1)->point().hx(), fit->vertex(1)->point().hy());
		glVertex2i(fit->vertex(2)->point().hx(), fit->vertex(2)->point().hy());
		glEnd();
	}//���Delaunay�����ʷֵĻ��ƣ�Delaunayͼ

	Delaunay::Edge_iterator eit;//����Delaunay�����бߣ�����Delaunayͼ�Ķ�żͼ����Voronoiͼ

	glEnable(GL_LINE_STIPPLE);//ʹ�õ㻭ģʽ����ʹ������������Voronoiͼ
	glLineStipple(1, 0x3333);
	glColor3f(0.0, 1.0, 0.0);

	for (eit = dt.edges_begin(); eit != dt.edges_end(); eit++)
	{
		CGAL::Object o = dt.dual(eit);//��eit�����żͼ������Ӧ�ı�

		if (CGAL::object_cast<K::Segment_2>(&o)) //������������߶Σ�������߶�
		{
			glBegin(GL_LINES);
			glVertex2i(CGAL::object_cast<K::Segment_2>(&o)->source().hx(), CGAL::object_cast<K::Segment_2>(&o)->source().hy());
			glVertex2i(CGAL::object_cast<K::Segment_2>(&o)->target().hx(), CGAL::object_cast<K::Segment_2>(&o)->target().hy());
			glEnd();
		}
		else if (CGAL::object_cast<K::Ray_2>(&o))//��������������ߣ����������
		{
			glBegin(GL_LINES);
			glVertex2i(CGAL::object_cast<K::Ray_2>(&o)->source().hx(), CGAL::object_cast<K::Ray_2>(&o)->source().hy());
			glVertex2i(CGAL::object_cast<K::Ray_2>(&o)->point(1).hx(), CGAL::object_cast<K::Ray_2>(&o)->point(1).hy());
			glEnd();
		}
	}
	glDisable(GL_LINE_STIPPLE);//�رյ㻭ģʽ

	glPopMatrix();
	glutSwapBuffers();

	tri_state = 1;//��������ʷ֣���״̬Ϊ1
}

void display(void)
{
}

void init(void)
{
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glShadeModel(GL_FLAT);

}

void reshape(int w, int h)
{
	global_w = w;
	global_h = h;
	points_clear();

	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	glOrtho(0, w, 0, h, -1.0, 1.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void mouse(int button, int state, int x, int y)
{
	if (button == GLUT_LEFT_BUTTON && state == GLUT_UP)
	{
		if (tri_state == 1) points_clear();
		else
		{
			points_add_point(x, y);
			//read_file();
			points_draw();
		}
	}
	if (button == GLUT_RIGHT_BUTTON && state == GLUT_UP)
	{
		if (tri_state == 1) points_clear();
		else points_triangulation();
	}
}

void keyboard(unsigned char key, int x, int y)
{
	switch (key) {
	case 27:
		exit(0);
		break;
	}
}

int main(int argc, char** argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(800, 600);
	glutInitWindowPosition(100, 100);
	glutCreateWindow(argv[0]);
	init();
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutMouseFunc(mouse);
	glutKeyboardFunc(keyboard);
	glutMainLoop();
	return 0;
}
