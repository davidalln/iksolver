#include <iostream>
#include <list>
#include <math.h>

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include "ikskel.h"

// starting position for the window
#define WINPOS_X 100
#define WINPOS_Y 100

// size of the displayed end effector
#define END_SZE_X 0.025
#define END_SZE_Y 0.025

// width of the bone
#define BONE_WIDTH 5

// radius of the joint markers
#define JOINT_RAD 0.03

// radius of the target marker
#define TARGET_RAD 0.025

struct Window {
    int id = -1;
    int w = 1024;
    int h = 768;
};

Window win;

// the skeleton holding joint pos/angle info
Skeleton skeleton;

// the target we are trying to reach
EndTarget target;

__inline__ GLfloat toWorldSpace(int x, int w) {
    int wx = 2 * x - w;
    return (GLfloat)wx / (GLfloat) w;
}

void display() {
    // display params
    glLineWidth(BONE_WIDTH);

    // white background
    glClearColor(1.f, 1.f, 1.f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT);

    // draw the joints and bones
    std::list<Joint>::const_iterator joints;
    for (joints = skeleton.joints.begin(); joints != skeleton.joints.end(); ++joints) {
        Joint joint_start = *joints;

        // peek at the next joint then reset the pointer
        std::list<Joint>::const_iterator nextJoint = ++joints;
        --joints;

        EndTarget js_coords, je_coords;
        js_coords.x = joint_start.x;
        js_coords.y = joint_start.y;

        if (nextJoint == skeleton.joints.end()) {
            je_coords.x = skeleton.end.x;
            je_coords.y = skeleton.end.y;
        } else {
            je_coords.x = (*nextJoint).x;
            je_coords.y = (*nextJoint).y;
        }

        GLfloat aspect = (GLfloat)win.w / (GLfloat)win.h;
        if (win.w > win.h) {
            js_coords.x *= aspect;
            je_coords.x *= aspect;
        } else {
            js_coords.y *= aspect;
            je_coords.y *= aspect;
        }

        // bone
        // TODO: different color if bone is inactive (i.e. dragged out)
        glColor3f(1.0, 0.0, 0.0);
        glBegin(GL_LINES);
            glVertex2f(js_coords.x, js_coords.y);
            glVertex2f(je_coords.x, je_coords.y);
        glEnd();

        // joint
        // TODO: don't draw if inactive
        glColor3f(1.0, 1.0, 0.0);
        glBegin(GL_TRIANGLE_FAN);
        for (int i = 0; i <= 20; i++) {
            glVertex2f(js_coords.x + (JOINT_RAD * cos(i * (2 * M_PI) / 20)),
                    js_coords.y + (JOINT_RAD * sin(i * (2 * M_PI) / 20)));
        }
        glEnd();
    }

    // end effector
    if (skeleton.end.active) {
        GLfloat endx = skeleton.end.x;
        GLfloat endy = skeleton.end.y;

        GLfloat aspect = (GLfloat)win.w / (GLfloat)win.h;
        if (win.w > win.h) {
            endx *= aspect;
        } else {
            endy *= aspect;
        }

        glBegin(GL_QUADS);
            glColor3f(0.f, 0.f, 1.f);
            glVertex2f(endx - END_SZE_X, endy - END_SZE_Y);
            glVertex2f(endx + END_SZE_X, endy - END_SZE_Y);
            glVertex2f(endx + END_SZE_X, endy + END_SZE_Y);
            glVertex2f(endx - END_SZE_X, endy + END_SZE_Y);
        glEnd();
    }

    // target
    if (target.active) {
        GLfloat tarx = target.x;
        GLfloat tary = target.y;

        GLfloat aspect = (GLfloat)win.w / (GLfloat)win.h;
        if (win.w > win.h) {
            tarx *= aspect;
        } else {
            tary *= aspect;
        }

        GLfloat ax, ay, bx, by, cx, cy;
        GLfloat pi23 = 2.f * M_PI / 3.f;
        GLfloat pi43 = 2.f * pi23;

        cx = 0;
        cy = TARGET_RAD;
        ax = (cx * cos(pi43)) - (cy * sin(pi43)) + tarx;
        ay = (cx * sin(pi43)) + (cy * cos(pi43)) + tary;
        bx = (cx * cos(pi23)) - (cy * sin(pi23)) + tarx;
        by = (cx * sin(pi23)) + (cy * cos(pi23)) + tary;
        cx += tarx;
        cy += tary;

        glColor3f(0.f, 1.f, 0.f);
        glBegin(GL_TRIANGLES);
            glVertex2f(ax, ay);
            glVertex2f(bx, by);
            glVertex2f(cx, cy);
        glEnd();
    }

    glutSwapBuffers();
}

void mouse(int button, int state, int x, int y) {
    switch (button) {
        // update our target
        case GLUT_LEFT_BUTTON:
            if (state == GLUT_UP) {
                GLfloat newTarX = toWorldSpace(x, win.w);
                GLfloat newTarY = toWorldSpace(win.h - y - 1, win.h);

                if (!skeleton.joints.empty()) {
                    target.active = true;
                    target.x = newTarX;
                    target.y = newTarY;
                }
            }
            break;

        // update the end position of the skeleton and add a joint
        case GLUT_RIGHT_BUTTON:
            if (state == GLUT_UP) {
                GLfloat newEndX = toWorldSpace(x, win.w);
                GLfloat newEndY = toWorldSpace(win.h - y - 1, win.h);

                if (!skeleton.end.active) {
                    skeleton.end.active = true;
                } else {
                    // create a new joint
                    GLfloat oldEndX = skeleton.end.x;
                    GLfloat oldEndY = skeleton.end.y;

                    if (newEndX == oldEndX && newEndY == oldEndY)
                        break;

                    GLfloat u1, u2;
                    if (skeleton.joints.empty()) {
                        u1 = 1;
                        u2 = 0;
                    } else {
                        Joint endJoint = skeleton.joints.back();
                        u1 = oldEndX - endJoint.x;
                        u2 = oldEndY - endJoint.y;
                    }

                    GLfloat v1 = newEndX - oldEndX;
                    GLfloat v2 = newEndY - oldEndY;

                    double num = u1 * v1 + u2 * v2;
                    double dem = sqrtf(u1 * u1 + u2 * u2) * sqrtf(v1 * v1 + v2 * v2);

                    GLfloat angle = acos(num/dem) * 180 / M_PI;
                    angle = (newEndY > oldEndY) ? angle : -angle;

                    skeleton.joints.push_back(Joint(oldEndX, oldEndY, angle));
                    std::cout << "joint placed at " << oldEndX << "," << oldEndY <<
                        " w/ angle " << angle << " (u1: " << u1 << ", v1: " << v1 << ", u2: " << u2 << ", v2: " << v2 << ", num: " << num << ", dem: " << dem << ")" << std::endl;
                }

                skeleton.end.x = newEndX;
                skeleton.end.y = newEndY;

                std::cout << "--" << std::endl;
            }
            break;
    }

    glutPostRedisplay();
}

void keyboard(unsigned char key, int x, int y) {
    switch (key) {
        case 27:        // ESC
            if (win.id) glutDestroyWindow(win.id);
            exit(0);
            break;
    }

    glutPostRedisplay();
}

// standard rescaling on window resize event
void reshape(GLsizei width, GLsizei height) {
    if (height == 0) height = 1;
    GLfloat aspect = (GLfloat)width / (GLfloat)height;

    glViewport(0, 0, width, height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    if (width >= height) {
        gluOrtho2D(-1.0 * aspect, 1.0 * aspect, -1.0, 1.0);
    } else {
        gluOrtho2D(-1.0, 1.0, -1.0 / aspect, 1.0 / aspect);
    }

    win.w = width;
    win.h = height;
}

int main(int argc, char **argv) {
    // initialize glut
    glutInit(&argc, argv);
    glutInitWindowPosition(WINPOS_X, WINPOS_Y);
    glutInitWindowSize(win.w, win.h);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);

    win.id = glutCreateWindow("15-664 P1: IK Solver (dallen1)");

    // glut callbacks
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouse);

    // start the application
    glutMainLoop();
    return 0;
}
