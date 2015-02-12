#include <ctime>
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

// number of times we run CCD before termination
#define NUM_CCD_ITERS 100
#define NUM_JAC_ITERS 100

// how close we need to get to terminate CCD
#define CCD_EPSILON 0.01
#define JAC_EPSILON 0.0001

struct Window {
    int id = -1;
    int w = 1024;
    int h = 768;
};

Window win;

// the skeleton holding joint pos/angle info
Skeleton skeletonCCD;
Skeleton skeletonIKT;
Skeleton skeletonIKP;

// the target we are trying to reach
EndTarget target;

bool updateIK = false;

__inline__ std::pair<GLfloat, GLfloat> toWorldSpace(int x, int y) {
    int wx = 2 * x - win.w;
    int wy = 2 * y - win.h;

    GLfloat wr = (GLfloat)wx / (GLfloat)win.w;
    GLfloat hr = (GLfloat)wy / (GLfloat)win.h;

    GLfloat aspect = (GLfloat)win.w / (GLfloat)win.h;
    if (win.w > win.h)
        wr *= aspect;
    else
        hr *= aspect;

    return std::make_pair(wr, hr);
}

void drawKinematicChain(Skeleton skel, GLfloat r, GLfloat g, GLfloat b) {
    // begin drawing the kinematic chain
    glPushMatrix();
    glLoadIdentity();

    GLfloat skelroot_x = skel.root_x;
    GLfloat skelroot_y = skel.root_y;

    glTranslatef(skelroot_x, skelroot_y, 0.f);

    GLfloat lastJointLen = 0.f;

    std::list<Joint>::const_iterator joints;
    for (joints = skel.joints.begin(); joints != skel.joints.end(); ++joints) {
        Joint joint = *joints;

        // translate this and all further joints by its angle
        GLfloat jlength = joint.length;
        glTranslatef(lastJointLen, 0.f, 0.f);
        glRotatef(joint.angle, 0.f, 0.f, 1.f);

        // draw the bone
        glColor3f(r, g, b);
        glBegin(GL_LINES);
            glVertex2f(0.f, 0.f);
            glVertex2f(jlength, 0.f);
        glEnd();

        // draw the joint
        glColor3f(1.0, 1.0, 0.0);
        glBegin(GL_TRIANGLE_FAN);
        for (int i = 0; i <= 20; i++) {
            glVertex2f((JOINT_RAD * cos(i * (2 * M_PI) / 20)),
                    (JOINT_RAD * sin(i * (2 * M_PI) / 20)));
        }
        glEnd();

        lastJointLen = jlength;
    }

    // draw the end effector
    if (skel.end.active) {
        glColor3f(0.f, 0.f, 1.f);
        glTranslatef(lastJointLen, 0.f, 0.f);

        // square if in drawing mode, circle otherwise
        if (!skel.frozen) {
            glBegin(GL_QUADS);
                glVertex2f(-END_SZE_X, -END_SZE_Y);
                glVertex2f(END_SZE_X, -END_SZE_Y);
                glVertex2f(END_SZE_X, END_SZE_Y);
                glVertex2f(-END_SZE_X, END_SZE_Y);
            glEnd();
        } else {
            glBegin(GL_TRIANGLE_FAN);
            for (int i = 0; i <= 20; i++) {
                glVertex2f((JOINT_RAD * cos(i * (2 * M_PI) / 20)),
                    (JOINT_RAD * sin(i * (2 * M_PI) / 20)));
            }
            glEnd();
        }
    }

    glPopMatrix();
    // end drawing the kinematic chain
}

void display() {
    // display params
    glLineWidth(BONE_WIDTH);

    // white background
    glClearColor(1.f, 1.f, 1.f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);

    if (skeletonCCD.active) drawKinematicChain(skeletonCCD, 1.f, 0.f, 0.f);
    if (skeletonIKT.active) drawKinematicChain(skeletonIKT, 0.f, 1.f, 1.f);
    if (skeletonIKP.active) drawKinematicChain(skeletonIKP, 1.f, 0.f, 1.f);

    // draw the target if one exists
    if (target.active) {
        GLfloat tarx = target.x;
        GLfloat tary = target.y;

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

void motion(int x, int y) {
    if (updateIK) {
        std::pair<GLfloat, GLfloat> tars = toWorldSpace(x, win.h - y - 1);

        if (!skeletonCCD.joints.empty()) {
            target.active = true;
            target.x = tars.first;
            target.y = tars.second;

            if (skeletonIKP.active) {
                using namespace std;
                clock_t begin = clock();
                for (int i = 0; i < NUM_JAC_ITERS; i++) {
                    GLfloat ox = skeletonIKP.end.x;
                    GLfloat oy = skeletonIKP.end.y;

                    JacobianMethod method = PSEUDOINVERSE;
                    skeletonIKP.solveIKwithJacobian(target, method);

                    GLfloat dx = skeletonIKP.end.x - ox;
                    GLfloat dy = skeletonIKP.end.y - oy;
                    if (sqrtf(dx * dx + dy * dy) < JAC_EPSILON)
                        break;
                }
                clock_t end = clock();
                double elapsed = double(end - begin) / CLOCKS_PER_SEC;

                std::cout << "ik jacobian pseudoinverse took " << elapsed << " seconds" << std::endl;
            }

            if (skeletonIKT.active) {
                using namespace std;
                clock_t begin = clock();
                for (int i = 0; i < NUM_JAC_ITERS; i++) {
                    GLfloat ox = skeletonIKT.end.x;
                    GLfloat oy = skeletonIKT.end.y;

                    JacobianMethod method = TRANSPOSE;
                    skeletonIKT.solveIKwithJacobian(target, method);

                    GLfloat dx = skeletonIKT.end.x - ox;
                    GLfloat dy = skeletonIKT.end.y - oy;
                    if (sqrtf(dx * dx + dy * dy) < JAC_EPSILON)
                        break;
                }
                clock_t end = clock();
                double elapsed = double(end - begin) / CLOCKS_PER_SEC;

                std::cout << "ik jacobian transpose took " << elapsed << " seconds" << std::endl;
            }

            if (skeletonCCD.active) {
                using namespace std;
                clock_t begin = clock();
                for (int i = 0; i < NUM_CCD_ITERS; i++) {
                    skeletonCCD.solveIKwithCCD(target);
                    GLfloat dx = target.x - skeletonCCD.end.x;
                    GLfloat dy = target.y - skeletonCCD.end.y;
                    if (sqrtf(dx * dx + dy * dy) < CCD_EPSILON)
                        break;
                }
                clock_t end = clock();
                double elapsed = double(end - begin) / CLOCKS_PER_SEC;

                std::cout << "ik ccd took " << elapsed << " seconds" << std::endl;
            }
        }
    }

    glutPostRedisplay();
}

void mouse(int button, int state, int x, int y) {
    switch (button) {
        case GLUT_LEFT_BUTTON:
            // update the end position of the skeleton and add a joint
            if (state == GLUT_UP && !skeletonCCD.frozen) {
                std::pair<GLfloat, GLfloat> nars = toWorldSpace(x, win.h - y - 1);
                GLfloat newEndX = nars.first;
                GLfloat newEndY = nars.second;

                if (!skeletonCCD.end.active) {
                    skeletonCCD.end.active = true;
                    skeletonCCD.root_x = newEndX;
                    skeletonCCD.root_y = newEndY;

                    skeletonIKT.end.active = true;
                    skeletonIKT.root_x = newEndX;
                    skeletonIKT.root_y = newEndY;

                    skeletonIKP.end.active = true;
                    skeletonIKP.root_x = newEndX;
                    skeletonIKP.root_y = newEndY;
                } else {
                    // create a new joint
                    GLfloat oldEndX = skeletonCCD.end.x;
                    GLfloat oldEndY = skeletonCCD.end.y;

                    if (newEndX == oldEndX && newEndY == oldEndY)
                        break;

                    GLfloat u1, u2;
                    if (skeletonCCD.joints.empty()) {
                        u1 = 1;
                        u2 = 0;
                    } else {
                        Joint endJoint = skeletonCCD.joints.back();
                        u1 = oldEndX - endJoint.x;
                        u2 = oldEndY - endJoint.y;
                    }

                    GLfloat v1 = newEndX - oldEndX;
                    GLfloat v2 = newEndY - oldEndY;

                    GLfloat atanA = atan2(u2, u1);
                    GLfloat atanB = atan2(v2, v1);

                    GLfloat angle = (atanB - atanA) * 180.f / M_PI;

                    GLfloat length = sqrtf(v1 * v1 + v2 * v2);

                    skeletonCCD.joints.push_back(Joint(oldEndX, oldEndY, angle, length));
                    skeletonIKT.joints.push_back(Joint(oldEndX, oldEndY, angle, length));
                    skeletonIKP.joints.push_back(Joint(oldEndX, oldEndY, angle, length));
                    std::cout << "joint placed at " << oldEndX << "," << oldEndY <<
                        " w/ angle " << angle << " and length " << length << std::endl;
                }

                skeletonCCD.end.x = newEndX;
                skeletonCCD.end.y = newEndY;

                skeletonIKT.end.x = newEndX;
                skeletonIKT.end.y = newEndY;

                skeletonIKP.end.x = newEndX;
                skeletonIKP.end.y = newEndY;

                std::cout << "--" << std::endl;

            // update our target
            } else if (state == GLUT_DOWN && skeletonCCD.frozen) {
                updateIK = true;
                motion(x, y);
            } else if (state == GLUT_UP && skeletonCCD.frozen) {
                updateIK = false;
                motion(x, y);
            }
            break;
    }

    glutPostRedisplay();
}

void keyboard(unsigned char key, int x, int y) {
    switch (key) {
        case 49:
            if (skeletonCCD.frozen) {
                skeletonCCD.active = !skeletonCCD.active;
            }
            break;

        case 50:
            if (skeletonIKT.frozen) {
                skeletonIKT.active = !skeletonIKT.active;
            }
            break;

        case 51:
            if (skeletonIKP.frozen) {
                skeletonIKP.active = !skeletonIKP.active;
            }
            break;

        case 27:        // ESC
            if (win.id) glutDestroyWindow(win.id);
            exit(0);
            break;

        case 32:        // SPACE
            if (skeletonCCD.frozen) {
                target.active = false;
                skeletonCCD.resetSkeleton();
                skeletonIKT.resetSkeleton();
                skeletonIKP.resetSkeleton();

                skeletonCCD.active = true;
                skeletonIKT.active = false;
                skeletonIKP.active = false;
            } else {
                skeletonCCD.freezeSkeleton();
                skeletonIKT.freezeSkeleton();
                skeletonIKP.freezeSkeleton();
            }
            break;
    }

    glutPostRedisplay();
}

// standard rescaling on window resize event
void reshape(GLsizei width, GLsizei height) {
    // TODO: resizing window during drawing mode breaks it

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
    skeletonCCD.active = true;

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
    glutMotionFunc(motion);

    // start the application
    glutMainLoop();
    return 0;
}

