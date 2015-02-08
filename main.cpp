#include <stdlib.h>

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#define WINPOS_X 100
#define WINPOS_Y 100

struct Window {
    int id = -1;
    int w = 1024;
    int h = 768;
};

struct Window win;

void display() {
    // white background
    glClearColor(1.f, 1.f, 1.f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT);

    glutSwapBuffers();
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

    // start the application
    glutMainLoop();
    return 0;
}
