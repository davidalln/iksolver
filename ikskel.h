/*
 * ikskel.h
 * ========
 *
 * definitions for the skeleton to be acted on by the IK solver
 *
 */

#include <list>

struct Joint {
    bool active = false;
    GLfloat start_x, start_y;
    GLfloat angle, length;

    Joint (GLfloat x, GLfloat y, GLfloat a, GLfloat l) :
        start_x(x), start_y(y), angle(a), length(l) { };
};

struct EndTarget {
    bool active = false;
    GLfloat x, y;
};

struct Skeleton {
    bool frozen = false;
    std::list<Joint> joints;
    EndTarget end;

    GLfloat root_x, root_y;

    void freezeSkeleton();
    void resetSkeleton();
};

