/*
 * ikskel.h
 * ========
 *
 * definitions for the skeleton to be acted on by the IK solver
 *
 */

#ifndef _IKSKEL_H_
#define _IKSKEL_H_

#include <list>

__inline__ GLfloat clamp(GLfloat x) {
    if (x < -180.f) x += 360.f;
    if (x > 180.f) x -= 360.f;
    return x;
}

struct Joint {
    bool active = false;
    GLfloat x, y;
    GLfloat angle, length;

    Joint (GLfloat xp, GLfloat yp, GLfloat a, GLfloat l) :
        x(xp), y(yp), angle(a), length(l) { };
};

struct EndTarget {
    bool active = false;
    GLfloat x, y;
};

enum JacobianMethod { TRANSPOSE, PSEUDOINVERSE };

struct Skeleton {
    bool active = false;
    bool frozen = false;
    std::list<Joint> joints;
    EndTarget end;

    GLfloat root_x, root_y;

    void freezeSkeleton();
    void resetSkeleton();

    void solveIKwithCCD(EndTarget target);
    void solveIKwithJacobian(EndTarget target, JacobianMethod method);
};

#endif
