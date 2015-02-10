/*
 * ikskel.h
 * ========
 *
 * definitions for the skeleton to be acted on by the IK solver
 *
 */

struct Joint {
    bool active = false;
    GLfloat x, y;
    GLfloat r;

    Joint(GLfloat xp, GLfloat yp, GLfloat rp) :
        x(xp), y(yp), r(rp) { };
};

struct EndTarget {
    bool active = false;
    GLfloat x, y;
};

struct Skeleton {
    std::list<Joint> joints;
    EndTarget end;
};

