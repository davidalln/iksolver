#include <iostream>

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include "ikskel.h"

void Skeleton::freezeSkeleton() {
    if (joints.empty())
        return;

    frozen = true;
}

void Skeleton::resetSkeleton() {
    frozen = false;
    end.active = false;
    joints = std::list<Joint>();
}

