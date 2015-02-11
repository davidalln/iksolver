#include <iostream>
#include <math.h>

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

void Skeleton::solveIKwithCCD(EndTarget target) {
    std::list<Joint>::reverse_iterator joint;
    for (joint = joints.rbegin(); joint != joints.rend(); ++joint) {
        // calculate vectors from the joint to the target and the
        // end effector to the target
        GLfloat rd1 = target.x - joint->x;
        GLfloat rd2 = target.y - joint->y;
        GLfloat re1 = end.x - joint->x;
        GLfloat re2 = end.y - joint->y;

        GLfloat atanA = atan2(re2, re1);
        GLfloat atanB = atan2(rd2, rd1);

        GLfloat ap = (atanB - atanA);

        // update our angle
        joint->angle = clamp((ap * 180.f / M_PI) + joint->angle);

        // calculate the new end effector position
        GLfloat rendx = re1;
        GLfloat rendy = re2;
        end.x = rendx * cos(ap) - rendy * sin(ap);
        end.y = rendy * cos(ap) + rendx * sin(ap);
        end.x += joint->x;
        end.y += joint->y;

        GLfloat lastJointX = joint->x;
        GLfloat lastJointY = joint->y;
        GLfloat rootX = 0.f;
        GLfloat rootY = 0.f;

        // rotate the rest of the joints
        std::list<Joint>::iterator ujoint;
        for (ujoint = joint.base(); ujoint != joints.end(); ++ujoint) {
            GLfloat ljx = lastJointX;
            GLfloat ljy = lastJointY;

            lastJointX = ujoint->x;
            lastJointY = ujoint->y;

            // place the joint centered at the origin translated
            // away from the rest of the joints
            rootX = ujoint->x - ljx + rootX;
            rootY = ujoint->y - ljy + rootY;

            // rotate the joint
            ujoint->x = rootX * cos(ap) - rootY * sin(ap);
            ujoint->y = rootY * cos(ap) + rootX * sin(ap);

            // put it back in the correct space
            ujoint->x += joint->x;
            ujoint->y += joint->y;
        }
    }
}

