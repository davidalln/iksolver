#include <iostream>
#include <math.h>

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include "ikskel.h"
#include "asst2/matrix.hpp"

#define JT_ALPHA 0.05

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

void Skeleton::solveIKwithJacobian(EndTarget target, JacobianMethod method) {
    matrix v = matrix(2, 1);
    matrix jacobian = matrix(2, joints.size()); 

    v.setValue(target.x - end.x, 0, 0);
    v.setValue(target.y - end.y, 1, 0);

    int j = 0;
    std::list<Joint>::iterator joint;
    for (joint = joints.begin(); joint != joints.end(); ++joint) {
        double x = end.x - joint->x;
        double y = end.y - joint->y;

        jacobian.setValue(-y, 0, j);
        jacobian.setValue(x, 1, j);

        ++j;
    }

    matrix jtranspose = matrix(joints.size(), 2);
    matrix dtheta = matrix(joints.size(), 1);

    jacobian.computeTranspose(&jtranspose);

    switch (method) {
        case TRANSPOSE:
            jtranspose.computeMatrixMul(&v, &dtheta);
            break;

        case PSEUDOINVERSE:
            matrix jjtranspose = matrix(2, 2);
            jacobian.computeMatrixMul(&jtranspose, &jjtranspose);

            matrix jjinverse = matrix(2, 2);
            jjtranspose.invertMatrix(&jjinverse, SVD_TOL);

            matrix total = matrix(joints.size(), 2);
            jtranspose.computeMatrixMul(&jjinverse, &total);

            total.computeMatrixMul(&v, &dtheta);
            break;
    }


    j = joints.size() - 1;
    std::list<Joint>::reverse_iterator rjoint;
    for (rjoint = joints.rbegin(); rjoint != joints.rend(); ++rjoint) {
        GLfloat dangle = JT_ALPHA * dtheta.getValue(j, 0);
        rjoint->angle += dangle * 180 / M_PI;

        GLfloat lastJointX = rjoint->x;
        GLfloat lastJointY = rjoint->y;
        GLfloat rootX = 0.f;
        GLfloat rootY = 0.f;

        // calculate the new end effector position
        GLfloat rendx = end.x - rjoint->x;
        GLfloat rendy = end.y - rjoint->y;
        end.x = rendx * cos(dangle) - rendy * sin(dangle);
        end.y = rendy * cos(dangle) + rendx * sin(dangle);
        end.x += rjoint->x;
        end.y += rjoint->y;

        // rotate the rest of the joints
        std::list<Joint>::iterator ujoint;
        for (ujoint = rjoint.base(); ujoint != joints.end(); ++ujoint) {
            GLfloat ljx = lastJointX;
            GLfloat ljy = lastJointY;

            lastJointX = ujoint->x;
            lastJointY = ujoint->y;

            // place the joint centered at the origin translated
            // away from the rest of the joints
            rootX = ujoint->x - ljx + rootX;
            rootY = ujoint->y - ljy + rootY;

            // rotate the joint
            ujoint->x = rootX * cos(dangle) - rootY * sin(dangle);
            ujoint->y = rootY * cos(dangle) + rootX * sin(dangle);

            // put it back in the correct space
            ujoint->x += rjoint->x;
            ujoint->y += rjoint->y;
        }

        --j;
    }
}

