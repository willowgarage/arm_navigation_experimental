#include "collision_checking/distance_primitive.h"

namespace collision_checking
{

BVH_DistanceResult::BVH_DistanceResult()
{
  num_bv_tests = 0;
  num_tri_tests = 0;

  query_time_seconds = 0;
}


BVH_DistanceResult::~BVH_DistanceResult()
{

}



void distanceQueueRecurse(BVNode<RSS>* tree1, BVNode<RSS>* tree2,
                          const Vec3f R[3], const Vec3f& T,
                          int b1, int b2,
                          Point* vertices1, Point* vertices2,
                          Triangle* tri_indices1, Triangle* tri_indices2,
                          BVH_DistanceResult* res)
{

}

void distanceRecurse(BVNode<RSS>* tree1, BVNode<RSS>* tree2,
                    const Vec3f R[3], const Vec3f& T,
                    int b1, int b2,
                    Point* vertices1, Point* vertices2,
                    Triangle* tri_indices1, Triangle* tri_indices2,
                    BVH_DistanceResult* res)
{

}


}
