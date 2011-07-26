#ifndef COLLISION_CHECKING_DISTANCE_PRIMITIVE_H
#define COLLISION_CHECKING_DISTANCE_PRIMITIVE_H

#include "collision_checking/BVH_defs.h"
#include "collision_checking/BVH_model.h"


/** \brief Main namespace */
namespace collision_checking
{

struct BVH_DistanceResult
{
  /** \brief Number of BV collision test performed */
  int num_bv_tests;

  /** \brief Number of triangle collision test performed */
  int num_tri_tests;

  /** \brief Query time used */
  BVH_REAL query_time_seconds;

  /** \brief relative and absolute error */
  BVH_REAL rel_err;
  BVH_REAL abs_err;

  /** \brief distance and points establishing the minimum distance for the models, within the relative and absolute error bounds specified. */
  BVH_REAL distance;
  Vec3f p1, p2;

  int qsize;

  BVH_DistanceResult();

  ~BVH_DistanceResult();
};


/** \brief Recursive collision kernel between between two BV trees */
template<typename BV>
void distanceRecurse(BVNode<BV>* tree1, BVNode<BV>* tree2, int b1, int b2,
                    Point* vertices1, Point* vertices2,
                    Triangle* tri_indices1, Triangle* tri_indices2,
                    BVH_DistanceResult* res)
{
  std::cerr << "Bounding volume structure default can not support distance operation!" << std::endl;
}

void distanceRecurse(BVNode<RSS>* tree1, BVNode<RSS>* tree2,
                    const Vec3f R[3], const Vec3f& T,
                    int b1, int b2,
                    Point* vertices1, Point* vertices2,
                    Triangle* tri_indices1, Triangle* tri_indices2,
                    BVH_DistanceResult* res);

}


#endif
