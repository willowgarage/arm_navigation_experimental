#include "collision_checking/collision.h"

namespace collision_checking
{

int collide(const BVHModel<OBB>& model1, const Vec3f R1[3], const Vec3f& T1,
            const BVHModel<OBB>& model2, const Vec3f R2[3], const Vec3f& T2, BVH_CollideResult* res, BVHFrontList* front_list)
{
  if(model1.build_state != BVH_BUILD_STATE_PROCESSED && model1.build_state != BVH_BUILD_STATE_UPDATED)
  {
    std::cerr << "BVH Error: Must finish BVH model construction before call collide()!" << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  if(model2.build_state != BVH_BUILD_STATE_PROCESSED && model2.build_state != BVH_BUILD_STATE_UPDATED)
  {
    std::cerr << "BVH Error: Must finish BVH model construction before call collide()!" << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  // currently only support the mesh-mesh collision
  if(model1.getModelType() != BVH_MODEL_TRIANGLES || model2.getModelType()!= BVH_MODEL_TRIANGLES)
  {
    std::cerr << "BVH Error: Collision only supported between two triangle models." << std::endl;
    return BVH_ERR_UNSUPPORTED_FUNCTION;
  }


  res->resetRecord();

  // compute the transform from 1 to 2
  Vec3f R1_col[3];
  R1_col[0] = Vec3f(R1[0][0], R1[1][0], R1[2][0]);
  R1_col[1] = Vec3f(R1[0][1], R1[1][1], R1[2][1]);
  R1_col[2] = Vec3f(R1[0][2], R1[1][2], R1[2][2]);

  Vec3f R2_col[3];
  R2_col[0] = Vec3f(R2[0][0], R2[1][0], R2[2][0]);
  R2_col[1] = Vec3f(R2[0][1], R2[1][1], R2[2][1]);
  R2_col[2] = Vec3f(R2[0][2], R2[1][2], R2[2][2]);

  Vec3f R[3];
  R[0] = Vec3f(R1_col[0].dot(R2_col[0]), R1_col[0].dot(R2_col[1]), R1_col[0].dot(R2_col[2]));
  R[1] = Vec3f(R1_col[1].dot(R2_col[0]), R1_col[1].dot(R2_col[1]), R1_col[1].dot(R2_col[2]));
  R[2] = Vec3f(R1_col[2].dot(R2_col[0]), R1_col[2].dot(R2_col[1]), R1_col[2].dot(R2_col[2]));

  Vec3f Ttemp = T2 - T1;
  Vec3f T(R1_col[0].dot(Ttemp), R1_col[1].dot(Ttemp), R1_col[2].dot(Ttemp));

  if(front_list && front_list->size() > 0)
    propagateBVHFrontList(model1.bvs, model2.bvs, R, T, model1.vertices, model2.vertices, model1.tri_indices, model2.tri_indices, res, front_list);
  else
    collideRecurse(model1.bvs, model2.bvs, R, T, 0, 0, model1.vertices, model2.vertices, model1.tri_indices, model2.tri_indices, res, front_list);


  // update the contact position and normal
  // contact R1 * c + T1
  // normal R1^{-T} * n = R1 * n
  for(int i = 0; i < res->numPairs(); ++i)
  {
    BVHCollisionPair* pairs = res->collidePairs();
    Vec3f normal = pairs[i].normal;
    pairs[i].normal = Vec3f(R1[0].dot(normal), R1[1].dot(normal), R1[2].dot(normal));
    Vec3f pos = pairs[i].contact_point;
    pairs[i].contact_point = Vec3f(R1[0].dot(pos), R1[1].dot(pos), R1[2].dot(pos)) + T1;
  }

  return BVH_OK;
}


int collide(const BVHModel<RSS>& model1, const Vec3f R1[3], const Vec3f& T1,
            const BVHModel<RSS>& model2, const Vec3f R2[3], const Vec3f& T2, BVH_CollideResult* res, BVHFrontList* front_list)
{
  if(model1.build_state != BVH_BUILD_STATE_PROCESSED && model1.build_state != BVH_BUILD_STATE_UPDATED)
  {
    std::cerr << "BVH Error: Must finish BVH model construction before call collide()!" << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  if(model2.build_state != BVH_BUILD_STATE_PROCESSED && model2.build_state != BVH_BUILD_STATE_UPDATED)
  {
    std::cerr << "BVH Error: Must finish BVH model construction before call collide()!" << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  // currently only support the mesh-mesh collision
  if(model1.getModelType() != BVH_MODEL_TRIANGLES || model2.getModelType()!= BVH_MODEL_TRIANGLES)
  {
    std::cerr << "BVH Error: Collision only supported between two triangle models." << std::endl;
    return BVH_ERR_UNSUPPORTED_FUNCTION;
  }


  res->resetRecord();

  // compute the transform from 1 to 2
  Vec3f R1_col[3];
  R1_col[0] = Vec3f(R1[0][0], R1[1][0], R1[2][0]);
  R1_col[1] = Vec3f(R1[0][1], R1[1][1], R1[2][1]);
  R1_col[2] = Vec3f(R1[0][2], R1[1][2], R1[2][2]);

  Vec3f R2_col[3];
  R2_col[0] = Vec3f(R2[0][0], R2[1][0], R2[2][0]);
  R2_col[1] = Vec3f(R2[0][1], R2[1][1], R2[2][1]);
  R2_col[2] = Vec3f(R2[0][2], R2[1][2], R2[2][2]);

  Vec3f R[3];
  R[0] = Vec3f(R1_col[0].dot(R2_col[0]), R1_col[0].dot(R2_col[1]), R1_col[0].dot(R2_col[2]));
  R[1] = Vec3f(R1_col[1].dot(R2_col[0]), R1_col[1].dot(R2_col[1]), R1_col[1].dot(R2_col[2]));
  R[2] = Vec3f(R1_col[2].dot(R2_col[0]), R1_col[2].dot(R2_col[1]), R1_col[2].dot(R2_col[2]));

  Vec3f Ttemp = T2 - T1;
  Vec3f T(R1_col[0].dot(Ttemp), R1_col[1].dot(Ttemp), R1_col[2].dot(Ttemp));

  if(front_list && front_list->size() > 0)
    propagateBVHFrontList(model1.bvs, model2.bvs, R, T, model1.vertices, model2.vertices, model1.tri_indices, model2.tri_indices, res, front_list);
  else
    collideRecurse(model1.bvs, model2.bvs, R, T, 0, 0, model1.vertices, model2.vertices, model1.tri_indices, model2.tri_indices, res, front_list);


  // update the contact position and normal
  // contact R1 * c + T1
  // normal R1^{-T} * n = R1 * n
  for(int i = 0; i < res->numPairs(); ++i)
  {
    BVHCollisionPair* pairs = res->collidePairs();
    Vec3f normal = pairs[i].normal;
    pairs[i].normal = Vec3f(R1[0].dot(normal), R1[1].dot(normal), R1[2].dot(normal));
    Vec3f pos = pairs[i].contact_point;
    pairs[i].contact_point = Vec3f(R1[0].dot(pos), R1[1].dot(pos), R1[2].dot(pos)) + T1;
  }

  return BVH_OK;
}


}

