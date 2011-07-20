#include "collision_checking/collision.h"
#include "collision_checking/continuous_collision.h"
#include <limits>

#include <cstdio>
#include <LinearMath/btVector3.h>
#include <LinearMath/btTransform.h>

#if USE_PQP
#include <PQP/PQP.h>
#endif

using namespace collision_checking;

void loadOBJFile(const char* filename, std::vector<Point>& points, std::vector<Triangle>& triangles);

void generateRandomTransform(BVH_REAL extents[6], std::vector<btTransform>& transforms, std::vector<btTransform>& transforms2, BVH_REAL delta_trans[3], BVH_REAL delta_rot, int n);

void generateRandomTransform_ccd(BVH_REAL extents[6], std::vector<btTransform>& transforms, std::vector<btTransform>& transforms2, BVH_REAL delta_trans[3], BVH_REAL delta_rot, int n,
                                 const std::vector<Point>& vertices1, const std::vector<Triangle>& triangles1,
                                 const std::vector<Point>& vertices2, const std::vector<Triangle>& triangles2);

BVH_REAL rand_interval(BVH_REAL rmin, BVH_REAL rmax);

void sortCollisionPair(BVHCollisionPair* pairs, int n);


#if USE_PQP
void sortCollisionPair_PQP(CollisionPair* pairs, int n);


bool collide_PQP(const btTransform& T,
                 const std::vector<Point>& vertices1, const std::vector<Triangle>& triangles1,
                 const std::vector<Point>& vertices2, const std::vector<Triangle>& triangles2, bool verbose = true);

bool collide_PQP2(const btTransform& T,
                 const std::vector<Point>& vertices1, const std::vector<Triangle>& triangles1,
                 const std::vector<Point>& vertices2, const std::vector<Triangle>& triangles2, bool verbose = true);
#endif

template<typename BV>
bool collide_Test(const btTransform& T,
                 const std::vector<Point>& vertices1, const std::vector<Triangle>& triangles1,
                 const std::vector<Point>& vertices2, const std::vector<Triangle>& triangles2,
                 SplitMethodType split_method,
                 bool verbose = true);

bool collide_Test2(const btTransform& T,
                 const std::vector<Point>& vertices1, const std::vector<Triangle>& triangles1,
                 const std::vector<Point>& vertices2, const std::vector<Triangle>& triangles2,
                 SplitMethodType split_method,
                 bool verbose = true);

template<typename BV>
bool collide_front_Test(const btTransform& T, const btTransform& T2,
                        const std::vector<Point>& vertices1, const std::vector<Triangle>& triangles1,
                        const std::vector<Point>& vertices2, const std::vector<Triangle>& triangles2,
                        SplitMethodType split_method,
                        bool refit_bottomup = true, bool verbose = true);

template<typename BV>
bool continuous_collide_Test(const btTransform& T, const btTransform& T2,
                        const std::vector<Point>& vertices1, const std::vector<Triangle>& triangles1,
                        const std::vector<Point>& vertices2, const std::vector<Triangle>& triangles2,
                        SplitMethodType split_method,
                        bool refit_bottomup = true, bool verbose = true);


static const int num_max_contacts = 10;

int main(int argc, char** argv)
{
  std::vector<Point> p1, p2;
  std::vector<Triangle> t1, t2;
  loadOBJFile("env.obj", p1, t1);
  loadOBJFile("rob.obj", p2, t2);

  std::cout << "model1 " << p1.size() << " " << t1.size() << std::endl;
  std::cout << "model2 " << p2.size() << " " << t2.size() << std::endl;

  std::vector<btTransform> transforms; // t0
  std::vector<btTransform> transforms2; // t1
  std::vector<btTransform> transforms_ccd; // t0;
  std::vector<btTransform> transforms_ccd2; // t1;
  BVH_REAL extents[] = {-3000, -3000, 0, 3000, 3000, 3000};
  BVH_REAL delta_trans[] = {1, 1, 1};
  int n = 1000;
  generateRandomTransform(extents, transforms, transforms2, delta_trans, 0.005 * 2 * 3.1415, n);
/*
  generateRandomTransform_ccd(extents, transforms_ccd, transforms_ccd2, delta_trans, 0.005 * 2 * 3.1415, n, p1, t1, p2, t2);

  for(unsigned int i = 0; i < transforms_ccd2.size(); ++i)
  {
    std::cout << "test id " << i << std::endl;

#if USE_PQP
    collide_PQP(transforms_ccd[i], p1, t1, p2, t2);

    collide_PQP(transforms_ccd2[i], p1, t1, p2, t2);
#endif

    continuous_collide_Test<OBB>(transforms_ccd[i], transforms_ccd2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN);

    continuous_collide_Test<OBB>(transforms_ccd[i], transforms_ccd2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER);

    continuous_collide_Test<OBB>(transforms_ccd[i], transforms_ccd2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN);

    continuous_collide_Test<AABB>(transforms_ccd[i], transforms_ccd2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN);

    continuous_collide_Test<AABB>(transforms_ccd[i], transforms_ccd2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER);

    continuous_collide_Test<AABB>(transforms_ccd[i], transforms_ccd2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN);

    continuous_collide_Test<KDOP<24> >(transforms_ccd[i], transforms_ccd2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN);

    continuous_collide_Test<KDOP<24> >(transforms_ccd[i], transforms_ccd2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER);

    continuous_collide_Test<KDOP<24> >(transforms_ccd[i], transforms_ccd2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN);

    continuous_collide_Test<KDOP<18> >(transforms_ccd[i], transforms_ccd2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN);

    continuous_collide_Test<KDOP<18> >(transforms_ccd[i], transforms_ccd2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER);

    continuous_collide_Test<KDOP<18> >(transforms_ccd[i], transforms_ccd2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN);

    continuous_collide_Test<KDOP<16> >(transforms_ccd[i], transforms_ccd2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN);

    continuous_collide_Test<KDOP<16> >(transforms_ccd[i], transforms_ccd2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER);

    continuous_collide_Test<KDOP<16> >(transforms_ccd[i], transforms_ccd2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN);

    std::cout << std::endl;
  }
*/

  for(unsigned int i = 0 ; i < transforms.size(); ++i)
  {
    std::cout << "test id " << i << std::endl;

#if USE_PQP
    collide_PQP(transforms[i], p1, t1, p2, t2);

    collide_PQP2(transforms[i], p1, t1, p2, t2);
#endif

    collide_Test<OBB>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN);

    collide_Test<OBB>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER);

    collide_Test<OBB>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN);

    collide_Test2(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN);

    collide_Test2(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER);

    collide_Test2(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN);

    collide_Test<AABB>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN);

    collide_Test<AABB>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER);

    collide_Test<AABB>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN);

    collide_Test<KDOP<24> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN);

    collide_Test<KDOP<24> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER);

    collide_Test<KDOP<24> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN);

    collide_Test<KDOP<18> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN);

    collide_Test<KDOP<18> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER);

    collide_Test<KDOP<18> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN);

    collide_Test<KDOP<16> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN);

    collide_Test<KDOP<16> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER);

    collide_Test<KDOP<16> >(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN);

    std::cout << std::endl;
  }

  return 1;

  for(unsigned int i = 0 ; i < transforms2.size(); ++i)
  {
    std::cout << "test id " << i << std::endl;

#if USE_PQP
    collide_PQP(transforms2[i], p1, t1, p2, t2);
#endif

    collide_front_Test<OBB>(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN);

    collide_front_Test<OBB>(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER);

    collide_front_Test<OBB>(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN);

    collide_front_Test<AABB>(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN);

    collide_front_Test<AABB>(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER);

    collide_front_Test<AABB>(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN);

    collide_front_Test<KDOP<24> >(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN);

    collide_front_Test<KDOP<24> >(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER);

    collide_front_Test<KDOP<24> >(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN);

    collide_front_Test<KDOP<18> >(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN);

    collide_front_Test<KDOP<18> >(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER);

    collide_front_Test<KDOP<18> >(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN);

    collide_front_Test<KDOP<16> >(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN);

    collide_front_Test<KDOP<16> >(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER);

    collide_front_Test<KDOP<16> >(transforms[i], transforms2[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN);

    std::cout << std::endl;
  }

  return 0;
}

void loadOBJFile(const char* filename, std::vector<Point>& points, std::vector<Triangle>& triangles)
{

  FILE* file = fopen(filename, "rb");
  if(!file)
  {
    std::cerr << "file not exist" << std::endl;
    return;
  }

  bool has_normal = false;
  bool has_texture = false;
  char line_buffer[2000];
  while(fgets(line_buffer, 2000, file))
  {
    char* first_token = strtok(line_buffer, "\r\n\t ");
    if(!first_token || first_token[0] == '#' || first_token[0] == 0)
      continue;

    switch(first_token[0])
    {
      case 'v':
      {
        if(first_token[1] == 'n')
        {
          strtok(NULL, "\t ");
          strtok(NULL, "\t ");
          strtok(NULL, "\t ");
          has_normal = true;
        }
        else if(first_token[1] == 't')
        {
          strtok(NULL, "\t ");
          strtok(NULL, "\t ");
          has_texture = true;
        }
        else
        {
          BVH_REAL x = (BVH_REAL)atof(strtok(NULL, "\t "));
          BVH_REAL y = (BVH_REAL)atof(strtok(NULL, "\t "));
          BVH_REAL z = (BVH_REAL)atof(strtok(NULL, "\t "));
          Point p(x, y, z);
          points.push_back(p);
        }
      }
      break;
      case 'f':
      {
        Triangle tri;
        char* data[30];
        int n = 0;
        while((data[n] = strtok(NULL, "\t \r\n")) != NULL)
        {
          if(strlen(data[n]))
            n++;
        }

        for(int t = 0; t < (n - 2); ++t)
        {
          if((!has_texture) && (!has_normal))
          {
            tri[0] = atoi(data[0]) - 1;
            tri[1] = atoi(data[1]) - 1;
            tri[2] = atoi(data[2]) - 1;
          }
          else
          {
            const char *v1;
            for(int i = 0; i < 3; i++)
            {
               // vertex ID
               if(i == 0)
                 v1 = data[0];
               else
                 v1 = data[t + i];

               tri[i] = atoi(v1) - 1;

               // texture coordinate
               const char *v2 = strchr(v1 + 1, '/');

               if(v2)
               {
                 strchr(v2 + 1, '/');
               }
            }
          }
          triangles.push_back(tri);
        }
      }
    }
  }
}

BVH_REAL rand_interval(BVH_REAL rmin, BVH_REAL rmax)
{
  BVH_REAL t = rand() / ((BVH_REAL)RAND_MAX + 1);
  return (t * (rmax - rmin) + rmin);
}

void generateRandomTransform(BVH_REAL extents[6], std::vector<btTransform>& transforms, std::vector<btTransform>& transforms2, BVH_REAL delta_trans[3], BVH_REAL delta_rot, int n)
{
  transforms.resize(n);
  transforms2.resize(n);
  for(int i = 0; i < n; ++i)
  {
    BVH_REAL x = rand_interval(extents[0], extents[3]);
    BVH_REAL y = rand_interval(extents[1], extents[4]);
    BVH_REAL z = rand_interval(extents[2], extents[5]);

    const BVH_REAL pi = 3.1415926;
    BVH_REAL a = rand_interval(0, 2 * pi);
    BVH_REAL b = rand_interval(0, 2 * pi);
    BVH_REAL c = rand_interval(0, 2 * pi);

    btMatrix3x3 M;
    M.setEulerZYX(a, b, c);
    btVector3 v(x, y, z);
    btTransform T(M, v);
    transforms[i] = T;

    BVH_REAL deltax = rand_interval(-delta_trans[0], delta_trans[0]);
    BVH_REAL deltay = rand_interval(-delta_trans[1], delta_trans[1]);
    BVH_REAL deltaz = rand_interval(-delta_trans[2], delta_trans[2]);

    BVH_REAL deltaa = rand_interval(-delta_rot, delta_rot);
    BVH_REAL deltab = rand_interval(-delta_rot, delta_rot);
    BVH_REAL deltac = rand_interval(-delta_rot, delta_rot);

    btMatrix3x3 M2;
    M2.setEulerZYX(a + deltaa, b + deltab, c + deltac);
    btVector3 v2(x + deltax, y + deltay, z + deltaz);
    btTransform T2(M2, v2);
    transforms2[i] = T2;
  }
}


void generateRandomTransform_ccd(BVH_REAL extents[6], std::vector<btTransform>& transforms, std::vector<btTransform>& transforms2, BVH_REAL delta_trans[3], BVH_REAL delta_rot, int n,
                                 const std::vector<Point>& vertices1, const std::vector<Triangle>& triangles1,
                                 const std::vector<Point>& vertices2, const std::vector<Triangle>& triangles2)
{
  transforms.resize(n);
  transforms2.resize(n);

  for(int i = 0; i < n;)
  {
    BVH_REAL x = rand_interval(extents[0], extents[3]);
    BVH_REAL y = rand_interval(extents[1], extents[4]);
    BVH_REAL z = rand_interval(extents[2], extents[5]);

    const BVH_REAL pi = 3.1415926;
    BVH_REAL a = rand_interval(0, 2 * pi);
    BVH_REAL b = rand_interval(0, 2 * pi);
    BVH_REAL c = rand_interval(0, 2 * pi);

    btMatrix3x3 M;
    M.setEulerZYX(a, b, c);
    btVector3 v(x, y, z);
    btTransform T(M, v);

#if USE_PQP
    if(!collide_PQP(T, vertices1, triangles1, vertices2, triangles2, false))
#else
    if(!collide_Test<OBB>(T, vertices1, triangles1, vertices2, triangles2, SPLIT_METHOD_MEAN, false))
#endif
    {
      transforms[i] = T;

      BVH_REAL deltax = rand_interval(-delta_trans[0], delta_trans[0]);
      BVH_REAL deltay = rand_interval(-delta_trans[1], delta_trans[1]);
      BVH_REAL deltaz = rand_interval(-delta_trans[2], delta_trans[2]);

      BVH_REAL deltaa = rand_interval(-delta_rot, delta_rot);
      BVH_REAL deltab = rand_interval(-delta_rot, delta_rot);
      BVH_REAL deltac = rand_interval(-delta_rot, delta_rot);

      btMatrix3x3 M2;
      M2.setEulerZYX(a + deltaa, b + deltab, c + deltac);
      btVector3 v2(x + deltax, y + deltay, z + deltaz);
      btTransform T2(M2, v2);
      transforms2[i] = T2;
      ++i;
    }
  }
}

#if USE_PQP
bool collide_PQP(const btTransform& T,
                 const std::vector<Point>& vertices1, const std::vector<Triangle>& triangles1,
                 const std::vector<Point>& vertices2, const std::vector<Triangle>& triangles2, bool verbose)
{
  PQP_Model m1, m2;

  m1.BeginModel();
  for(unsigned int i = 0; i < triangles1.size(); ++i)
  {
    Triangle t = triangles1[i];
    Point p1 = vertices1[t[0]];
    Point p2 = vertices1[t[1]];
    Point p3 = vertices1[t[2]];

    btVector3 v1(p1[0], p1[1], p1[2]);
    btVector3 v2(p2[0], p2[1], p2[2]);
    btVector3 v3(p3[0], p3[1], p3[2]);

    v1 = T * v1;
    v2 = T * v2;
    v3 = T * v3;

    PQP_REAL q1[3];
    PQP_REAL q2[3];
    PQP_REAL q3[3];

    q1[0] = v1.x(); q1[1] = v1.y(); q1[2] = v1.z();
    q2[0] = v2.x(); q2[1] = v2.y(); q2[2] = v2.z();
    q3[0] = v3.x(); q3[1] = v3.y(); q3[2] = v3.z();

    m1.AddTri(q1, q2, q3, i);
  }

  m1.EndModel();


  m2.BeginModel();
  for(unsigned int i = 0; i < triangles2.size(); ++i)
  {
    Triangle t = triangles2[i];
    Point p1 = vertices2[t[0]];
    Point p2 = vertices2[t[1]];
    Point p3 = vertices2[t[2]];

    PQP_REAL q1[3];
    PQP_REAL q2[3];
    PQP_REAL q3[3];

    q1[0] = p1[0]; q1[1] = p1[1]; q1[2] = p1[2];
    q2[0] = p2[0]; q2[1] = p2[1]; q2[2] = p2[2];
    q3[0] = p3[0]; q3[1] = p3[1]; q3[2] = p3[2];

    m2.AddTri(q1, q2, q3, i);
  }

  m2.EndModel();


  PQP_CollideResult res;
  PQP_REAL R1[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  PQP_REAL R2[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  PQP_REAL T1[3] = {0, 0, 0};
  PQP_REAL T2[3] = {0, 0, 0};
  PQP_Collide(&res, R1, T1, &m1, R2, T2, &m2);

  if(res.NumPairs() > 0)
  {
    if(verbose)
    {
      std::cout << "in collision " << res.NumPairs() << ": " << std::endl;
      sortCollisionPair_PQP(res.pairs, res.NumPairs());
      for(int i = 0; i < res.NumPairs(); ++i)
      {
        std::cout << "(" << res.Id1(i) << " " << res.Id2(i) << ") ";
      }
      std::cout << std::endl;
    }
    if(verbose) std::cout << res.num_bv_tests << " " << res.num_tri_tests << std::endl;
    return true;
  }
  else
  {
    if(verbose) std::cout << "collision free " << std::endl;
    if(verbose) std::cout << res.num_bv_tests << " " << res.num_tri_tests << std::endl;
    return false;
  }
}


bool collide_PQP2(const btTransform& T,
                 const std::vector<Point>& vertices1, const std::vector<Triangle>& triangles1,
                 const std::vector<Point>& vertices2, const std::vector<Triangle>& triangles2, bool verbose)
{
  PQP_Model m1, m2;

  m1.BeginModel();
  for(unsigned int i = 0; i < triangles1.size(); ++i)
  {
    Triangle t = triangles1[i];
    Point p1 = vertices1[t[0]];
    Point p2 = vertices1[t[1]];
    Point p3 = vertices1[t[2]];

    PQP_REAL q1[3];
    PQP_REAL q2[3];
    PQP_REAL q3[3];

    q1[0] = p1[0]; q1[1] = p1[1]; q1[2] = p1[2];
    q2[0] = p2[0]; q2[1] = p2[1]; q2[2] = p2[2];
    q3[0] = p3[0]; q3[1] = p3[1]; q3[2] = p3[2];

    m1.AddTri(q1, q2, q3, i);
  }

  m1.EndModel();


  m2.BeginModel();
  for(unsigned int i = 0; i < triangles2.size(); ++i)
  {
    Triangle t = triangles2[i];
    Point p1 = vertices2[t[0]];
    Point p2 = vertices2[t[1]];
    Point p3 = vertices2[t[2]];

    PQP_REAL q1[3];
    PQP_REAL q2[3];
    PQP_REAL q3[3];

    q1[0] = p1[0]; q1[1] = p1[1]; q1[2] = p1[2];
    q2[0] = p2[0]; q2[1] = p2[1]; q2[2] = p2[2];
    q3[0] = p3[0]; q3[1] = p3[1]; q3[2] = p3[2];

    m2.AddTri(q1, q2, q3, i);
  }

  m2.EndModel();


  PQP_CollideResult res;
  PQP_REAL R1[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  PQP_REAL R2[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  PQP_REAL T1[3] = {0, 0, 0};
  PQP_REAL T2[3] = {0, 0, 0};
  btVector3 t = T.getOrigin();
  T1[0] = t.x();
  T1[1] = t.y();
  T1[2] = t.z();
  btMatrix3x3 r = T.getBasis();
  for(int i = 0; i < 3; ++i)
  {
    R1[i][0] = r[i].x();
    R1[i][1] = r[i].y();
    R1[i][2] = r[i].z();
  }

  PQP_Collide(&res, R1, T1, &m1, R2, T2, &m2);

  if(res.NumPairs() > 0)
  {
    if(verbose)
    {
      std::cout << "in collision " << res.NumPairs() << ": " << std::endl;
      sortCollisionPair_PQP(res.pairs, res.NumPairs());
      for(int i = 0; i < res.NumPairs(); ++i)
      {
        std::cout << "(" << res.Id1(i) << " " << res.Id2(i) << ") ";
      }
      std::cout << std::endl;
    }
    if(verbose) std::cout << res.num_bv_tests << " " << res.num_tri_tests << std::endl;
    return true;
  }
  else
  {
    if(verbose) std::cout << "collision free " << std::endl;
    if(verbose) std::cout << res.num_bv_tests << " " << res.num_tri_tests << std::endl;
    return false;
  }
}

#endif

template<typename BV>
bool collide_Test(const btTransform& T,
                 const std::vector<Point>& vertices1, const std::vector<Triangle>& triangles1,
                 const std::vector<Point>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method, bool verbose)
{
  BVHModel<BV> m1;
  BVHModel<BV> m2;
  m1.bv_splitter.setSplitType(split_method);
  m2.bv_splitter.setSplitType(split_method);

  std::vector<Point> vertices1_transformed(vertices1.size());
  for(unsigned int i = 0; i < vertices1.size(); ++i)
  {
    Point p = vertices1[i];
    btVector3 v(p[0], p[1], p[2]);
    v = T * v;

    vertices1_transformed[i] = Point(v.x(), v.y(), v.z());
  }

  m1.beginModel();
  m1.addSubModel(vertices1_transformed, triangles1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2, triangles2);
  m2.endModel();

  BVH_CollideResult res;
  res.num_max_contacts = num_max_contacts;
  collide(m1, m2, &res);

  if(res.numPairs() > 0)
  {
    if(verbose)
    {
      std::cout << "in collision " << res.numPairs() << ": " << std::endl;
      sortCollisionPair(res.collidePairs(), res.numPairs());
      for(int i = 0; i < res.numPairs(); ++i)
      {
        std::cout << "(" << res.id1(i) << " " << res.id2(i) << ") ";
      }
      std::cout << std::endl;

      if(res.num_max_contacts > 0)
      {
        BVHCollisionPair* pairs = res.collidePairs();
        for(int i = 0; i < res.numPairs(); ++i)
        {
          Vec3f normal = pairs[i].normal;
          std::cout << "(" << normal[0] << " " << normal[1] << " " << normal[2] << ") ";
        }
        std::cout << std::endl;
        for(int i = 0; i < res.numPairs(); ++i)
        {
          std::cout << "(" << pairs[i].contact_point[0] << " " << pairs[i].contact_point[1] << " " << pairs[i].contact_point[2] << ") ";
        }
        std::cout << std::endl;
      }
    }
    if(verbose) std::cout << res.num_bv_tests << " " << res.num_tri_tests << std::endl;
    return true;
  }
  else
  {
    if(verbose) std::cout << "collision free " << std::endl;
    if(verbose) std::cout << res.num_bv_tests << " " << res.num_tri_tests << std::endl;
    return false;
  }
}


bool collide_Test2(const btTransform& T,
                 const std::vector<Point>& vertices1, const std::vector<Triangle>& triangles1,
                 const std::vector<Point>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method, bool verbose)
{
  BVHModel<OBB> m1;
  BVHModel<OBB> m2;
  m1.bv_splitter.setSplitType(split_method);
  m2.bv_splitter.setSplitType(split_method);

  m1.beginModel();
  m1.addSubModel(vertices1, triangles1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2, triangles2);
  m2.endModel();

  Vec3f R1[3];
  Vec3f R2[3] = {Vec3f(1, 0, 0), Vec3f(0, 1, 0), Vec3f(0, 0, 1)};
  Vec3f T1;
  Vec3f T2(0, 0, 0);

  btVector3 t = T.getOrigin();
  T1 = Vec3f(t.x(), t.y(), t.z());
  btMatrix3x3 r = T.getBasis();
  for(int i = 0; i < 3; ++i)
  {
    R1[i] = Vec3f(r[i].x(), r[i].y(), r[i].z());
  }

  BVH_CollideResult res;
  res.num_max_contacts = num_max_contacts;
  collide(m1, R1, T1, m2, R2, T2, &res);

  if(res.numPairs() > 0)
  {
    if(verbose)
    {
      std::cout << "in collision " << res.numPairs() << ": " << std::endl;
      sortCollisionPair(res.collidePairs(), res.numPairs());
      for(int i = 0; i < res.numPairs(); ++i)
      {
        std::cout << "(" << res.id1(i) << " " << res.id2(i) << ") ";
      }
      std::cout << std::endl;

      if(res.num_max_contacts > 0)
      {
        BVHCollisionPair* pairs = res.collidePairs();
        for(int i = 0; i < res.numPairs(); ++i)
        {
          Vec3f normal = pairs[i].normal;
          std::cout << "(" << normal[0] << " " << normal[1] << " " << normal[2] << ") ";
        }
        std::cout << std::endl;
        for(int i = 0; i < res.numPairs(); ++i)
        {
          std::cout << "(" << pairs[i].contact_point[0] << " " << pairs[i].contact_point[1] << " " << pairs[i].contact_point[2] << ") ";
        }
        std::cout << std::endl;
      }
    }
    if(verbose) std::cout << res.num_bv_tests << " " << res.num_tri_tests << std::endl;
    return true;
  }
  else
  {
    if(verbose) std::cout << "collision free " << std::endl;
    if(verbose) std::cout << res.num_bv_tests << " " << res.num_tri_tests << std::endl;
    return false;
  }
}


template<typename BV>
bool collide_front_Test(const btTransform& T, const btTransform& T2,
                        const std::vector<Point>& vertices1, const std::vector<Triangle>& triangles1,
                        const std::vector<Point>& vertices2, const std::vector<Triangle>& triangles2,
                        SplitMethodType split_method,
                        bool refit_bottomup, bool verbose)
{
  BVHModel<BV> m1;
  BVHModel<BV> m2;
  m1.bv_splitter.setSplitType(split_method);
  m2.bv_splitter.setSplitType(split_method);

  BVHFrontList front_list;

  std::vector<Point> vertices1_transformed(vertices1.size());
  for(unsigned int i = 0; i < vertices1.size(); ++i)
  {
    Point p = vertices1[i];
    btVector3 v(p[0], p[1], p[2]);
    v = T * v;

    vertices1_transformed[i] = Point(v.x(), v.y(), v.z());
  }

  m1.beginModel();
  m1.addSubModel(vertices1_transformed, triangles1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2, triangles2);
  m2.endModel();

  BVH_CollideResult res;
  res.num_max_contacts = std::numeric_limits<int>::max(); // front technique needs all the contacts
  collide(m1, m2, &res, &front_list);

  if(verbose) std::cout << "front list size " << front_list.size() << std::endl;

  // update
  for(unsigned int i = 0; i < vertices1.size(); ++i)
  {
    Point p = vertices1[i];
    btVector3 v(p[0], p[1], p[2]);
    v = T2 * v;

    vertices1_transformed[i] = Point(v.x(), v.y(), v.z());
  }
  m1.beginReplaceModel();
  m1.replaceSubModel(vertices1_transformed);
  m1.endReplaceModel(true, refit_bottomup);

  m2.beginReplaceModel();
  m2.replaceSubModel(vertices2);
  m2.endReplaceModel(true, refit_bottomup);

  BVH_CollideResult res2;
  res2.num_max_contacts = std::numeric_limits<int>::max(); // front technique needs all the contacts
  collide(m1, m2, &res2, &front_list);

  if(res2.numPairs() > 0)
  {
    if(verbose)
    {
      std::cout << "in collision " << res2.numPairs() << ": " << std::endl;
      sortCollisionPair(res2.collidePairs(), res2.numPairs());
      for(int i = 0; i < res2.numPairs(); ++i)
      {
        std::cout << "(" << res2.id1(i) << " " << res2.id2(i) << ") ";
      }
      std::cout << std::endl;
    }

    if(verbose) std::cout << res2.num_bv_tests << " " << res2.num_tri_tests << std::endl;

    return true;
  }
  else
  {
    if(verbose) std::cout << "collision free " << std::endl;
    if(verbose) std::cout << res2.num_bv_tests << " " << res2.num_tri_tests << std::endl;
    return false;
  }
}


template<typename BV>
bool continuous_collide_Test(const btTransform& T, const btTransform& T2,
                        const std::vector<Point>& vertices1, const std::vector<Triangle>& triangles1,
                        const std::vector<Point>& vertices2, const std::vector<Triangle>& triangles2,
                        SplitMethodType split_method,
                        bool refit_bottomup, bool verbose)
{
  BVHModel<BV> m1;
  BVHModel<BV> m2;
  m1.bv_splitter.setSplitType(split_method);
  m2.bv_splitter.setSplitType(split_method);

  BVHFrontList front_list;

  std::vector<Point> vertices1_transformed(vertices1.size());
  for(unsigned int i = 0; i < vertices1.size(); ++i)
  {
    Point p = vertices1[i];
    btVector3 v(p[0], p[1], p[2]);
    v = T * v;

    vertices1_transformed[i] = Point(v.x(), v.y(), v.z());
  }

  m1.beginModel();
  m1.addSubModel(vertices1_transformed, triangles1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2, triangles2);
  m2.endModel();

  // update
  for(unsigned int i = 0; i < vertices1.size(); ++i)
  {
    Point p = vertices1[i];
    btVector3 v(p[0], p[1], p[2]);
    v = T2 * v;

    vertices1_transformed[i] = Point(v.x(), v.y(), v.z());
  }
  m1.beginUpdateModel();
  m1.updateSubModel(vertices1_transformed);
  m1.endUpdateModel(true, refit_bottomup);

  m2.beginUpdateModel();
  m2.updateSubModel(vertices2);
  m2.endUpdateModel(true, refit_bottomup);

  BVH_CollideResult res;

  continuousCollide(m1, m2, &res);

  if(res.numPairs() > 0)
  {
    if(verbose)
    {
      std::cout << "in collision " << res.numPairs() << ": " << std::endl;
      sortCollisionPair(res.collidePairs(), res.numPairs());
      for(int i = 0; i < res.numPairs(); ++i)
      {
        std::cout << "(" << res.id1(i) << " " << res.id2(i) << ") ";
      }
      std::cout << std::endl;
    }

    if(verbose) std::cout << res.num_bv_tests << " " << res.num_vf_tests << " " << res.num_ee_tests << std::endl;

    return true;
  }
  else
  {
    if(verbose) std::cout << "collision free " << std::endl;
    if(verbose) std::cout << res.num_bv_tests << " " << res.num_vf_tests << " " << res.num_ee_tests << std::endl;
    return false;
  }
}


struct CollisionPairComp
{
  bool operator()(const BVHCollisionPair& a, const BVHCollisionPair& b)
  {
    if(a.id1 == b.id1)
      return a.id2 < b.id2;
    return a.id1 < b.id1;
  }
};

void sortCollisionPair(BVHCollisionPair* pairs, int n)
{
  std::vector<BVHCollisionPair> pair_array(n);
  for(int i = 0; i < n; ++i)
    pair_array[i] = pairs[i];

  std::sort(pair_array.begin(), pair_array.end(), CollisionPairComp());

  for(int i = 0; i < n; ++i)
    pairs[i] = pair_array[i];
}

#if USE_PQP
struct CollisionPairComp_PQP
{
  bool operator()(const CollisionPair& a, const CollisionPair& b)
  {
    if(a.id1 == b.id1)
      return a.id2 < b.id2;
    return a.id1 < b.id1;
  }
};

void sortCollisionPair_PQP(CollisionPair* pairs, int n)
{
  std::vector<CollisionPair> pair_array(n);
  for(int i = 0; i < n; ++i)
    pair_array[i] = pairs[i];

  std::sort(pair_array.begin(), pair_array.end(), CollisionPairComp_PQP());

  for(int i = 0; i < n; ++i)
    pairs[i] = pair_array[i];
}
#endif
