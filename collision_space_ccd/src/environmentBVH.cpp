/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Jia Pan */

#include "collision_space_ccd/environmentBVH.h"
#include <geometric_shapes/shape_operations.h>
#include <ros/console.h>
#include <cassert>
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <map>

namespace collision_space_ccd
{
template<typename BV>
EnvironmentModelBVH<BV>::EnvironmentModelBVH(void) : EnvironmentModel()
{
  previous_set_robot_model_ = false;
}

template<typename BV>
EnvironmentModelBVH<BV>::~EnvironmentModelBVH(void)
{
  freeMemory();
}

template<typename BV>
void EnvironmentModelBVH<BV>::freeMemory(void)
{ 
  for(unsigned int j = 0; j < model_geom_.link_geom.size(); ++j)
    delete model_geom_.link_geom[j];
  model_geom_.link_geom.clear();
  for(typename std::map<std::string, CollisionNamespace*>::iterator it = coll_namespaces_.begin(); it != coll_namespaces_.end(); ++it)
    delete it->second;
  coll_namespaces_.clear();
  self_geom_manager.clear();
}

template<typename BV>
void EnvironmentModelBVH<BV>::setRobotModel(const planning_models::KinematicModel* model,
                                                         const AllowedCollisionMatrix& allowed_collision_matrix,
                                                         const std::map<std::string, double>& link_padding_map,
                                                         double default_padding,
                                                         double scale) 
{
  EnvironmentModel::setRobotModel(model, allowed_collision_matrix, link_padding_map, default_padding, scale);
  if(previous_set_robot_model_)
  {
    for(unsigned int j = 0; j < model_geom_.link_geom.size(); ++j)
      delete model_geom_.link_geom[j];
    model_geom_.link_geom.clear();
    self_geom_manager.clear();
    attached_bodies_in_collision_matrix_.clear();
    geom_lookup_map_.clear();
  }
  createBVHRobotModel();
  previous_set_robot_model_ = true;
}

template<typename BV>
void EnvironmentModelBVH<BV>::getAttachedBodyPoses(std::map<std::string, std::vector<btTransform> >& pose_map) const
{
  pose_map.clear();

  const unsigned int n = model_geom_.link_geom.size();    
  for(unsigned int i = 0; i < n; ++i)
  {
    LinkGeom* lg = model_geom_.link_geom[i];
    
    /* create new set of attached bodies */	
    const unsigned int nab = lg->att_bodies.size();
    std::vector<btTransform> nbt;
    for(unsigned int j = 0; j < nab; ++j)
    {
      for(unsigned int k = 0; k < lg->att_bodies[j]->geom.size(); k++)
      {
        nbt.push_back(lg->att_bodies[j]->geom[k]->t1);
      }

      pose_map[lg->att_bodies[j]->att->getName()] = nbt;
    }
  }
}

template<typename BV>
void EnvironmentModelBVH<BV>::createBVHRobotModel()
{
  for(unsigned int i = 0; i < robot_model_->getLinkModels().size(); ++i)
  {
    /* skip this link if we have no geometry or if the link
       name is not specified as enabled for collision
       checking */
    const planning_models::KinematicModel::LinkModel* link = robot_model_->getLinkModels()[i];
    if (!link || !link->getLinkShape())
      continue;
	
    LinkGeom* lg = new LinkGeom();
    lg->link = link;
    if(!default_collision_matrix_.getEntryIndex(link->getName(), lg->index))
    {
      ROS_WARN_STREAM("Link " << link->getName() << " not in provided collision matrix");
    } 

    double padd = default_robot_padding_;
    if(default_link_padding_map_.find(link->getName()) != default_link_padding_map_.end())
    {
      padd = default_link_padding_map_.find(link->getName())->second;
    }
    ROS_DEBUG_STREAM("Link " << link->getName() << " padding " << padd);

    CollisionGeom* unpadd_g = createBVHGeom(link->getLinkShape(), 1.0, 0.0);
    assert(unpadd_g);
    lg->geom.push_back(unpadd_g);
    self_geom_manager.registerGeom(unpadd_g);
    geom_lookup_map_[unpadd_g] = std::pair<std::string, BodyType>(link->getName(), LINK);

    CollisionGeom* padd_g = createBVHGeom(link->getLinkShape(), robot_scale_, padd);
    assert(padd_g);
    lg->padded_geom.push_back(padd_g);
    geom_lookup_map_[padd_g] = std::pair<std::string, BodyType>(link->getName(), LINK);
    const std::vector<planning_models::KinematicModel::AttachedBodyModel*>& attached_bodies = link->getAttachedBodyModels();
    for(unsigned int j = 0 ; j < attached_bodies.size() ; ++j)
    {
      padd = default_robot_padding_;
      if(default_link_padding_map_.find(attached_bodies[j]->getName()) != default_link_padding_map_.end())
        padd = default_link_padding_map_.find(attached_bodies[j]->getName())->second;
      else if(default_link_padding_map_.find("attached") != default_link_padding_map_.end())
        padd = default_link_padding_map_.find("attached")->second;
      addAttachedBody(lg, attached_bodies[j], padd);
    }
    model_geom_.link_geom.push_back(lg);
  }
}

template<typename BV>
CollisionGeom* EnvironmentModelBVH<BV>::createBVHGeom(const shapes::StaticShape *shape)
{
  CollisionGeom* g = NULL;
  switch(shape->type)
  {
  case shapes::PLANE:
  {
    // TODO: plane implementation
    ROS_WARN_STREAM("Plane is not implemented for BVH yet");
  }
  break;
  default:
    break;
  }
  return g;
}

template<typename BV>
CollisionGeom* EnvironmentModelBVH<BV>::createBVHGeom(const shapes::Shape *shape, double scale, double padding)
{
  CollisionGeom* g = NULL;
  switch (shape->type)
  {
  case shapes::SPHERE:
    {
      g = makeSphere<BV>(static_cast<const shapes::Sphere*>(shape)->radius * scale + padding);
    }
    break;
  case shapes::BOX:
    {
      const double *size = static_cast<const shapes::Box*>(shape)->size;
      g = makeBox<BV>(size[0] * scale + padding * 2.0, size[1] * scale + padding * 2.0, size[2] * scale + padding * 2.0);
    }	
    break;
  case shapes::CYLINDER:
    {
      g = makeCylinder<BV>(static_cast<const shapes::Cylinder*>(shape)->radius * scale + padding,
                       static_cast<const shapes::Cylinder*>(shape)->length * scale + padding * 2.0);
    }
    break;
  case shapes::MESH:
    {
      const shapes::Mesh *mesh = static_cast<const shapes::Mesh*>(shape);
      if(mesh->vertexCount > 0 && mesh->triangleCount > 0)
      {		
        std::vector<Triangle> tri_indices(mesh->triangleCount);
        for(unsigned int i = 0; i < mesh->triangleCount; ++i)
          tri_indices[i] = Triangle(mesh->triangles[3 * i], mesh->triangles[3 * i + 1], mesh->triangles[3 * i + 2]);

        std::vector<Point> points(mesh->vertexCount);
        double sx = 0.0, sy = 0.0, sz = 0.0;
        for(unsigned int i = 0; i < mesh->vertexCount; ++i)
        {
          points[i] = Point(mesh->vertices[3 * i], mesh->vertices[3 * i + 1], mesh->vertices[3 * i + 2]);
          sx += points[i][0];
          sy += points[i][1];
          sz += points[i][2];
        }
        // the center of the mesh
        sx /= (double)mesh->vertexCount;
        sy /= (double)mesh->vertexCount;
        sz /= (double)mesh->vertexCount;

        // scale the mesh
        for(unsigned int i = 0; i < mesh->vertexCount; ++i)
        {
          // vector from center to the vertex
          double dx = points[i][0] - sx;
          double dy = points[i][1] - sy;
          double dz = points[i][2] - sz;
		    
          // length of vector
          //double norm = sqrt(dx * dx + dy * dy + dz * dz);
		    
          double ndx = ((dx > 0) ? dx+padding : dx-padding);
          double ndy = ((dy > 0) ? dy+padding : dy-padding);
          double ndz = ((dz > 0) ? dz+padding : dz-padding);

          // the new distance of the vertex from the center
          //double fact = scale + padding/norm;
          points[i] = Point(sx + ndx, sy + ndy, sz + ndz);
        }

        g = makeMesh<BV>(points, tri_indices);
      }
    }
	
  default:
    break;
  }
  return g;
}

template<typename BV>
void EnvironmentModelBVH<BV>::updateGeom(CollisionGeom* geom,  const btTransform &pose) const
{
  geom->applyTransform(pose);
}

template<typename BV>
void EnvironmentModelBVH<BV>::updateAttachedBodies()
{
  updateAttachedBodies(default_link_padding_map_);
}


template<typename BV>
void EnvironmentModelBVH<BV>::updateAttachedBodies(const std::map<std::string, double>& link_padding_map)
{
  //getting rid of all entries associated with the current attached bodies
  for(typename std::map<std::string, bool>::iterator it = attached_bodies_in_collision_matrix_.begin();
      it != attached_bodies_in_collision_matrix_.end();
      it++)
  {
    if(!default_collision_matrix_.removeEntry(it->first))
    {
      ROS_WARN_STREAM("No entry in default collision matrix for attached body " << it->first <<
                      " when there really should be.");
    }
  }

  attached_bodies_in_collision_matrix_.clear();
  for(unsigned int i = 0; i < model_geom_.link_geom.size(); ++i)
  {
    LinkGeom* lg = model_geom_.link_geom[i];

    for(unsigned int j = 0; j < lg->att_bodies.size(); j++)
    {
      for(unsigned int k = 0; k < lg->att_bodies[j]->geom.size(); k++)
      {
        geom_lookup_map_.erase(lg->att_bodies[j]->geom[k]);
        self_geom_manager.unregisterGeom(lg->att_bodies[j]->geom[k]);
      }
      for(unsigned int k = 0; k < lg->att_bodies[j]->padded_geom.size(); k++)
      {
        geom_lookup_map_.erase(lg->att_bodies[j]->padded_geom[k]);
      }
    }
    lg->deleteAttachedBodies();

    /* create new set of attached bodies */
    const std::vector<planning_models::KinematicModel::AttachedBodyModel*>& attached_bodies = lg->link->getAttachedBodyModels();
    for(unsigned int j = 0; j < attached_bodies.size(); ++j)
    {
      double padd = default_robot_padding_;
      if(link_padding_map.find(attached_bodies[j]->getName()) != link_padding_map.end())
      {
        padd = link_padding_map.find(attached_bodies[j]->getName())->second;
      }
      else if (link_padding_map.find("attached") != link_padding_map.end())
      {
        padd = link_padding_map.find("attached")->second;
      }
      ROS_DEBUG_STREAM("Updating attached body " << attached_bodies[j]->getName());      
      addAttachedBody(lg, attached_bodies[j], padd);
    }
  }
}


template<typename BV>
void EnvironmentModelBVH<BV>::addAttachedBody(LinkGeom* lg,
                                                         const planning_models::KinematicModel::AttachedBodyModel* attm,
                                                         double padd)
{

  AttGeom* attg = new AttGeom();
  attg->att = attm;

  if(!default_collision_matrix_.addEntry(attm->getName(), false))
  {
    ROS_WARN_STREAM("Must already have an entry in allowed collision matrix for " << attm->getName());
  } 
  attached_bodies_in_collision_matrix_[attm->getName()] = true;
  default_collision_matrix_.getEntryIndex(attm->getName(), attg->index);
  //setting touch links
  for(unsigned int i = 0; i < attm->getTouchLinks().size(); i++)
  {
    if(!default_collision_matrix_.changeEntry(attm->getName(), attm->getTouchLinks()[i], true))
    {
      ROS_WARN_STREAM("No entry in allowed collision matrix for " << attm->getName() << " and " << attm->getTouchLinks()[i]);
    }
  }
  for(unsigned int i = 0; i < attm->getShapes().size(); i++)
  {
    CollisionGeom* ga = createBVHGeom(attm->getShapes()[i], 1.0, 0.0);
    assert(ga);
    attg->geom.push_back(ga);
    self_geom_manager.registerGeom(ga);
    geom_lookup_map_[ga] = std::pair<std::string, BodyType>(attm->getName(), ATTACHED);    

    CollisionGeom* padd_ga = createBVHGeom(attm->getShapes()[i], robot_scale_, padd);
    assert(padd_ga);
    attg->padded_geom.push_back(padd_ga);
    geom_lookup_map_[padd_ga] = std::pair<std::string, BodyType>(attm->getName(), ATTACHED);    
  }
  lg->att_bodies.push_back(attg);
}


template<typename BV>
void EnvironmentModelBVH<BV>::setAttachedBodiesLinkPadding()
{
  for(unsigned int i = 0; i < model_geom_.link_geom.size(); ++i)
  {
    LinkGeom* lg = model_geom_.link_geom[i];
    
    const std::vector<planning_models::KinematicModel::AttachedBodyModel*>& attached_bodies = lg->link->getAttachedBodyModels();
    for(unsigned int j = 0; j < attached_bodies.size(); ++j)
    {
      double new_padd = -1.0;
      if(altered_link_padding_map_.find(attached_bodies[j]->getName()) != altered_link_padding_map_.end())
      {
        new_padd = altered_link_padding_map_.find(attached_bodies[j]->getName())->second;
      }
      else if(altered_link_padding_map_.find("attached") != altered_link_padding_map_.end())
      {
        new_padd = altered_link_padding_map_.find("attached")->second;
      }
      if(new_padd != -1.0)
      {
        for(unsigned int k = 0; k < attached_bodies[j]->getShapes().size(); k++)
        {
          geom_lookup_map_.erase(lg->att_bodies[j]->padded_geom[k]);
          delete lg->att_bodies[j]->padded_geom[k];
          CollisionGeom* padd_ga = createBVHGeom(attached_bodies[j]->getShapes()[k], robot_scale_, new_padd);
          assert(padd_ga);
          lg->att_bodies[j]->padded_geom[k] = padd_ga;
          geom_lookup_map_[padd_ga] = std::pair<std::string, BodyType>(attached_bodies[j]->getName(), ATTACHED);
        }
      }
    }
  }
}

template<typename BV>
void EnvironmentModelBVH<BV>::revertAttachedBodiesLinkPadding()
{
  for(unsigned int i = 0; i < model_geom_.link_geom.size(); ++i)
  {
    LinkGeom* lg = model_geom_.link_geom[i];
    
    const std::vector<planning_models::KinematicModel::AttachedBodyModel*>& attached_bodies = lg->link->getAttachedBodyModels();
    for(unsigned int j = 0; j < attached_bodies.size(); ++j)
    {
      double new_padd = -1.0;
      if(altered_link_padding_map_.find(attached_bodies[j]->getName()) != altered_link_padding_map_.end())
      {
        new_padd = default_link_padding_map_.find(attached_bodies[j]->getName())->second;
      }
      else if(altered_link_padding_map_.find("attached") != altered_link_padding_map_.end())
      {
        new_padd = default_link_padding_map_.find("attached")->second;
      }
      if(new_padd != -1.0)
      {
        for(unsigned int k = 0; k < attached_bodies[j]->getShapes().size(); k++)
        {
          geom_lookup_map_.erase(lg->att_bodies[j]->padded_geom[k]);
          delete lg->att_bodies[j]->padded_geom[k];
          CollisionGeom* padd_ga = createBVHGeom(attached_bodies[j]->getShapes()[k], robot_scale_, new_padd);
          assert(padd_ga);
          lg->att_bodies[j]->padded_geom[k] = padd_ga;
          geom_lookup_map_[padd_ga] = std::pair<std::string, BodyType>(attached_bodies[j]->getName(), ATTACHED);
        }
      }
    }
  }
}


template<typename BV>
void EnvironmentModelBVH<BV>::updateRobotModel(const planning_models::KinematicState* state)
{ 
  const unsigned int n = model_geom_.link_geom.size();
    
  for(unsigned int i = 0; i < n; ++i)
  {
    const planning_models::KinematicState::LinkState* link_state = state->getLinkState(model_geom_.link_geom[i]->link->getName());
    if(link_state == NULL)
    {
      ROS_WARN_STREAM("No link state for link " << model_geom_.link_geom[i]->link->getName());
      continue;
    }
    updateGeom(model_geom_.link_geom[i]->geom[0], link_state->getGlobalCollisionBodyTransform());
    updateGeom(model_geom_.link_geom[i]->padded_geom[0], link_state->getGlobalCollisionBodyTransform());
    const std::vector<planning_models::KinematicState::AttachedBodyState*>& attached_bodies = link_state->getAttachedBodyStateVector();
    for(unsigned int j = 0; j < attached_bodies.size(); ++j)
    {
      for(unsigned int k = 0; k < attached_bodies[j]->getGlobalCollisionBodyTransforms().size(); k++)
      {
        updateGeom(model_geom_.link_geom[i]->att_bodies[j]->geom[k], attached_bodies[j]->getGlobalCollisionBodyTransforms()[k]);
        updateGeom(model_geom_.link_geom[i]->att_bodies[j]->padded_geom[k], attached_bodies[j]->getGlobalCollisionBodyTransforms()[k]);
      }
    }
  }

  self_geom_manager.update();
}

template<typename BV>
void EnvironmentModelBVH<BV>::setAlteredLinkPadding(const std::map<std::string, double>& new_link_padding)
{
  
  //updating altered map
  EnvironmentModel::setAlteredLinkPadding(new_link_padding);

  for(unsigned int i = 0; i < model_geom_.link_geom.size(); i++)
  {
    LinkGeom* lg = model_geom_.link_geom[i];

    if(altered_link_padding_map_.find(lg->link->getName()) != altered_link_padding_map_.end())
    {
      double new_padding = altered_link_padding_map_.find(lg->link->getName())->second;
      const planning_models::KinematicModel::LinkModel* link = lg->link;
      if (!link || !link->getLinkShape())
      {
        ROS_WARN_STREAM("Can't get kinematic model for link " << link->getName() << " to make new padding");
        continue;
      }
      ROS_DEBUG_STREAM("Setting padding for link " << lg->link->getName() << " from " 
                       << default_link_padding_map_[lg->link->getName()] 
                       << " to " << new_padding);
      //otherwise we clear out the data associated with the old one
      for(unsigned int j = 0; j < lg->padded_geom.size() ; ++j)
      {
        geom_lookup_map_.erase(lg->padded_geom[j]);
        delete lg->padded_geom[j];
      }
      lg->padded_geom.clear();
      CollisionGeom* g = createBVHGeom(link->getLinkShape(), robot_scale_, new_padding);
      assert(g);
      lg->padded_geom.push_back(g);
      geom_lookup_map_[g] = std::pair<std::string, BodyType>(link->getName(), LINK);
    }
  }
  //this does all the work
  setAttachedBodiesLinkPadding();  
}

template<typename BV>
void EnvironmentModelBVH<BV>::revertAlteredLinkPadding()
{
  for(unsigned int i = 0; i < model_geom_.link_geom.size(); i++)
  {
    LinkGeom* lg = model_geom_.link_geom[i];

    if(altered_link_padding_map_.find(lg->link->getName()) != altered_link_padding_map_.end())
    {
      double old_padding = default_link_padding_map_.find(lg->link->getName())->second;
      const planning_models::KinematicModel::LinkModel* link = lg->link;
      if (!link || !link->getLinkShape())
      {
        ROS_WARN_STREAM("Can't get kinematic model for link " << link->getName() << " to revert to old padding");
        continue;
      }
      //otherwise we clear out the data associated with the old one
      for(unsigned int j = 0; j < lg->padded_geom.size(); ++j)
      {
        geom_lookup_map_.erase(lg->padded_geom[j]);
        delete lg->padded_geom[j];
      }
      ROS_DEBUG_STREAM("Reverting padding for link " << lg->link->getName() << " from " << altered_link_padding_map_[lg->link->getName()]
                      << " to " << old_padding);
      lg->padded_geom.clear();
      CollisionGeom* g = createBVHGeom(link->getLinkShape(), robot_scale_, old_padding);
      assert(g);
      lg->padded_geom.push_back(g);
      geom_lookup_map_[g] = std::pair<std::string, BodyType>(link->getName(), LINK);
    }
  }
  revertAttachedBodiesLinkPadding();
  
  //clears altered map
  EnvironmentModel::revertAlteredLinkPadding();
} 

template<typename BV>
bool EnvironmentModelBVH<BV>::getCollisionContacts(const std::vector<AllowedContact>& allowedContacts, std::vector<Contact>& contacts, unsigned int max_count) const
{
  contacts.clear();
  CollisionData cdata;
  cdata.allowed_collision_matrix = &getCurrentAllowedCollisionMatrix();
  cdata.geom_lookup_map = &geom_lookup_map_;
  cdata.contacts = &contacts;
  cdata.max_contacts = max_count;
  if (!allowedContacts.empty())
    cdata.allowed = &allowedContacts;
  contacts.clear();
  testCollision(&cdata);
  return cdata.collides;
}

template<typename BV>
bool EnvironmentModelBVH<BV>::getAllCollisionContacts(const std::vector<AllowedContact>& allowedContacts, std::vector<Contact>& contacts, unsigned int num_contacts_per_pair) const
{
  contacts.clear();
  CollisionData cdata;
  cdata.geom_lookup_map = &geom_lookup_map_;
  cdata.allowed_collision_matrix = &getCurrentAllowedCollisionMatrix();
  cdata.contacts = &contacts;
  cdata.max_contacts = num_contacts_per_pair;
  if (!allowedContacts.empty())
    cdata.allowed = &allowedContacts;
  cdata.exhaustive = true;
  contacts.clear();
  testCollision(&cdata);
  return cdata.collides;
}


template<typename BV>
bool EnvironmentModelBVH<BV>::isCollision(void) const
{
  CollisionData cdata;
  cdata.allowed_collision_matrix = &getCurrentAllowedCollisionMatrix();
  cdata.geom_lookup_map = &geom_lookup_map_;
  testCollision(&cdata);
  return cdata.collides;
}

template<typename BV>
bool EnvironmentModelBVH<BV>::isSelfCollision(void) const
{
  CollisionData cdata; 
  cdata.geom_lookup_map = &geom_lookup_map_;
  cdata.allowed_collision_matrix = &getCurrentAllowedCollisionMatrix();
  testSelfCollision(&cdata);
  return cdata.collides;
}

template<typename BV>
bool EnvironmentModelBVH<BV>::isEnvironmentCollision(void) const
{
  CollisionData cdata; 
  cdata.geom_lookup_map = &geom_lookup_map_;
  cdata.allowed_collision_matrix = &getCurrentAllowedCollisionMatrix();
  testEnvironmentCollision(&cdata);
  return cdata.collides;
}

template<typename BV>
void EnvironmentModelBVH<BV>::testObjectCollision(CollisionNamespace *cn, CollisionData *cdata) const
{ 
  cn->env_geom_manager.setup();
  for(int i = model_geom_.link_geom.size() - 1; i >= 0 && (!cdata->done || cdata->exhaustive); --i)
  {
    LinkGeom* lg = model_geom_.link_geom[i];
    
    bool allowed = false;
    if(cdata->allowed_collision_matrix)
    {
      if(!cdata->allowed_collision_matrix->getAllowedCollision(cn->name, lg->link->getName(), allowed))
      {
        ROS_WARN_STREAM("No entry in cdata allowed collision matrix for " << cn->name << " and " << lg->link->getName());
        return;
      } 
    }
    
    //have to test collisions with link
    if(!allowed)
    {
      for(unsigned int j = 0; j < lg->padded_geom.size(); j++)
      {
        //have to figure
        unsigned int current_contacts_size = 0;
        if(cdata->contacts)
        {
          current_contacts_size = cdata->contacts->size();
        }

        //Collide core
        cn->env_geom_manager.collide(lg->padded_geom[j], cdata);

        if(cdata->contacts && cdata->contacts->size() > current_contacts_size)
        {
          //new contacts must mean collision
          for(unsigned int k = current_contacts_size; k < cdata->contacts->size(); k++)
          {
            if(cdata->contacts->at(k).body_type_1 == OBJECT)
            {
              cdata->contacts->at(k).body_name_1 = cn->name;
            }
            else if(cdata->contacts->at(k).body_type_2 == OBJECT)
            {
              cdata->contacts->at(k).body_name_2 = cn->name;
            }
            else
            {
              ROS_WARN_STREAM("New contacts really should have an object as one of the bodies");
            }
          }
        }
        if(cdata->done)
        {
          return;
        }
      }
    }
    //now we need to do the attached bodies
    for(unsigned int j = 0; j < lg->att_bodies.size(); j++)
    {
      std::string att_name = lg->att_bodies[j]->att->getName();
      allowed = false;
      if(cdata->allowed_collision_matrix)
      {
        if(!cdata->allowed_collision_matrix->getAllowedCollision(cn->name, att_name, allowed))
        {
          ROS_WARN_STREAM("No entry in current allowed collision matrix for " << cn->name << " and " << att_name);
          return;
        }
      }
      if(!allowed)
      {
        for(unsigned int k = 0; k < lg->att_bodies[j]->padded_geom.size(); k++)
        {
          //have to figure
          unsigned int current_contacts_size = 0;
          if(cdata->contacts)
          {
            current_contacts_size = cdata->contacts->size();
          }

          //collision core
          cn->env_geom_manager.collide(lg->att_bodies[j]->padded_geom[k], cdata);


          if(cdata->contacts && cdata->contacts->size() > current_contacts_size)
          {
            //new contacts must mean collision
            for(unsigned int l = current_contacts_size; l < cdata->contacts->size(); l++)
            {
              if(cdata->contacts->at(l).body_type_1 == OBJECT)
              {
                cdata->contacts->at(l).body_name_1 = cn->name;
              }
              else if(cdata->contacts->at(l).body_type_2 == OBJECT)
              {
                cdata->contacts->at(l).body_name_2 = cn->name;
              }
              else
              {
                ROS_WARN_STREAM("New contacts really should have an object as one of the bodys");
              }
            }
          }
          if(cdata->done)
          {
            return;
          }
        }
      } 
    }
  }
}


template<typename BV>
void EnvironmentModelBVH<BV>::testGeomCollision(CollisionData* cdata, CollisionGeom* o1, CollisionGeom* o2)
{
  if(cdata->done && !cdata->exhaustive)
  {
    return;
  }

  // first figure out what check is happening
  bool check_in_allowed_collision_matrix = true;

  std::map<CollisionGeom*, std::pair<std::string, EnvironmentModelBVH::BodyType> >::const_iterator it1 = cdata->geom_lookup_map->find(o1);
  std::map<CollisionGeom*, std::pair<std::string, EnvironmentModelBVH::BodyType> >::const_iterator it2 = cdata->geom_lookup_map->find(o2);

  if(it1 != cdata->geom_lookup_map->end())
  {
    cdata->body_name_1 = it1->second.first;
    cdata->body_type_1 = it1->second.second;
  }
  else
  {
    cdata->body_name_1 = "";
    cdata->body_type_1 = EnvironmentModelBVH::OBJECT;
    check_in_allowed_collision_matrix = false;
  }

  if(it2 != cdata->geom_lookup_map->end())
  {
    cdata->body_name_2 = it2->second.first;
    cdata->body_type_2 = it2->second.second;
  }
  else
  {
    cdata->body_name_2 = "";
    cdata->body_type_2 = EnvironmentModelBVH::OBJECT;
    check_in_allowed_collision_matrix = false;
  }

  // determine whether or not this collision is allowed in the self_collision matrix
  if (cdata->allowed_collision_matrix && check_in_allowed_collision_matrix)
  {
    bool allowed;
    if(!cdata->allowed_collision_matrix->getAllowedCollision(cdata->body_name_1, cdata->body_name_2, allowed))
    {
      ROS_WARN_STREAM("No entry in allowed collision matrix for " << cdata->body_name_1 << " and " << cdata->body_name_2);
      return;
    }
    if(allowed)
    {
      ROS_DEBUG_STREAM("Collision but allowed touch between " << cdata->body_name_1 << " and " << cdata->body_name_2);
      return;
    }
    else
    {
      ROS_DEBUG_STREAM("Collision and no allowed touch between " << cdata->body_name_1 << " and " << cdata->body_name_2);
    }
  }

  // do the actual collision check to get the desired number of contacts
  unsigned int num_contacts = 1;
  if(cdata->contacts)
  {
    num_contacts = cdata->max_contacts;
  }
  num_contacts = std::max(num_contacts, (unsigned int)1);

  if(!cdata->contacts)
  {
    BVH_CollideResult res = o1->collide(o2);
    int num_c = res.numPairs();

    // we don't care about contact information, so just set to true if there's been collision
    if(num_c)
    {
      cdata->collides = true;
      cdata->done = true;
    }
  }
  else
  {
    BVH_CollideResult res = o1->collide(o2, num_contacts);
    int num_c = res.numPairs();

    BVHCollisionPair* pairs = res.collidePairs();
    for(int i = 0; i < num_c; ++i)
    {
      //already enough contacts, so just quit
      if(!cdata->exhaustive && cdata->max_contacts > 0 && cdata->contacts->size() >= cdata->max_contacts)
      {
        break;
      }

      Vec3f normal = pairs[i].normal;
      BVH_REAL depth = pairs[i].penetration_depth;
      Vec3f contact = pairs[i].contact_point;

      ROS_DEBUG_STREAM("Contact at " << contact[0] << " " << contact[1] << " " << contact[2]);
      btVector3 pos(contact[0], contact[1], contact[2]);

      //figure out whether the contact is allowed
      //allowed contacts only allowed with objects for now
      if(cdata->allowed && (cdata->body_type_1 == EnvironmentModelBVH::OBJECT || cdata->body_type_2 == EnvironmentModelBVH::OBJECT))
      {
        std::string body_name;
        if(cdata->body_type_1 != EnvironmentModelBVH::OBJECT)
        {
          body_name = cdata->body_name_1;
        }
        else
        {
          body_name = cdata->body_name_2;
        }

        bool allow = false;
        for(unsigned int j = 0; !allow && j < cdata->allowed->size(); ++j)
        {
          if(cdata->allowed->at(j).bound->containsPoint(pos) && cdata->allowed->at(j).depth > fabs(depth))
          {
            for(unsigned int k = 0; k < cdata->allowed->at(j).links.size(); ++k)
            {
              if(cdata->allowed->at(j).links[k] == body_name)
              {
                allow = true;
                break;
              }
            }
          }
        }

        if (allow)
          continue;
      }

      cdata->collides = true;

      EnvironmentModelBVH::Contact add;
      add.pos = pos;
      add.normal = btVector3(normal[0], normal[1], normal[2]);
      add.depth = depth;
      add.body_name_1 = cdata->body_name_1;
      add.body_name_2 = cdata->body_name_2;
      add.body_type_1 = cdata->body_type_1;
      add.body_type_2 = cdata->body_type_2;

      cdata->contacts->push_back(add);
    }

    if (!cdata->exhaustive && cdata->max_contacts > 0 && cdata->contacts->size() >= cdata->max_contacts)
      cdata->done = true;
  }
}


template<typename BV>
void EnvironmentModelBVH<BV>::SAPManager::unregisterGeom(CollisionGeom* geom)
{
  // must sorted before
  setup();

  EndPoint g;
  g.value = geom->aabb.min_[0];
  typename std::vector<EndPoint>::iterator start1 = std::lower_bound(endpoints[0].begin(), endpoints[0].end(), g, SortByValue());
  g.value = geom->aabb.max_[0];
  typename std::vector<EndPoint>::iterator end1 = std::upper_bound(start1, endpoints[0].end(), g, SortByValue());

  if(start1 < end1)
  {
    unsigned int start_id = start1 - endpoints[0].begin();
    unsigned int end_id = end1 - endpoints[0].begin();
    unsigned int cur_id = start_id;
    for(unsigned int i = start_id; i < end_id; ++i)
    {
      if(endpoints[0][i].g != geom)
      {
        if(i == cur_id) cur_id++;
        else
        {
          endpoints[0][cur_id] = endpoints[0][i];
          cur_id++;
        }
      }
    }
    if(cur_id < end_id)
    {
      endpoints[0].resize(endpoints[0].size() - 2);
    }
  }

  g.value = geom->aabb.min_[1];
  typename std::vector<EndPoint>::iterator start2 = std::lower_bound(endpoints[1].begin(), endpoints[1].end(), g, SortByValue());
  g.value = geom->aabb.max_[1];
  typename std::vector<EndPoint>::iterator end2 = std::upper_bound(start2, endpoints[1].end(), g, SortByValue());

  if(start2 < end2)
  {
    unsigned int start_id = start2 - endpoints[1].begin();
    unsigned int end_id = end2 - endpoints[1].begin();
    unsigned int cur_id = start_id;
    for(unsigned int i = start_id; i < end_id; ++i)
    {
      if(endpoints[1][i].g != geom)
      {
        if(i == cur_id) cur_id++;
        else
        {
          endpoints[1][cur_id] = endpoints[1][i];
          cur_id++;
        }
      }
    }
    if(cur_id < end_id)
    {
      endpoints[1].resize(endpoints[1].size() - 2);
    }
  }


  g.value = geom->aabb.min_[2];
  typename std::vector<EndPoint>::iterator start3 = std::lower_bound(endpoints[2].begin(), endpoints[2].end(), g, SortByValue());
  g.value = geom->aabb.max_[2];
  typename std::vector<EndPoint>::iterator end3 = std::upper_bound(start3, endpoints[2].end(), g, SortByValue());

  if(start3 < end3)
  {
    unsigned int start_id = start3 - endpoints[2].begin();
    unsigned int end_id = end3 - endpoints[2].begin();
    unsigned int cur_id = start_id;
    for(unsigned int i = start_id; i < end_id; ++i)
    {
      if(endpoints[2][i].g != geom)
      {
        if(i == cur_id) cur_id++;
        else
        {
          endpoints[2][cur_id] = endpoints[2][i];
          cur_id++;
        }
      }
    }
    if(cur_id < end_id)
    {
      endpoints[2].resize(endpoints[2].size() - 2);
    }
  }
}


template<typename BV>
void EnvironmentModelBVH<BV>::SAPManager::registerGeom(CollisionGeom* geom)
{
  EndPoint p, q;

  p.g = geom;
  q.g = geom;
  p.type = 0;
  q.type = 1;
  p.value = geom->aabb.min_[0];
  q.value = geom->aabb.max_[0];
  endpoints[0].push_back(p);
  endpoints[0].push_back(q);

  p.value = geom->aabb.min_[1];
  q.value = geom->aabb.max_[1];
  endpoints[1].push_back(p);
  endpoints[1].push_back(q);

  p.value = geom->aabb.min_[2];
  q.value = geom->aabb.max_[2];
  endpoints[2].push_back(p);
  endpoints[2].push_back(q);
  setup_ = false;
}

template<typename BV>
void EnvironmentModelBVH<BV>::SAPManager::setup()
{
  if(!setup_)
  {
    std::sort(endpoints[0].begin(), endpoints[0].end(), SortByValue());
    std::sort(endpoints[1].begin(), endpoints[1].end(), SortByValue());
    std::sort(endpoints[2].begin(), endpoints[2].end(), SortByValue());

    for(int i = 0; i < 3; ++i)
      delete interval_trees[i];

    for(int i = 0; i < 3; ++i)
      interval_trees[i] = new IntervalTree;

    for(unsigned int i = 0; i < endpoints[0].size(); ++i)
    {
      EndPoint p = endpoints[0][i];
      CollisionGeom* g = p.g;
      if(p.type == 0)
      {
        SAPInterval* ivl1 = new SAPInterval(g->aabb.min_[0], g->aabb.max_[0], g);
        SAPInterval* ivl2 = new SAPInterval(g->aabb.min_[1], g->aabb.max_[1], g);
        SAPInterval* ivl3 = new SAPInterval(g->aabb.min_[2], g->aabb.max_[2], g);
        interval_trees[0]->insert(ivl1);
        interval_trees[1]->insert(ivl2);
        interval_trees[2]->insert(ivl3);
      }
    }

    setup_ = true;
  }
}

template<typename BV>
void EnvironmentModelBVH<BV>::SAPManager::clear()
{
  endpoints[0].clear();
  endpoints[1].clear();
  endpoints[2].clear();
  setup_ = false;
}

template<typename BV>
void EnvironmentModelBVH<BV>::SAPManager::getGeoms(std::vector<CollisionGeom*>& geoms) const
{
  geoms.resize(endpoints[0].size() / 2);
  unsigned int j = 0;
  for(unsigned int i = 0; i < endpoints[0].size(); ++i)
  {
    if(endpoints[0][i].type == 0)
    {
      geoms[j] = endpoints[0][i].g; j++;
    }
  }
}

template<typename BV>
void EnvironmentModelBVH<BV>::SAPManager::update()
{
  setup_ = false;

  for(unsigned int i = 0; i < endpoints[0].size(); ++i)
  {
    if(endpoints[0][i].type == 0)
      endpoints[0][i].value = endpoints[0][i].g->aabb.min_[0];
    else
      endpoints[0][i].value = endpoints[0][i].g->aabb.max_[0];
  }

  for(unsigned int i = 0; i < endpoints[1].size(); ++i)
  {
    if(endpoints[1][i].type == 0)
      endpoints[1][i].value = endpoints[1][i].g->aabb.min_[1];
    else
      endpoints[1][i].value = endpoints[1][i].g->aabb.max_[1];
  }

  for(unsigned int i = 0; i < endpoints[2].size(); ++i)
  {
    if(endpoints[2][i].type == 0)
      endpoints[2][i].value = endpoints[2][i].g->aabb.min_[2];
    else
      endpoints[2][i].value = endpoints[2][i].g->aabb.max_[2];
  }

  setup();
}

template<typename BV>
void EnvironmentModelBVH<BV>::SAPManager::collide(CollisionGeom* geom, CollisionData* cdata) const
{
  static const unsigned int CUTOFF = 100;

  std::deque<Interval*> results0, results1, results2;

  results0 = interval_trees[0]->query(geom->aabb.min_[0], geom->aabb.max_[0]);
  if(results0.size() > CUTOFF)
  {
    results1 = interval_trees[1]->query(geom->aabb.min_[1], geom->aabb.max_[1]);
    if(results1.size() > CUTOFF)
    {
      results2 = interval_trees[2]->query(geom->aabb.min_[2], geom->aabb.max_[2]);
      if(results2.size() > CUTOFF)
      {
        int d1 = results0.size();
        int d2 = results1.size();
        int d3 = results2.size();

        if(d1 >= d2 && d1 >= d3)
        {
          for(unsigned int i = 0; i < results0.size(); ++i)
          {
            SAPInterval* ivl = (SAPInterval*)results0[i];
            EnvironmentModelBVH<BV>::testGeomCollision(cdata, ivl->g, geom);
          }
        }
        else if(d2 >= d1 && d2 >= d3)
        {
          for(unsigned int i = 0; i < results1.size(); ++i)
          {
            SAPInterval* ivl = (SAPInterval*)results1[i];
            EnvironmentModelBVH<BV>::testGeomCollision(cdata, ivl->g, geom);
          }
        }
        else
        {
          for(unsigned int i = 0; i < results2.size(); ++i)
          {
            SAPInterval* ivl = (SAPInterval*)results2[i];
            EnvironmentModelBVH<BV>::testGeomCollision(cdata, ivl->g, geom);
          }
        }
      }
      else
      {
        for(unsigned int i = 0; i < results2.size(); ++i)
        {
          SAPInterval* ivl = (SAPInterval*)results2[i];
          EnvironmentModelBVH<BV>::testGeomCollision(cdata, ivl->g, geom);
        }
      }
    }
    else
    {
      for(unsigned int i = 0; i < results1.size(); ++i)
      {
        SAPInterval* ivl = (SAPInterval*)results1[i];
        EnvironmentModelBVH<BV>::testGeomCollision(cdata, ivl->g, geom);
      }
    }
  }
  else
  {
    for(unsigned int i = 0; i < results0.size(); ++i)
    {
      SAPInterval* ivl = (SAPInterval*)results0[i];
      EnvironmentModelBVH<BV>::testGeomCollision(cdata, ivl->g, geom);
    }
  }

  results0.clear();
  results1.clear();
  results2.clear();

  //checkColl(endpoints[0].begin(), endpoints[0].end(), geom, cdata);
}

template<typename BV>
void EnvironmentModelBVH<BV>::SAPManager::collide(CollisionData* cdata) const
{
  std::set<CollisionGeom*> active;
  std::set<std::pair<CollisionGeom*, CollisionGeom*> > overlap;
  unsigned int n = endpoints[0].size();
  double diff_x = endpoints[0][0].value - endpoints[0][n-1].value;
  double diff_y = endpoints[1][0].value - endpoints[1][n-1].value;
  double diff_z = endpoints[2][0].value - endpoints[2][n-1].value;

  int axis = 0;
  if(diff_y > diff_x && diff_y > diff_z)
    axis = 1;
  else if(diff_z > diff_y && diff_z > diff_x)
    axis = 2;

  for(unsigned int i = 0; i < n; ++i)
  {
    const EndPoint& endpoint = endpoints[axis][i];
    CollisionGeom* index = endpoint.g;
    if(endpoint.type == 0)
    {
      std::set<CollisionGeom*>::iterator iter = active.begin();
      std::set<CollisionGeom*>::iterator end = active.end();
      for(; iter != end; ++iter)
      {
        CollisionGeom* active_index = *iter;
        const AABB& b0 = active_index->aabb;
        const AABB& b1 = index->aabb;

        int axis2 = (axis + 1) % 3;
        int axis3 = (axis + 2) % 3;

        if(b0.axisOverlap(b1, axis2)
          && b0.axisOverlap(b1, axis3))
        {
          std::pair<std::set<std::pair<CollisionGeom*, CollisionGeom*> >::iterator, bool> insert_res;
          if(active_index < index)
            insert_res = overlap.insert(std::make_pair(active_index, index));
          else
            insert_res = overlap.insert(std::make_pair(index, active_index));

          if(insert_res.second)
          {
            EnvironmentModelBVH<BV>::testGeomCollision(cdata, active_index, index);
            if (cdata->done && !cdata->exhaustive)
              return;
          }
        }
      }
      active.insert(index);
    }
    else
      active.erase(index);
  }
}

template<typename BV>
void EnvironmentModelBVH<BV>::SAPManager::checkColl(typename std::vector<EndPoint>::const_iterator start, typename std::vector<EndPoint>::const_iterator end, CollisionGeom* geom, CollisionData* cdata) const
{
  std::set<CollisionGeom*> mask;

  for(typename std::vector<EndPoint>::const_iterator pos = start; pos != end; ++pos)
  {
    std::pair<std::set<CollisionGeom*>::iterator,bool> res = mask.insert(pos->g);
    if(res.second)
    {
      if(pos->g->aabb.overlap(geom->aabb))
      {
        EnvironmentModelBVH<BV>::testGeomCollision(cdata, pos->g, geom);
      }
    }
  }
}

template<typename BV>
void EnvironmentModelBVH<BV>::testCollision(CollisionData *cdata) const
{
  testSelfCollision(cdata);
  testEnvironmentCollision(cdata);    
}

template<typename BV>
void EnvironmentModelBVH<BV>::testSelfCollision(CollisionData *cdata) const
{
  self_geom_manager.setup();

  self_geom_manager.collide(cdata);
}

template<typename BV>
void EnvironmentModelBVH<BV>::testEnvironmentCollision(CollisionData *cdata) const
{
  /* check collision with other bodies until done*/
  for(typename std::map<std::string, CollisionNamespace*>::const_iterator it = coll_namespaces_.begin() ; it != coll_namespaces_.end() && !cdata->done ; ++it)
  {
    testObjectCollision(it->second, cdata);
  }
}

template<typename BV>
bool EnvironmentModelBVH<BV>::hasObject(const std::string& ns)
{
  if(coll_namespaces_.find(ns) != coll_namespaces_.end())
  {
    return true;
  }
  return false;
}

template<typename BV>
void EnvironmentModelBVH<BV>::addObjects(const std::string& ns, const std::vector<shapes::Shape*>& shapes, const std::vector<btTransform>& poses)
{
  assert(shapes.size() == poses.size());
  typename std::map<std::string, CollisionNamespace*>::iterator it = coll_namespaces_.find(ns);
  CollisionNamespace* cn = NULL;
  if(it == coll_namespaces_.end())
  {
    cn = new CollisionNamespace(ns);
    coll_namespaces_[ns] = cn;
    default_collision_matrix_.addEntry(ns, false);
  }
  else
  {
     cn = it->second;
  }

  //we're going to create the namespace in objects_ even if it doesn't have anything in it
  objects_->addObjectNamespace(ns);

  unsigned int n = shapes.size();
  for(unsigned int i = 0; i < n; ++i)
  {
    CollisionGeom* g = createBVHGeom(shapes[i], 1.0, 0.0);
    assert(g);
    updateGeom(g, poses[i]);
    cn->geoms.push_back(g);
    cn->env_geom_manager.registerGeom(g);
    objects_->addObject(ns, shapes[i], poses[i]);
  }
}

template<typename BV>
void EnvironmentModelBVH<BV>::addObject(const std::string& ns, shapes::Shape* shape, const btTransform& pose)
{
  typename std::map<std::string, CollisionNamespace*>::iterator it = coll_namespaces_.find(ns);
  CollisionNamespace* cn = NULL;
  if(it == coll_namespaces_.end())
  {
    cn = new CollisionNamespace(ns);
    coll_namespaces_[ns] = cn;
    default_collision_matrix_.addEntry(ns, false);
  }
  else
    cn = it->second;

  CollisionGeom* g = createBVHGeom(shape, 1.0, 0.0);
  assert(g);

  default_collision_matrix_.addEntry(ns, false);

  updateGeom(g, pose);
  cn->geoms.push_back(g);
  objects_->addObject(ns, shape, pose);
}

template<typename BV>
void EnvironmentModelBVH<BV>::addObject(const std::string& ns, shapes::StaticShape* shape)
{   
  typename std::map<std::string, CollisionNamespace*>::iterator it = coll_namespaces_.find(ns);
  CollisionNamespace* cn = NULL;
  if(it == coll_namespaces_.end())
  {
    cn = new CollisionNamespace(ns);
    coll_namespaces_[ns] = cn;
    default_collision_matrix_.addEntry(ns, false);
  }
  else
    cn = it->second;

  CollisionGeom* g = createBVHGeom(shape);
  assert(g);
  cn->geoms.push_back(g);
  objects_->addObject(ns, shape);
}

template<typename BV>
void EnvironmentModelBVH<BV>::clearObjects(void)
{
  for(typename std::map<std::string, CollisionNamespace*>::iterator it = coll_namespaces_.begin(); it != coll_namespaces_.end(); ++it)
  {
    default_collision_matrix_.removeEntry(it->first);
    delete it->second;
  }
  coll_namespaces_.clear();
  objects_->clearObjects();
}

template<typename BV>
void EnvironmentModelBVH<BV>::clearObjects(const std::string &ns)
{
  typename std::map<std::string, CollisionNamespace*>::iterator it = coll_namespaces_.find(ns);
  if(it != coll_namespaces_.end())
  {
    default_collision_matrix_.removeEntry(ns);
    delete it->second;
    coll_namespaces_.erase(ns);
  }
  objects_->clearObjects(ns);
}

template<typename BV>
CollisionGeom* EnvironmentModelBVH<BV>::copyGeom(CollisionGeom* geom) const
{
  // TODO
  return NULL;
  /*
  int c = dGeomGetClass(geom);
  dGeomID ng = NULL;
  bool location = true;
  switch (c)
  {
  case dSphereClass:
    ng = dCreateSphere(space, dGeomSphereGetRadius(geom));
    break;
  case dBoxClass:
    {
      dVector3 r;
      dGeomBoxGetLengths(geom, r);
      ng = dCreateBox(space, r[0], r[1], r[2]);
    }
    break;
  case dCylinderClass:
    {
      dReal r, l;
      dGeomCylinderGetParams(geom, &r, &l);
      ng = dCreateCylinder(space, r, l);
    }
    break;
  case dPlaneClass:
    {
      dVector4 p;
      dGeomPlaneGetParams(geom, p);
      ng = dCreatePlane(space, p[0], p[1], p[2], p[3]);
      location = false;
    }
    break;
  case dTriMeshClass:
    {
      dTriMeshDataID tdata = dGeomTriMeshGetData(geom);
      dTriMeshDataID cdata = dGeomTriMeshDataCreate();
      for (unsigned int i = 0 ; i < sourceStorage.mesh.size() ; ++i)
        if (sourceStorage.mesh[i].data == tdata)
        {
          unsigned int p = storage.mesh.size();
          storage.mesh.resize(p + 1);
          storage.mesh[p].n_vertices = sourceStorage.mesh[i].n_vertices;
          storage.mesh[p].n_indices = sourceStorage.mesh[i].n_indices;
          storage.mesh[p].indices = new dTriIndex[storage.mesh[p].n_indices];
          for (int j = 0 ; j < storage.mesh[p].n_indices ; ++j)
            storage.mesh[p].indices[j] = sourceStorage.mesh[i].indices[j];
          storage.mesh[p].vertices = new double[storage.mesh[p].n_vertices];
          for (int j = 0 ; j < storage.mesh[p].n_vertices ; ++j)
            storage.mesh[p].vertices[j] = sourceStorage.mesh[i].vertices[j];
          dGeomTriMeshDataBuildDouble(cdata, storage.mesh[p].vertices, sizeof(double) * 3, storage.mesh[p].n_vertices, storage.mesh[p].indices, storage.mesh[p].n_indices, sizeof(dTriIndex) * 3);
          storage.mesh[p].data = cdata;
          break;
        }
      ng = dCreateTriMesh(space, cdata, NULL, NULL, NULL);
    }
    break;
  default:
    assert(0); // this should never happen
    break;
  }
    
  if (ng && location)
  {
    const dReal *pos = dGeomGetPosition(geom);
    dGeomSetPosition(ng, pos[0], pos[1], pos[2]);
    dQuaternion q;
    dGeomGetQuaternion(geom, q);
    dGeomSetQuaternion(ng, q);
  }
    
  return ng;
  */
}

template<typename BV>
EnvironmentModel* EnvironmentModelBVH<BV>::clone(void) const
{
  return NULL;
  /*
  EnvironmentModelBVH *env = new EnvironmentModelBVH();
  env->default_collision_matrix_ = default_collision_matrix_;
  env->default_link_padding_map_ = default_link_padding_map_;
  env->verbose_ = verbose_;
  env->robot_scale_ = robot_scale_;
  env->default_robot_padding_ = default_robot_padding_;
  env->robot_model_ = new planning_models::KinematicModel(*robot_model_);
  env->createODERobotModel();

  for (std::map<std::string, CollisionNamespace*>::const_iterator it = coll_namespaces_.begin() ; it != coll_namespaces_.end() ; ++it) {
    // construct a map of the shape pointers we have; this points to the index positions where they are stored;
    std::map<void*, int> shapePtrs;
    const EnvironmentObjects::NamespaceObjects &ns = objects_->getObjects(it->first);
    unsigned int n = ns.static_shape.size();
    for (unsigned int i = 0 ; i < n ; ++i)
      shapePtrs[ns.static_shape[i]] = -1 - i;
    n = ns.shape.size();
    for (unsigned int i = 0 ; i < n ; ++i)
      shapePtrs[ns.shape[i]] = i;
    
    // copy the collision namespace structure, geom by geom
    CollisionNamespace *cn = new CollisionNamespace(it->first);
    env->coll_namespaces_[it->first] = cn;
    n = it->second->geoms.size();
    cn->geoms.reserve(n);
    for (unsigned int i = 0 ; i < n ; ++i)
    {
      dGeomID newGeom = copyGeom(cn->space, cn->storage, it->second->geoms[i], it->second->storage);
      int idx = shapePtrs[dGeomGetData(it->second->geoms[i])];
      if (idx < 0) // static geom
      {
        shapes::StaticShape *newShape = shapes::cloneShape(ns.static_shape[-idx - 1]);
        dGeomSetData(newGeom, reinterpret_cast<void*>(newShape));
        env->objects_->addObject(it->first, newShape);
      }
      else // movable geom
      {
        shapes::Shape *newShape = shapes::cloneShape(ns.shape[idx]);
        dGeomSetData(newGeom, reinterpret_cast<void*>(newShape));
        env->objects_->addObject(it->first, newShape, ns.shape_pose[idx]);
      }
      cn->geoms.push_back(newGeom);
    }
    std::vector<dGeomID> geoms;
    it->second->collide2.getGeoms(geoms);
    n = geoms.size();
    for (unsigned int i = 0 ; i < n ; ++i)
    {
      dGeomID newGeom = copyGeom(cn->space, cn->storage, geoms[i], it->second->storage);
      int idx = shapePtrs[dGeomGetData(geoms[i])];
      if (idx < 0) // static geom
      {
        shapes::StaticShape *newShape = shapes::cloneShape(ns.static_shape[-idx - 1]);
        dGeomSetData(newGeom, reinterpret_cast<void*>(newShape));
        env->objects_->addObject(it->first, newShape);
      }
      else // movable geom
      {
        shapes::Shape *newShape = shapes::cloneShape(ns.shape[idx]);
        dGeomSetData(newGeom, reinterpret_cast<void*>(newShape));
        env->objects_->addObject(it->first, newShape, ns.shape_pose[idx]);
      }
      cn->collide2.registerGeom(newGeom);
    }
  }
    
  return env;    
  */

}


template class EnvironmentModelBVH<KDOP<16> >;
template class EnvironmentModelBVH<KDOP<18> >;
template class EnvironmentModelBVH<KDOP<24> >;
template class EnvironmentModelBVH<OBB>;
template class EnvironmentModelBVH<AABB>;


}
