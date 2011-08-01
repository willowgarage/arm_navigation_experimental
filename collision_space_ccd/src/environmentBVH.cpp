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
bool EnvironmentModelBVH<BV>::hasObject(const std::string& ns) const
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
}

template<typename BV>
EnvironmentModel* EnvironmentModelBVH<BV>::clone(void) const
{
  // TODO
  return NULL;
}


template<typename BV>
void EnvironmentModelBVH<BV>::SAPManager::unregisterGeom(CollisionGeom* geom)
{
  setup();
  CollisionGeom* found = NULL;
  std::vector<CollisionGeom*>::iterator pos_start1 = std::lower_bound(geoms_x.begin(), geoms_x.end(), geom, SortByXTest());
  std::vector<CollisionGeom*>::iterator pos_end1 = std::upper_bound(pos_start1, geoms_x.end(), geom, SortByXTest());
  while(pos_start1 < pos_end1)
  {
    if(*pos_start1 == geom)
    {
      found = *pos_start1;
      geoms_x.erase(pos_start1);
      break;
    }
    ++pos_start1;
  }

  std::vector<CollisionGeom*>::iterator pos_start2 = std::lower_bound(geoms_y.begin(), geoms_y.end(), geom, SortByYTest());
  std::vector<CollisionGeom*>::iterator pos_end2 = std::upper_bound(pos_start2, geoms_y.end(), geom, SortByYTest());
  while(pos_start2 < pos_end2)
  {
    if(*pos_start2 == geom)
    {
      assert(found == *pos_start2);
      geoms_y.erase(pos_start2);
      break;
    }
    ++pos_start2;
  }

  std::vector<CollisionGeom*>::iterator pos_start3 = std::lower_bound(geoms_z.begin(), geoms_z.end(), geom, SortByZTest());
  std::vector<CollisionGeom*>::iterator pos_end3 = std::upper_bound(pos_start3, geoms_z.end(), geom, SortByZTest());
  while(pos_start3 < pos_end3)
  {
    if(*pos_start3 == geom)
    {
      assert(found == *pos_start3);
      geoms_z.erase(pos_start3);
      break;
    }
    ++pos_start3;
  }
}




template<typename BV>
void EnvironmentModelBVH<BV>::SAPManager::registerGeom(CollisionGeom* geom)
{
  geoms_x.push_back(geom);
  geoms_y.push_back(geom);
  geoms_z.push_back(geom);
  setup_ = false;
}

template<typename BV>
void EnvironmentModelBVH<BV>::SAPManager::setup()
{
  if(!setup_)
  {
    std::sort(geoms_x.begin(), geoms_x.end(), SortByXLow());
    std::sort(geoms_y.begin(), geoms_y.end(), SortByYLow());
    std::sort(geoms_z.begin(), geoms_z.end(), SortByZLow());
    setup_ = true;
  }
}

template<typename BV>
void EnvironmentModelBVH<BV>::SAPManager::update()
{
  setup();
}

template<typename BV>
void EnvironmentModelBVH<BV>::SAPManager::clear()
{
  geoms_x.clear();
  geoms_y.clear();
  geoms_z.clear();
  setup_ = false;
}

template<typename BV>
void EnvironmentModelBVH<BV>::SAPManager::getGeoms(std::vector<CollisionGeom*>& geoms) const
{
  geoms.resize(geoms_x.size());
  for(unsigned int i = 0; i < geoms.size(); ++i)
    geoms[i] = geoms_x[i];
}

template<typename BV>
void EnvironmentModelBVH<BV>::SAPManager::collide(CollisionGeom* geom, CollisionData* cdata) const
{
  static const unsigned int CUTOFF = 100;

  assert(setup_);

  std::vector<CollisionGeom*>::const_iterator pos_start1 = std::lower_bound(geoms_x.begin(), geoms_x.end(), geom, SortByXTest());
  if(pos_start1 != geoms_x.end())
  {
    std::vector<CollisionGeom*>::const_iterator pos_end1 = std::upper_bound(pos_start1, geoms_x.end(), geom, SortByXTest());
    unsigned int d1 = pos_end1 - pos_start1;

    if(d1 > CUTOFF)
    {
      std::vector<CollisionGeom*>::const_iterator pos_start2 = std::lower_bound(geoms_y.begin(), geoms_y.end(), geom, SortByYTest());
      if(pos_start2 != geoms_y.end())
      {
        std::vector<CollisionGeom*>::const_iterator pos_end2 = std::upper_bound(pos_start2, geoms_y.end(), geom, SortByYTest());
        unsigned int d2 = pos_end2 - pos_start2;

        if(d2 > CUTOFF)
        {
          std::vector<CollisionGeom*>::const_iterator pos_start3 = std::lower_bound(geoms_z.begin(), geoms_z.end(), geom, SortByZTest());
          if(pos_start3 != geoms_z.end())
          {
            std::vector<CollisionGeom*>::const_iterator pos_end3 = std::upper_bound(pos_start3, geoms_z.end(), geom, SortByZTest());
            unsigned int d3 = pos_end3 - pos_start3;

            if(d3 > CUTOFF)
            {
              if(d3 <= d2 && d3 <= d1)
                checkColl(pos_start3, pos_end3, geom, cdata);
              else
              {
                if(d2 <= d3 && d2 <= d1)
                  checkColl(pos_start2, pos_end2, geom, cdata);
                else
                  checkColl(pos_start1, pos_end1, geom, cdata);
              }
            }
            else
              checkColl(pos_start3, pos_end3, geom, cdata);
          }
        }
        else
          checkColl(pos_start2, pos_end2, geom, cdata);
      }
    }
    else
      checkColl(pos_start1, pos_end1, geom, cdata);
  }
}


template<typename BV>
void EnvironmentModelBVH<BV>::SAPManager::checkColl(std::vector<CollisionGeom*>::const_iterator pos_start, std::vector<CollisionGeom*>::const_iterator pos_end,
                                                    CollisionGeom* geom, CollisionData* cdata) const
{
  while(pos_start < pos_end)
  {
    if((*pos_start)->aabb.overlap(geom->aabb))
    {
      EnvironmentModelBVH<BV>::testGeomCollision(cdata, *pos_start, geom);
      if (cdata->done && !cdata->exhaustive)
        return;
    }
    pos_start++;
  }
}

template<typename BV>
void EnvironmentModelBVH<BV>::SAPManager::collide(CollisionData* cdata) const
{
  // simple sweep and prune

  // choose the best axis
  double delta_x = (geoms_x[geoms_x.size() - 1])->aabb.min_[0] - (geoms_x[0])->aabb.min_[0];
  double delta_y = (geoms_x[geoms_y.size() - 1])->aabb.min_[1] - (geoms_y[0])->aabb.min_[1];
  double delta_z = (geoms_z[geoms_z.size() - 1])->aabb.min_[2] - (geoms_z[0])->aabb.min_[2];

  int axis = 0;
  if(delta_y > delta_x && delta_y > delta_z)
    axis = 1;
  else if(delta_z > delta_y && delta_z > delta_x)
    axis = 2;

  int axis2 = (axis + 1 > 2) ? 0 : (axis + 1);
  int axis3 = (axis2 + 1 > 2) ? 0 : (axis2 + 1);

  std::vector<CollisionGeom*>::const_iterator pos, run_pos, pos_end;

  switch(axis)
  {
    case 0:
      pos = geoms_x.begin();
      pos_end = geoms_x.end();
      break;
    case 1:
      pos = geoms_y.begin();
      pos_end = geoms_y.end();
      break;
    case 2:
      pos = geoms_z.begin();
      pos_end = geoms_z.end();
      break;
  }
  run_pos = pos;

  while((run_pos != pos_end) && (pos != pos_end))
  {
    CollisionGeom* geom = *(pos++);

    while(1)
    {
      if((*run_pos)->aabb.min_[axis] < geom->aabb.min_[axis])
      {
        run_pos++;
        if(run_pos == pos_end) break;
        continue;
      }
      else
      {
        run_pos++;
        break;
      }
    }

    if(run_pos != pos_end)
    {
      std::vector<CollisionGeom*>::const_iterator run_pos2 = run_pos;

      while((*run_pos2)->aabb.min_[axis] <= geom->aabb.max_[axis])
      {
        CollisionGeom* geom2 = *run_pos2;
        run_pos2++;

        if((geom->aabb.max_[axis2] >= geom2->aabb.min_[axis2]) && (geom2->aabb.max_[axis2] >= geom->aabb.min_[axis2]))
        {
          if((geom->aabb.max_[axis3] >= geom2->aabb.min_[axis3]) && (geom2->aabb.max_[axis3] >= geom->aabb.min_[axis3]))
          {
            EnvironmentModelBVH<BV>::testGeomCollision(cdata, geom, geom2);
            if (cdata->done && !cdata->exhaustive)
              return;
          }
        }

        if(run_pos2 == pos_end) break;
      }
    }
  }
}

template<typename BV>
bool EnvironmentModelBVH<BV>::SAPManager::empty() const
{
  return geoms_x.empty();
}


template class EnvironmentModelBVH<KDOP<16> >;
template class EnvironmentModelBVH<KDOP<18> >;
template class EnvironmentModelBVH<KDOP<24> >;
template class EnvironmentModelBVH<OBB>;
template class EnvironmentModelBVH<AABB>;


}
