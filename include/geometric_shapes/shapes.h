/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/

/*  Author: Ioan Sucan */

#ifndef GEOMETRIC_SHAPES_SHAPES_
#define GEOMETRIC_SHAPES_SHAPES_

#include <cstdlib>
#include <vector>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <string>
#include <Eigen/Dense>
#include <octomap/octomap.h>

namespace octomap
{
class OcTree;
}

/** \brief Definition of various shapes. No properties such as
    position are included. These are simply the descriptions and
    dimensions of shapes. */
namespace shapes
{

/** \brief A list of known shape types */
enum ShapeType { UNKNOWN_SHAPE, SPHERE, CYLINDER, CONE, BOX, PLANE, MESH, OCTREE, OCCMAP };


/** \brief A basic definition of a shape. Shapes are considered centered at origin */
class Shape
{
public:
  Shape();
  virtual ~Shape();

  /** \brief Create a copy of this shape */
  virtual Shape* clone() const = 0;

  /** \brief Print information about this shape */
  virtual void print(std::ostream &out = std::cout) const;

  /** \brief Scale this shape by a factor */
  void scale(double scale);

  /** \brief Add padding to this shape */
  void padd(double padding);

  /** \brief Scale and padd this shape */
  virtual void scaleAndPadd(double scale, double padd) = 0;

  /** \brief Return a flag indicating whether this shape can be scaled and/or padded */
  virtual bool isFixed() const;

  /** \brief The type of the shape */
  ShapeType type;
};

/** \brief Definition of a sphere */
class Sphere : public Shape
{
public:
  Sphere();

  /** \brief The radius of the shpere */
  Sphere(double r);

  /** \brief The type of the shape, as a string */
  static const std::string STRING_NAME;

  virtual void scaleAndPadd(double scale, double padd);
  virtual Shape* clone() const;
  virtual void print(std::ostream &out = std::cout) const;

  /** \brief The radius of the sphere */
  double radius;
};

/** \brief Definition of a cylinder
 * Length is along z axis.  Origin is at center of mass. */
class Cylinder : public Shape
{
public:
  Cylinder();

  /** \brief The radius and the length of the cylinder */
  Cylinder(double r, double l);

  /** \brief The type of the shape, as a string */
  static const std::string STRING_NAME;

  virtual void scaleAndPadd(double scale, double padd);
  virtual Shape* clone() const;
  virtual void print(std::ostream &out = std::cout) const;

  /** \brief The length of the cylinder */
  double length;

  /** \brief The radius of the cylinder */
  double radius;
};

/** \brief Definition of a cone
 * Tip is on positive z axis.  Center of base is on negative z axis.  Origin is
 * halway between tip and center of base. */
class Cone : public Shape
{
public:
  Cone();
  Cone(double r, double l);

  /** \brief The type of the shape, as a string */
  static const std::string STRING_NAME;

  virtual void scaleAndPadd(double scale, double padd);
  virtual Shape* clone() const;
  virtual void print(std::ostream &out = std::cout) const;

  /** \brief The length (height) of the cone */
  double length;

  /** \brief The radius of the cone */
  double radius;
};

/** \brief Definition of a box
 * Aligned with the XYZ axes. */
class Box : public Shape
{
public:
  Box();
  Box(double x, double y, double z);

  /** \brief The type of the shape, as a string */
  static const std::string STRING_NAME;

  virtual void scaleAndPadd(double scale, double padd);
  virtual Shape* clone() const;
  virtual void print(std::ostream &out = std::cout) const;

  /** \brief x, y, z dimensions of the box (axis-aligned) */
  double size[3];
};

/** \brief Definition of a triangle mesh
 * By convention the "center" of the shape is at the origin.  For a mesh this
 * implies that the AABB of the mesh is centered at the origin.  Some methods
 * may not work with arbitrary meshes whose AABB is not centered at the origin.
 * */
class Mesh : public Shape
{
public:

  Mesh();
  Mesh(unsigned int v_count, unsigned int t_count);
  virtual ~Mesh();

  /** \brief The type of the shape, as a string */
  static const std::string STRING_NAME;

  virtual void scaleAndPadd(double scale, double padd);
  virtual Shape* clone() const;
  virtual void print(std::ostream &out = std::cout) const;

  /** \brief The normals to each triangle can be computed from the vertices using cross products. This function performs this computation and allocates memory for normals if needed */
  void computeTriangleNormals();

  /** \brief The normals to each vertex, averaged from the triangle normals. computeTriangleNormals() is automatically called if needed. */
  void computeVertexNormals();

  /** \brief Merge vertices that are very close to each other, up to a threshold*/
  void mergeVertices(double threshold);

  /** \brief The number of available vertices */
  unsigned int  vertex_count;

  /** \brief The position for each vertex vertex k has values at
   * index (3k, 3k+1, 3k+2) = (x,y,z) */
  double       *vertices;

  /** \brief The number of triangles formed with the vertices */
  unsigned int  triangle_count;

  /** \brief The vertex indices for each triangle
   * triangle k has vertices at index (3k, 3k+1, 3k+2) = (v1, v2, v3) */
  unsigned int *triangles;

  /** \brief The normal to each triangle; unit vector represented
      as (x,y,z); If missing from the mesh, these vectors can be computed using computeTriangleNormals() */
  double       *triangle_normals;

  /** \brief The normal to each vertex; unit vector represented
      as (x,y,z); If missing from the mesh, these vectors can be computed using computeVertexNormals()  */
  double       *vertex_normals;
};

/** \brief Definition of a plane with equation ax + by + cz + d = 0 */
class Plane : public Shape
{
public:

  Plane();
  Plane(double pa, double pb, double pc, double pd);

  /** \brief The type of the shape, as a string */
  static const std::string STRING_NAME;

  virtual Shape* clone() const;
  virtual void print(std::ostream &out = std::cout) const;
  virtual void scaleAndPadd(double scale, double padd);
  virtual bool isFixed() const;

  /** \brief The plane equation is ax + by + cz + d = 0 */
  double a, b, c, d;
};

/** \brief Representation of an octomap::OcTree as a Shape */
class OcTree : public Shape
{
public:
  OcTree();
  OcTree(const boost::shared_ptr<const octomap::OcTree> &t);

  /** \brief The type of the shape, as a string */
  static const std::string STRING_NAME;

  virtual Shape* clone() const;
  virtual void print(std::ostream &out = std::cout) const;
  virtual void scaleAndPadd(double scale, double padd);
  virtual bool isFixed() const;

  boost::shared_ptr<const octomap::OcTree> octree;
};

class Voxel
{
public:
  virtual bool   isOccupied() const = 0;
  virtual double getOccupancy() const = 0;
  virtual double getSize() const = 0;
  virtual double getX() const = 0;
  virtual double getY() const = 0;
  virtual double getZ() const = 0;
  Eigen::Vector3d getCoordinate() const {return Eigen::Vector3d(getX(), getY(), getZ());}
};

class occmap_iterator : public std::iterator<std::forward_iterator_tag, Voxel>
{
public:
  /// Default Constructor, only used for the end-iterator
  occmap_iterator() : vox(NULL){}
  occmap_iterator(const occmap_iterator &other) : vox(other.vox){}
  ~occmap_iterator() {
    delete vox;
  }

  virtual bool operator==(const occmap_iterator& other) const = 0;
  virtual bool operator!=(const occmap_iterator& other) const = 0;
  virtual occmap_iterator& operator=(const occmap_iterator& other) = 0;
  virtual occmap_iterator& operator++() = 0;
  void operator++(int){
    ++*this;
  }
  Voxel const* operator->() const {return vox;}
  Voxel* operator->() {return vox;}
  const Voxel& operator*() const {return *vox;}
  Voxel& operator*() {return *vox;}

protected:
  Voxel* vox;
};

class OccMap : public Shape
{
public:
  OccMap();

  /** \brief The type of the shape, as a string */
  static const std::string STRING_NAME;

  virtual Shape* clone() const = 0;
  virtual OccMap* cloneMap() const = 0;
  virtual void print(std::ostream &out = std::cout) const = 0;
  virtual void scaleAndPadd(double scale, double padd);
  virtual bool isFixed() const;

  virtual boost::shared_ptr<occmap_iterator> begin() = 0;
  virtual boost::shared_ptr<occmap_iterator> end() = 0;
  virtual boost::shared_ptr<occmap_iterator> begin_bbx(const Eigen::Vector3d &min, const Eigen::Vector3d &max) = 0;
  virtual boost::shared_ptr<occmap_iterator> end_bbx() = 0;
};

/** \brief Shared pointer to a OcTree */
typedef boost::shared_ptr<const octomap::OcTree> OctTreePtr;

class OctomapVoxel : public Voxel
{
public:
  OctomapVoxel(octomap::OcTreeNode n, octomap::OcTreeKey k, unsigned d, const OctTreePtr &tree) : key(k), node(n), depth(d), octree(tree){}
  OctomapVoxel(const OctomapVoxel &other) : key(other.key), node(other.node), depth(other.depth), octree(other.octree){}

  virtual bool   isOccupied() const {return octree->isNodeOccupied(node);}
  virtual double getOccupancy() const {return node.getOccupancy();}
  virtual double getSize() const {return octree->getNodeSize(depth);}
  virtual double getX() const {return octree->keyToCoord(key[0], depth);}
  virtual double getY() const {return octree->keyToCoord(key[1], depth);}
  virtual double getZ() const {return octree->keyToCoord(key[2], depth);}

  octomap::OcTreeKey getKey() {return key;}
  octomap::OcTreeNode getNode() {return node;}
  unsigned getDepth() {return depth;}

protected:
  octomap::OcTreeKey key;
  octomap::OcTreeNode node;
  unsigned char depth;
  OctTreePtr octree;
};

template<class ITYPE>
class occmap_iterator_octomap : public occmap_iterator
{
public:
  /// Constructor of the iterator, takes another iterator and wraps it
  occmap_iterator_octomap(ITYPE i, const OctTreePtr &tree) : it(i), octree(tree){
    vox = new OctomapVoxel(*it, it.getKey(), it.getDepth(), octree);
  }
  /// Copy constructor
  occmap_iterator_octomap(const occmap_iterator_octomap &other) : occmap_iterator(other), it(other.it), octree(other.octree){}

  virtual bool operator==(const occmap_iterator& other) const {
    const occmap_iterator_octomap *o = dynamic_cast<const occmap_iterator_octomap*>(&other);
    if (o == NULL) return false;
    return it==o->it;
  }
  virtual bool operator!=(const occmap_iterator& other) const {
    const occmap_iterator_octomap *o = dynamic_cast<const occmap_iterator_octomap*>(&other);
    if (o == NULL) return true;
    return it!=o->it;
  }
  virtual occmap_iterator& operator=(const occmap_iterator& other){
    const occmap_iterator_octomap *o = dynamic_cast<const occmap_iterator_octomap*>(&other);
    //if (o == NULL) return *this;
    it = o->it;
    octree = o->octree;
    vox = o->vox;
    return *this;
  }
  virtual occmap_iterator& operator++() {
    ++it;
    delete vox;
    vox = new OctomapVoxel(*it, it.getKey(), it.getDepth(), octree);
    return *this;
  }
  occmap_iterator_octomap& operator=(const occmap_iterator_octomap& other){
    it = other.it;
    octree = other.octree;
    vox = other.vox;
    return *this;
  }

protected:
  OctTreePtr octree;
  ITYPE it;
};

class OctomapOccMap : public OccMap
{
public:
  OctomapOccMap() {}
  OctomapOccMap(const OctTreePtr &tree) : octree(tree){}

  virtual Shape* clone() const { return new OctomapOccMap(octree); };
  virtual OccMap* cloneMap() const { return new OctomapOccMap(octree); };
  virtual void print(std::ostream &out = std::cout) const {};

  virtual boost::shared_ptr<occmap_iterator> begin() {
    return boost::shared_ptr<occmap_iterator>(new occmap_iterator_octomap<octomap::OcTree::leaf_iterator>(octree->begin_leafs(), octree));
  }
  virtual boost::shared_ptr<occmap_iterator> end() {
    return boost::shared_ptr<occmap_iterator>(new occmap_iterator_octomap<octomap::OcTree::leaf_iterator>(octree->end_leafs(), octree));
  }
  virtual boost::shared_ptr<occmap_iterator> begin_bbx(const Eigen::Vector3d &min, const Eigen::Vector3d &max) {
    octomap::point3d omin = octomap::point3d(min(0),min(1),min(2));
    octomap::point3d omax = octomap::point3d(max(0),max(1),max(2));
    return boost::shared_ptr<occmap_iterator>(new occmap_iterator_octomap<octomap::OcTree::leaf_bbx_iterator>(octree->begin_leafs_bbx(omin, omax), octree));
  }
  virtual boost::shared_ptr<occmap_iterator> end_bbx() {
    return boost::shared_ptr<occmap_iterator>(new occmap_iterator_octomap<octomap::OcTree::leaf_bbx_iterator>(octree->end_leafs_bbx(), octree));
  }

  OctTreePtr octree;
};

/** \brief Shared pointer to a Shape */
typedef boost::shared_ptr<Shape> ShapePtr;

/** \brief Shared pointer to a const Shape */
typedef boost::shared_ptr<const Shape> ShapeConstPtr;

}

#endif
