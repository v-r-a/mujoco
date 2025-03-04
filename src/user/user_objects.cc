// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "user/user_objects.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "lodepng.h"
#include <mujoco/mjmacro.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjplugin.h>
#include <mujoco/mjtnum.h>
#include "cc/array_safety.h"
#include "engine/engine_resource.h"
#include "engine/engine_io.h"
#include "engine/engine_passive.h"
#include "engine/engine_plugin.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_solve.h"
#include "engine/engine_util_spatial.h"
#include "engine/engine_vfs.h"
#include "user/user_api.h"
#include "user/user_model.h"
#include "user/user_util.h"

namespace {
namespace mju = ::mujoco::util;
using std::string;
using std::vector;
}  // namespace

// utiility function for checking size parameters
static void checksize(double* size, mjtGeom type, mjCBase* object, const char* name, int id) {
  // plane: handle infinite
  if (type==mjGEOM_PLANE) {
    if (size[2]<=0) {
      throw mjCError(object, "plane size(3) must be positive in object '%s' (id = %d)", name, id);
    }
  }

  // regular geom
  else {
    for (int i=0; i<mjGEOMINFO[type]; i++) {
      if (size[i]<=0) {
        throw mjCError(object, "sizes must be positive in object '%s' (id = %d)", name, id);
      }
    }
  }
}

// error message for missing "limited" attribute
static void checklimited(
    const mjCBase* obj,
    bool autolimits, const char* entity, const char* attr, int limited, bool hasrange) {
  if (!autolimits && limited == 2 && hasrange) {
    std::stringstream ss;
    ss << entity << " has `" << attr << "range` but not `" << attr << "limited`. "
       << "set the autolimits=\"true\" compiler option, specify `" << attr << "limited` "
       << "explicitly (\"true\" or \"false\"), or remove the `" << attr << "range` attribute.";
    throw mjCError(obj, "%s", ss.str().c_str());
  }
}

// returns true if limits should be active
static bool islimited(int limited, const double range[2]) {
  if (limited == mjLIMITED_TRUE || (limited == mjLIMITED_AUTO && range[0] < range[1])) {
    return true;
  }
  return false;
}


//------------------------- class mjCError implementation ------------------------------------------

// constructor
mjCError::mjCError(const mjCBase* obj, const char* msg, const char* str, int pos1, int pos2) {
  char temp[600];

  // init
  warning = false;
  if (obj || msg) {
    mju::sprintf_arr(message, "Error");
  } else {
    message[0] = 0;
  }

  // construct error message
  if (msg) {
    if (str) {
      mju::sprintf_arr(temp, msg, str, pos1, pos2);
    } else {
      mju::sprintf_arr(temp, msg, pos1, pos2);
    }

    mju::strcat_arr(message, ": ");
    mju::strcat_arr(message, temp);
  }

  // append info from mjCBase object
  if (obj) {
    // with or without xml position
    if (!obj->info.empty()) {
      mju::sprintf_arr(temp, "Object name = %s, id = %d, %s",
                       obj->name.c_str(), obj->id, obj->info.c_str());
    } else {
      mju::sprintf_arr(temp, "Object name = %s, id = %d", obj->name.c_str(), obj->id);
    }

    // append to message
    mju::strcat_arr(message, "\n");
    mju::strcat_arr(message, temp);
  }
}



//------------------ class mjCAlternative implementation -------------------------------------------

// constructor
mjCAlternative::mjCAlternative() {
  axisangle[0] = xyaxes[0] = zaxis[0] = euler[0] = mjNAN;
}


// compute frame orientation given alternative specifications
// used for geom, site, body and camera frames
const char* mjCAlternative::Set(double* quat, bool degree, const char* sequence) {
  // set quat using axisangle
  if (mjuu_defined(axisangle[0])) {
    // convert to radians if necessary, normalize axis
    if (degree) {
      axisangle[3] = axisangle[3] / 180.0 * mjPI;
    }
    if (mjuu_normvec(axisangle, 3)<mjEPS) {
      return "axisangle too small";
    }

    // construct quaternion
    double ang2 = axisangle[3]/2;
    quat[0] = cos(ang2);
    quat[1] = sin(ang2)*axisangle[0];
    quat[2] = sin(ang2)*axisangle[1];
    quat[3] = sin(ang2)*axisangle[2];
  }

  // set quat using xyaxes
  if (mjuu_defined(xyaxes[0])) {
    // normalize x axis
    if (mjuu_normvec(xyaxes, 3)<mjEPS) {
      return "xaxis too small";
    }

    // make y axis orthogonal to x axis, normalize
    double d = mjuu_dot3(xyaxes, xyaxes+3);
    xyaxes[3] -= xyaxes[0]*d;
    xyaxes[4] -= xyaxes[1]*d;
    xyaxes[5] -= xyaxes[2]*d;
    if (mjuu_normvec(xyaxes+3, 3)<mjEPS) {
      return "yaxis too small";
    }

    // compute and normalize z axis
    double z[3];
    mjuu_crossvec(z, xyaxes, xyaxes+3);
    if (mjuu_normvec(z, 3)<mjEPS) {
      return "cross(xaxis, yaxis) too small";
    }

    // convert frame into quaternion
    mjuu_frame2quat(quat, xyaxes, xyaxes+3, z);
  }

  // set quat using zaxis
  if (mjuu_defined(zaxis[0])) {
    if (mjuu_normvec(zaxis, 3)<mjEPS) {
      return "zaxis too small";
    }
    mjuu_z2quat(quat, zaxis);
  }


  // handle euler
  if (mjuu_defined(euler[0])) {
    // convert to radians if necessary
    if (degree) {
      for (int i=0; i<3; i++) {
        euler[i] = euler[i] / 180.0 * mjPI;
      }
    }

    // init
    mjuu_setvec(quat, 1, 0, 0, 0);

    // loop over euler angles, accumulate rotations
    for (int i=0; i<3; i++) {
      double tmp[4], qrot[4] = {cos(euler[i]/2), 0, 0, 0};
      double sa = sin(euler[i]/2);

      // construct quaternion rotation
      if (sequence[i]=='x' || sequence[i]=='X') {
        qrot[1] = sa;
      } else if (sequence[i]=='y' || sequence[i]=='Y') {
        qrot[2] = sa;
      } else if (sequence[i]=='z' || sequence[i]=='Z') {
        qrot[3] = sa;
      } else {
        return "euler sequence can only contain x, y, z, X, Y, Z";
      }

      // accumulate rotation
      if (sequence[i]=='x' || sequence[i]=='y' || sequence[i]=='z') {
        mjuu_mulquat(tmp, quat, qrot);  // moving axes: post-multiply
      } else {
        mjuu_mulquat(tmp, qrot, quat);  // fixed axes: pre-multiply
      }
      mjuu_copyvec(quat, tmp, 4);
    }

    // normalize, just in case
    mjuu_normvec(quat, 4);
  }

  return 0;
}



//------------------------- class mjCBoundingVolumeHierarchy implementation ------------------------

// constructor
mjCBoundingVolumeHierarchy::mjCBoundingVolumeHierarchy() {
  nbvh = 0;
  mjuu_setvec(ipos_, 0, 0, 0);
  mjuu_setvec(iquat_, 1, 0, 0, 0);
}


// assign position and orientation
void mjCBoundingVolumeHierarchy::Set(mjtNum ipos_element[3], mjtNum iquat_element[4]) {
  mjuu_copyvec(ipos_, ipos_element, 3);
  mjuu_copyvec(iquat_, iquat_element, 4);
}



void mjCBoundingVolumeHierarchy::AllocateBoundingVolumes(int nleaf) {
  nbvh = 0;
  child.clear();
  nodeid.clear();
  level.clear();
  bvleaf_.clear();
  bvleaf_.resize(nleaf);
}


void mjCBoundingVolumeHierarchy::RemoveInactiveVolumes(int nmax) {
  bvleaf_.erase(bvleaf_.begin() + nmax, bvleaf_.end());
}


mjCBoundingVolume* mjCBoundingVolumeHierarchy::GetBoundingVolume(int id) {
  return bvleaf_.data() + id;
}


// create bounding volume hierarchy
void mjCBoundingVolumeHierarchy::CreateBVH() {
  // precompute the positions of each element in the hierarchy's axes, and drop
  // visual-only elements.
  std::vector<BVElement> elements;
  elements.reserve(bvleaf_.size());
  mjtNum qinv[4] = {iquat_[0], -iquat_[1], -iquat_[2], -iquat_[3]};
  for (int i = 0; i < bvleaf_.size(); i++) {
    if (bvleaf_[i].conaffinity || bvleaf_[i].contype) {
      BVElement element;
      element.e = &bvleaf_[i];
      element.index = i;
      mjtNum vert[3] = {element.e->pos[0] - ipos_[0],
                        element.e->pos[1] - ipos_[1],
                        element.e->pos[2] - ipos_[2]};
      mju_rotVecQuat(element.lpos, vert, qinv);
      elements.push_back(std::move(element));
    }
  }
  MakeBVH(elements.begin(), elements.end());
}

// compute bounding volume hierarchy
int mjCBoundingVolumeHierarchy::MakeBVH(
    std::vector<BVElement>::iterator elements_begin,
    std::vector<BVElement>::iterator elements_end, int lev) {
  int nelements = elements_end - elements_begin;
  if (nelements == 0) {
    return -1;
  }
  mjtNum AAMM[6] = {mjMAXVAL, mjMAXVAL, mjMAXVAL, -mjMAXVAL, -mjMAXVAL, -mjMAXVAL};

  // inverse transformation
  mjtNum qinv[4] = {iquat_[0], -iquat_[1], -iquat_[2], -iquat_[3]};

  // accumulate AAMM over elements
  for (auto element = elements_begin; element != elements_end; ++element) {
    // transform element aabb to aamm format
    mjtNum aamm[6] = {element->e->aabb[0] - element->e->aabb[3],
                      element->e->aabb[1] - element->e->aabb[4],
                      element->e->aabb[2] - element->e->aabb[5],
                      element->e->aabb[0] + element->e->aabb[3],
                      element->e->aabb[1] + element->e->aabb[4],
                      element->e->aabb[2] + element->e->aabb[5]};

    // update node AAMM
    for (int v=0; v<8; v++) {
      mjtNum vert[3], box[3];
      vert[0] = (v&1 ? aamm[3] : aamm[0]);
      vert[1] = (v&2 ? aamm[4] : aamm[1]);
      vert[2] = (v&4 ? aamm[5] : aamm[2]);

      // rotate to the body inertial frame if specified
      if (element->e->quat) {
        mju_rotVecQuat(box, vert, element->e->quat);
        box[0] += element->e->pos[0] - ipos_[0];
        box[1] += element->e->pos[1] - ipos_[1];
        box[2] += element->e->pos[2] - ipos_[2];
        mju_rotVecQuat(vert, box, qinv);
      }

      AAMM[0] = mjMIN(AAMM[0], vert[0]);
      AAMM[1] = mjMIN(AAMM[1], vert[1]);
      AAMM[2] = mjMIN(AAMM[2], vert[2]);
      AAMM[3] = mjMAX(AAMM[3], vert[0]);
      AAMM[4] = mjMAX(AAMM[4], vert[1]);
      AAMM[5] = mjMAX(AAMM[5], vert[2]);
    }
  }

  // inflate flat AABBs
  for (int i=0; i<3; i++) {
    if (mju_abs(AAMM[i]-AAMM[i+3])<mjEPS) {
      AAMM[i+0] -= mjEPS;
      AAMM[i+3] += mjEPS;
    }
  }

  // store current index
  int index = nbvh++;
  child.push_back(-1);
  child.push_back(-1);
  nodeid.push_back(nullptr);
  level.push_back(lev);

  // store bounding box of the current node
  for (int i=0; i<3; i++) {
    bvh.push_back((AAMM[3+i] + AAMM[i]) / 2);
  }
  for (int i=0; i<3; i++) {
    bvh.push_back((AAMM[3+i] - AAMM[i]) / 2);
  }

  // leaf node, return
  if (nelements==1) {
    for (int i=0; i<2; i++) {
      child[2*index+i] = -1;
    }
    nodeid[index] = (int*)elements_begin->e->GetId();
    return index;
  }

  // find longest axis for splitting the bounding box
  mjtNum edges[3] = { AAMM[3]-AAMM[0], AAMM[4]-AAMM[1], AAMM[5]-AAMM[2] };
  int axis = edges[0] > edges[1] ? 0 : 1;
  axis = edges[axis] > edges[2] ? axis : 2;

  // find median along the axis
  auto m = nelements/2;
  // Note: nth element performs a partial sort of elements
  BVElementCompare compare;
  compare.axis = axis;
  std::nth_element(elements_begin, elements_begin + m, elements_end, compare);

  // recursive calls
  if (m > 0) {
    child[2*index+0] = MakeBVH(elements_begin, elements_begin + m, lev+1);
  }

  if (m != nelements) {
    child[2*index+1] = MakeBVH(elements_begin + m, elements_end, lev+1);
  }

  // SHOULD NOT OCCUR
  if (child[2*index+0]==-1 && child[2*index+1]==-1) {
    mju_error("this should have been a leaf, body=%s nelements=%d",
              name_.c_str(), nelements);
  }

  if (lev>mjMAXTREEDEPTH) {
    mju_warning("max tree depth exceeded in body=%s", name_.c_str());
  }

  return index;
}

//------------------------- class mjCDef implementation --------------------------------------------

// constructor
mjCDef::mjCDef(void) {
  name.clear();
  parentid = -1;
  childid.clear();
  mjm_defaultJoint(joint.spec);
  mjm_defaultGeom(geom.spec);
  mjm_defaultSite(site.spec);
  mjm_defaultCamera(camera.spec);
  mjm_defaultLight(light.spec);
  mjm_defaultFlex(flex.spec);
  mjm_defaultMesh(mesh.spec);
  mjm_defaultMaterial(material.spec);
  mjm_defaultPair(pair.spec);
  mjm_defaultEquality(equality.spec);
  mjm_defaultTendon(tendon.spec);
  mjm_defaultActuator(actuator.spec);

  // make sure all the pointers are local
  PointToLocal();
}



// copy constructor
mjCDef::mjCDef(const mjCDef& other) {
  *this = other;
}



// compiler
void mjCDef::Compile(const mjCModel* model) {
  CopyFromSpec();

  // enforce length of all default userdata arrays
  joint.userdata_.resize(model->nuser_jnt);
  geom.userdata_.resize(model->nuser_geom);
  site.userdata_.resize(model->nuser_site);
  camera.userdata_.resize(model->nuser_cam);
  tendon.userdata_.resize(model->nuser_tendon);
  actuator.userdata_.resize(model->nuser_actuator);
}



// assignment operator
mjCDef& mjCDef::operator=(const mjCDef& other) {
  if (this != &other) {
    name = other.name;
    parentid = other.parentid;
    childid = other.childid;
    joint = other.joint;
    geom = other.geom;
    site = other.site;
    camera = other.camera;
    light = other.light;
    flex = other.flex;
    mesh = other.mesh;
    material = other.material;
    pair = other.pair;
    equality = other.equality;
    tendon = other.tendon;
    actuator = other.actuator;
  }
  PointToLocal();
  return *this;
}



void mjCDef::PointToLocal() {
  joint.PointToLocal();
  geom.PointToLocal();
  site.PointToLocal();
  camera.PointToLocal();
  light.PointToLocal();
  flex.PointToLocal();
  mesh.PointToLocal();
  material.PointToLocal();
  pair.PointToLocal();
  equality.PointToLocal();
  tendon.PointToLocal();
  actuator.PointToLocal();
  spec.element = (mjElement)this;
  spec.name = (mjString)&name;
  spec.joint = &joint.spec;
  spec.geom = &geom.spec;
  spec.site = &site.spec;
  spec.camera = &camera.spec;
  spec.light = &light.spec;
  spec.flex = &flex.spec;
  spec.mesh = &mesh.spec;
  spec.material = &material.spec;
  spec.pair = &pair.spec;
  spec.equality = &equality.spec;
  spec.tendon = &tendon.spec;
  spec.actuator = &actuator.spec;
}



void mjCDef::CopyFromSpec() {
  joint.CopyFromSpec();
  geom.CopyFromSpec();
  site.CopyFromSpec();
  camera.CopyFromSpec();
  light.CopyFromSpec();
  flex.CopyFromSpec();
  mesh.CopyFromSpec();
  material.CopyFromSpec();
  pair.CopyFromSpec();
  equality.CopyFromSpec();
  tendon.CopyFromSpec();
  actuator.CopyFromSpec();
}



//------------------------- class mjCBase implementation -------------------------------------------

// constructor
mjCBase::mjCBase() {
  name.clear();
  classname.clear();
  id = -1;
  info = "";
  model = 0;
  def = 0;
  frame = nullptr;
}



mjCBase::mjCBase(const mjCBase& other) {
  *this = other;
}



mjCBase& mjCBase::operator=(const mjCBase& other) {
  if (this != &other) {
    *static_cast<mjCBase_*>(this) = static_cast<const mjCBase_&>(other);
    if (other.def) {
      def = new mjCDef(*other.def);
    }
    if (other.frame) {
      frame = new mjCFrame(*other.frame);
    }
  }
  return *this;
}



// load resource if found (fallback to OS filesystem)
mjResource* mjCBase::LoadResource(string filename, const mjVFS* vfs) {
  // try reading from provided VFS
  mjResource* r = mju_openVfsResource(filename.c_str(), vfs);

  if (!r) {
    std::array<char, 1024> error;
    // not in vfs try a provider or fallback to OS filesystem
    r = mju_openResource(filename.c_str(), error.data(), error.size());
    if (!r) {
      throw mjCError(nullptr, "%s", error.data());
    }
  }
  return r;
}


// Get and sanitize content type from raw_text if not empty, otherwise parse
// content type from resource_name; throw error on failure
std::string mjCBase::GetAssetContentType(std::string_view resource_name,
                                         std::string_view raw_text) {
  if (!raw_text.empty()) {
    auto type = mjuu_parseContentTypeAttrType(raw_text);
    auto subtype = mjuu_parseContentTypeAttrSubtype(raw_text);
    if (!type.has_value() || !subtype.has_value()) {
      throw mjCError(this, "invalid format for content_type");
    }
    return std::string(*type) + "/" + std::string(*subtype);
  } else {
    return mjuu_extToContentType(resource_name);
  }
}


void mjCBase::SetFrame(mjCFrame* _frame) {
  if (!_frame) {
    return;
  }
  frame = _frame;
  frame->Compile();
}


//------------------ class mjCBody implementation --------------------------------------------------

// constructor
mjCBody::mjCBody(mjCModel* _model) {
  // set model pointer
  model = _model;

  mjm_defaultBody(spec);
  parentid = -1;
  weldid = -1;
  dofnum = 0;
  lastdof = -1;
  subtreedofs = 0;
  contype = 0;
  conaffinity = 0;
  margin = 0;
  mjuu_zerovec(xpos0, 3);
  mjuu_setvec(xquat0, 1, 0, 0, 0);

  // clear object lists
  bodies.clear();
  geoms.clear();
  frames.clear();
  joints.clear();
  sites.clear();
  cameras.clear();
  lights.clear();
  spec_userdata_.clear();

  // in case this body is not compiled
  CopyFromSpec();

  // point to local (needs to be after defaults)
  PointToLocal();
}



mjCBody::mjCBody(const mjCBody& other) {
  *this = other;
}



mjCBody& mjCBody::operator=(const mjCBody& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjCBody_*>(this) = static_cast<const mjCBody_&>(other);
    *static_cast<mjmBody*>(this) = static_cast<const mjmBody&>(other);
    this->bodies.clear();
    this->frames.clear();
    this->geoms.clear();
    this->joints.clear();
    this->sites.clear();
    this->cameras.clear();
    this->lights.clear();

    // copy all children
    for (int i=0; i<other.bodies.size(); i++) {
      this->bodies.push_back(new mjCBody(*other.bodies[i]));  // triggers recursive call
    }
    for (int i=0; i<other.frames.size(); i++) {
      this->frames.push_back(new mjCFrame(*other.frames[i]));
    }
    for (int i=0; i<other.geoms.size(); i++) {
      this->geoms.push_back(new mjCGeom(*other.geoms[i]));
      this->geoms.back()->body = this;
    }
    for (int i=0; i<other.joints.size(); i++) {
      this->joints.push_back(new mjCJoint(*other.joints[i]));
      this->joints.back()->body = this;
    }
    for (int i=0; i<other.sites.size(); i++) {
      this->sites.push_back(new mjCSite(*other.sites[i]));
      this->sites.back()->body = this;
    }
    for (int i=0; i<other.cameras.size(); i++) {
      this->cameras.push_back(new mjCCamera(*other.cameras[i]));
      this->cameras.back()->body = this;
    }
    for (int i=0; i<other.lights.size(); i++) {
      this->lights.push_back(new mjCLight(*other.lights[i]));
      this->lights.back()->body = this;
    }
  }
  PointToLocal();
  return *this;
}



void mjCBody::PointToLocal() {
  spec.element = (mjElement)this;
  spec.name = (mjString)&name;
  spec.childclass = (mjString)&classname;
  spec.userdata = (mjDoubleVec)&spec_userdata_;
  spec.plugin.name = (mjString)&plugin_name;
  spec.plugin.instance_name = (mjString)&plugin_instance_name;
  spec.info = (mjString)&info;
}


void mjCBody::CopyFromSpec() {
  *static_cast<mjmBody*>(this) = spec;
  userdata_ = spec_userdata_;
  userdata = (mjDoubleVec)&userdata_;
  mju_copy4(alt_.axisangle, alt.axisangle);
  mju_copy(alt_.xyaxes, alt.xyaxes, 6);
  mju_copy3(alt_.zaxis, alt.zaxis);
  mju_copy3(alt_.euler, alt.euler);
  mju_copy4(ialt_.axisangle, ialt.axisangle);
  mju_copy(ialt_.xyaxes, ialt.xyaxes, 6);
  mju_copy3(ialt_.zaxis, ialt.zaxis);
  mju_copy3(ialt_.euler, ialt.euler);
  plugin.active = spec.plugin.active;
  plugin.instance = spec.plugin.instance;
  plugin.name = spec.plugin.name;
  plugin.instance_name = spec.plugin.instance_name;
}



// destructor
mjCBody::~mjCBody() {
  // delete objects allocated here
  for (int i=0; i<bodies.size(); i++) delete bodies[i];
  for (int i=0; i<geoms.size(); i++) delete geoms[i];
  for (int i=0; i<frames.size(); i++) delete frames[i];
  for (int i=0; i<joints.size(); i++) delete joints[i];
  for (int i=0; i<sites.size(); i++) delete sites[i];
  for (int i=0; i<cameras.size(); i++) delete cameras[i];
  for (int i=0; i<lights.size(); i++) delete lights[i];

  bodies.clear();
  geoms.clear();
  frames.clear();
  joints.clear();
  sites.clear();
  cameras.clear();
  lights.clear();
}



// create child body and add it to body
mjCBody* mjCBody::AddBody(mjCDef* _def) {
  // create body
  mjCBody* obj = new mjCBody(model);

  // handle def recursion (i.e. childclass)
  obj->def = _def ? _def : def;

  bodies.push_back(obj);
  return obj;
}



// create new frame and add it to body
mjCFrame* mjCBody::AddFrame(mjCFrame* _frame) {
  mjCFrame* obj = new mjCFrame(model, _frame ? _frame : NULL);
  frames.push_back(obj);
  return obj;
}



// create new free joint (no default inheritance) and add it to body
mjCJoint* mjCBody::AddFreeJoint() {
  // create free joint, don't inherit from defaults
  mjCJoint* obj = new mjCJoint(model, NULL);
  obj->spec.type = mjJNT_FREE;

  // set body pointer, add
  obj->body = this;

  joints.push_back(obj);
  return obj;
}



// create new joint and add it to body
mjCJoint* mjCBody::AddJoint(mjCDef* _def) {
  // create joint
  mjCJoint* obj = new mjCJoint(model, _def ? _def : def);

  // set body pointer, add
  obj->body = this;

  joints.push_back(obj);
  return obj;
}



// create new geom and add it to body
mjCGeom* mjCBody::AddGeom(mjCDef* _def) {
  // create geom
  mjCGeom* obj = new mjCGeom(model, _def ? _def : def);

  //  set body pointer, add
  obj->body = this;

  geoms.push_back(obj);
  return obj;
}



// create new site and add it to body
mjCSite* mjCBody::AddSite(mjCDef* _def) {
  // create site
  mjCSite* obj = new mjCSite(model, _def ? _def : def);

  // set body pointer, add
  obj->body = this;

  sites.push_back(obj);
  return obj;
}



// create new camera and add it to body
mjCCamera* mjCBody::AddCamera(mjCDef* _def) {
  // create camera
  mjCCamera* obj = new mjCCamera(model, _def ? _def : def);

  // set body pointer, add
  obj->body = this;

  cameras.push_back(obj);
  return obj;
}



// create new light and add it to body
mjCLight* mjCBody::AddLight(mjCDef* _def) {
  // create light
  mjCLight* obj = new mjCLight(model, _def ? _def : def);

  // set body pointer, add
  obj->body = this;

  lights.push_back(obj);
  return obj;
}



// get number of objects of specified type
int mjCBody::NumObjects(mjtObj type) {
  switch (type) {
  case mjOBJ_BODY:
  case mjOBJ_XBODY:
    return (int)bodies.size();
  case mjOBJ_JOINT:
    return (int)joints.size();
  case mjOBJ_GEOM:
    return (int)geoms.size();
  case mjOBJ_SITE:
    return (int)sites.size();
  case mjOBJ_CAMERA:
    return (int)cameras.size();
  case mjOBJ_LIGHT:
    return (int)lights.size();
  default:
    return 0;
  }
}



// get poiner to specified object
mjCBase* mjCBody::GetObject(mjtObj type, int id) {
  if (id>=0 && id<NumObjects(type)) {
    switch (type) {
    case mjOBJ_BODY:
    case mjOBJ_XBODY:
      return bodies[id];
    case mjOBJ_JOINT:
      return joints[id];
    case mjOBJ_GEOM:
      return geoms[id];
    case mjOBJ_SITE:
      return sites[id];
    case mjOBJ_CAMERA:
      return cameras[id];
    case mjOBJ_LIGHT:
      return lights[id];
    default:
      return 0;
    }
  }

  return 0;
}



// find object by name in given list
template <class T>
static T* findobject(string name, vector<T*>& list) {
  for (unsigned int i=0; i<list.size(); i++) {
    if (list[i]->name == name) {
      return list[i];
    }
  }

  return 0;
}



// recursive find by name
mjCBase* mjCBody::FindObject(mjtObj type, string _name, bool recursive) {
  mjCBase* res = 0;

  // check self: just in case
  if (name == _name) {
    return this;
  }

  // search elements of this body
  if (type==mjOBJ_BODY || type==mjOBJ_XBODY) {
    res = findobject(_name, bodies);
  } else if (type==mjOBJ_JOINT) {
    res = findobject(_name, joints);
  } else if (type==mjOBJ_GEOM) {
    res = findobject(_name, geoms);
  } else if (type==mjOBJ_SITE) {
    res = findobject(_name, sites);
  } else if (type==mjOBJ_CAMERA) {
    res = findobject(_name, cameras);
  } else if (type==mjOBJ_LIGHT) {
    res = findobject(_name, lights);
  }

  // found
  if (res) {
    return res;
  }

  // search children
  if (recursive) {
    for (int i=0; i<(int)bodies.size(); i++) {
      if ((res = bodies[i]->FindObject(type, _name, true))) {
        return res;
      }
    }
  }

  // not found
  return res;
}



// compute geom inertial frame: ipos, iquat, mass, inertia
void mjCBody::GeomFrame(void) {
  int sz;
  double com[3] = {0, 0, 0};
  double toti[6] = {0, 0, 0, 0, 0, 0};
  vector<mjCGeom*> sel;

  // select geoms based on group
  sel.clear();
  for (int i=0; i<geoms.size(); i++) {
    if (geoms[i]->group>=model->inertiagrouprange[0] &&
        geoms[i]->group<=model->inertiagrouprange[1]) {
      sel.push_back(geoms[i]);
    }
  }
  sz = sel.size();

  // single geom: copy
  if (sz==1) {
    mjuu_copyvec(ipos, sel[0]->pos, 3);
    mjuu_copyvec(iquat, sel[0]->quat, 4);
    mass = sel[0]->mass_;
    mjuu_copyvec(inertia, sel[0]->inertia, 3);
  }

  // multiple geoms
  else if (sz>1) {
    // compute total mass and center of mass
    mass = 0;
    for (int i=0; i<sz; i++) {
      mass += sel[i]->mass_;
      com[0] += sel[i]->mass_ * sel[i]->pos[0];
      com[1] += sel[i]->mass_ * sel[i]->pos[1];
      com[2] += sel[i]->mass_ * sel[i]->pos[2];
    }

    // check for small mass
    if (mass<mjMINVAL) {
      throw mjCError(this, "body mass is too small, cannot compute center of mass");
    }

    // ipos = geom com
    ipos[0] = com[0]/mass;
    ipos[1] = com[1]/mass;
    ipos[2] = com[2]/mass;

    // add geom inertias
    for (int i=0; i<sz; i++) {
      double inert0[6], inert1[6];
      double dpos[3] = {
        sel[i]->pos[0] - ipos[0],
        sel[i]->pos[1] - ipos[1],
        sel[i]->pos[2] - ipos[2]
      };

      mjuu_globalinertia(inert0, sel[i]->inertia, sel[i]->quat);
      mjuu_offcenter(inert1, sel[i]->mass_, dpos);
      for (int j=0; j<6; j++) {
        toti[j] = toti[j] + inert0[j] + inert1[j];
      }
    }

    // compute principal axes of inertia
    mjuu_copyvec(fullinertia, toti, 6);
    const char* errq = FullInertia(iquat, inertia);
    if (errq) {
      throw mjCError(this, "error '%s' in alternative for principal axes", errq);
    }
  }
}



// compute full inertia
const char* mjCBody::FullInertia(double quat[4], double inertia[3]) {
  if (!mjuu_defined(fullinertia[0])) {
    return 0;
  }

  mjtNum eigval[3], eigvec[9], quattmp[4];
  mjtNum full[9] = {
    fullinertia[0], fullinertia[3], fullinertia[4],
    fullinertia[3], fullinertia[1], fullinertia[5],
    fullinertia[4], fullinertia[5], fullinertia[2]
  };

  mju_eig3(eigval, eigvec, quattmp, full);

  // check mimimal eigenvalue
  if (eigval[2]<mjEPS) {
    return "inertia must have positive eigenvalues";
  }

  // copy
  for (int i=0; i<4; i++) {
    quat[i] = quattmp[i];
  }

  if (inertia) {
    for (int i=0; i<3; i++) {
      inertia[i] = eigval[i];
    }
  }

  return 0;
}



// set explicitinertial to true
void mjCBody::MakeInertialExplicit() {
  spec.explicitinertial = true;
}


// compiler
void mjCBody::Compile(void) {
  CopyFromSpec();

  // resize userdata
  if (userdata_.size() > model->nuser_body) {
    throw mjCError(this, "user has more values than nuser_body in body '%s' (id = %d)",
                   name.c_str(), id);
  }
  userdata_.resize(model->nuser_body);

  // pos defaults to (0,0,0)
  if (!mjuu_defined(pos[0])) {
     mjuu_setvec(pos, 0, 0, 0);
  }

  // normalize user-defined quaternions
  mjuu_normvec(quat, 4);
  mjuu_normvec(iquat, 4);

  // set parentid and weldid of children
  for (int i=0; i<bodies.size(); i++) {
    bodies[i]->parentid = id;
    bodies[i]->weldid = (!bodies[i]->joints.empty() ? bodies[i]->id : weldid);
  }

  // check and process orientation alternatives for body
  const char* err = alt_.Set(quat, model->degree, model->euler);
  if (err) {
    throw mjCError(this, "error '%s' in frame alternative", err);
  }

  // check and process orientation alternatives for inertia
  const char* ierr = FullInertia(iquat, inertia);
  if (ierr) {
    throw mjCError(this, "error '%s' in inertia alternative", ierr);
  }

  // compile all geoms, phase 1
  for (int i=0; i<geoms.size(); i++) {
    geoms[i]->inferinertia = id>0 &&
      (!explicitinertial || model->inertiafromgeom == mjINERTIAFROMGEOM_TRUE) &&
      geoms[i]->spec.group >= model->inertiagrouprange[0] &&
      geoms[i]->spec.group <= model->inertiagrouprange[1];
    geoms[i]->Compile();
  }

  // set inertial frame from geoms if necessary
  if (id>0 && (model->inertiafromgeom==mjINERTIAFROMGEOM_TRUE ||
               (!mjuu_defined(ipos[0]) && model->inertiafromgeom==mjINERTIAFROMGEOM_AUTO))) {
    GeomFrame();
  }

  // both pos and ipos undefined: error
  if (!mjuu_defined(ipos[0]) && !mjuu_defined(pos[0])) {
    throw mjCError(this, "body pos and ipos are both undefined");
  }

  // ipos undefined: copy body frame into inertial
  else if (!mjuu_defined(ipos[0])) {
    mjuu_copyvec(ipos, pos, 3);
    mjuu_copyvec(iquat, quat, 4);
  }

  // pos undefined: copy inertial frame into body frame
  else if (!mjuu_defined(pos[0])) {
    mjuu_copyvec(pos, ipos, 3);
    mjuu_copyvec(quat, iquat, 4);
  }

  // check and correct mass and inertia
  if (id>0) {
    // fix minimum
    mass = mju_max(mass, model->boundmass);
    inertia[0] = mju_max(inertia[0], model->boundinertia);
    inertia[1] = mju_max(inertia[1], model->boundinertia);
    inertia[2] = mju_max(inertia[2], model->boundinertia);

    // check for negative values
    if (mass<0 || inertia[0]<0 || inertia[1]<0 ||inertia[2]<0) {
      throw mjCError(this, "mass and inertia cannot be negative");
    }

    // check for non-physical inertia
    if (inertia[0] + inertia[1] < inertia[2] ||
        inertia[0] + inertia[2] < inertia[1] ||
        inertia[1] + inertia[2] < inertia[0]) {
      if (model->balanceinertia) {
        inertia[0] = inertia[1] = inertia[2] = (inertia[0] + inertia[1] + inertia[2])/3.0;
      } else {
        throw mjCError(this, "inertia must satisfy A + B >= C; use 'balanceinertia' to fix");
      }
    }
  }

  // frame
  if (frame) {
    mjuu_frameaccumChild(frame->pos, frame->quat, pos, quat);
  }

  // accumulate rbound, contype, conaffinity over geoms
  contype = conaffinity = 0;
  margin = 0;
  for (int i=0; i<geoms.size(); i++) {
    contype |= geoms[i]->contype;
    conaffinity |= geoms[i]->conaffinity;
    margin = mju_max(margin, geoms[i]->margin);
  }

  // compute bounding volume hierarchy
  if (!geoms.empty()) {
    tree.Set(ipos, iquat);
    tree.AllocateBoundingVolumes(geoms.size());
    for (int i=0; i<geoms.size(); i++) {
      geoms[i]->SetBoundingVolume(tree.GetBoundingVolume(i));
    }
    tree.CreateBVH();
  }

  // compile all joints, count dofs
  dofnum = 0;
  for (int i=0; i<joints.size(); i++) {
    dofnum += joints[i]->Compile();
  }

  // check for excessive number of dofs
  if (dofnum>6) {
    throw mjCError(this, "more than 6 dofs in body '%s'", name.c_str());
  }

  // check for rotation dof after ball joint
  bool hasball = false;
  for (int i=0; i<joints.size(); i++) {
    if ((joints[i]->type==mjJNT_BALL || joints[i]->type==mjJNT_HINGE) && hasball) {
      throw mjCError(this, "ball followed by rotation in body '%s'", name.c_str());
    }
    if (joints[i]->type==mjJNT_BALL) {
      hasball = true;
    }
  }

  // make sure mocap body is fixed child of world
  if (mocap)
    if (dofnum || parentid) {
      throw mjCError(this, "mocap body '%s' is not a fixed child of world", name.c_str());
    }

  // compute body global pose (no joint transformations in qpos0)
  if (id>0) {
    mjCBody* par = model->bodies[parentid];
    mju_rotVecQuat(xpos0, pos, par->xquat0);
    mju_addTo3(xpos0, par->xpos0);
    mju_mulQuat(xquat0, par->xquat0, quat);
  }

  // compile all sites
  for (int i=0; i<sites.size(); i++) sites[i]->Compile();

  // compile all cameras
  for (int i=0; i<cameras.size(); i++) cameras[i]->Compile();

  // compile all lights
  for (int i=0; i<lights.size(); i++) lights[i]->Compile();

  // plugin
  if (plugin.active) {
    if (plugin_name.empty() && plugin_instance_name.empty()) {
      throw mjCError(
          this, "neither 'plugin' nor 'instance' is specified for body '%s', (id = %d)",
          name.c_str(), id);
    }

    mjCPlugin** plugin_instance = (mjCPlugin**)&plugin.instance;
    model->ResolvePlugin(this, plugin_name, plugin_instance_name, plugin_instance);
    const mjpPlugin* pplugin = mjp_getPluginAtSlot((*plugin_instance)->spec.plugin_slot);
    if (!(pplugin->capabilityflags & mjPLUGIN_PASSIVE)) {
      throw mjCError(this, "plugin '%s' does not support passive forces", pplugin->name);
    }
  }

  if (!model->discardvisual) {
    return;
  }

  // set inertial to explicit for bodies containing visual geoms
  for (int j=0; j<geoms.size(); j++) {
    if (geoms[j]->IsVisual()) {
      explicitinertial = true;
      break;
    }
  }
}



//------------------ class mjCFrame implementation -------------------------------------------------

// initialize frame
mjCFrame::mjCFrame(mjCModel* _model, mjCFrame* _frame) {
  mjm_defaultFrame(spec);
  compiled = false;
  model = _model;
  frame = _frame ? _frame : NULL;
  PointToLocal();
  CopyFromSpec();
}



mjCFrame::mjCFrame(const mjCFrame& other) {
  *this = other;
}



mjCFrame& mjCFrame::operator=(const mjCFrame& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjCFrame_*>(this) = static_cast<const mjCFrame_&>(other);
    *static_cast<mjmFrame*>(this) = static_cast<const mjmFrame&>(other);
  }
  PointToLocal();
  return *this;
}



void mjCFrame::PointToLocal() {
  spec.element = (mjElement)this;
  spec.name = (mjString)&name;
  spec.childclass = (mjString)&classname;
  spec.info = (mjString)&info;
}



void mjCFrame::CopyFromSpec() {
  *static_cast<mjmFrame*>(this) = spec;
  mju_copy3(pos, spec.pos);
  mju_copy4(quat, spec.quat);
  mju_copy4(alt_.axisangle, alt.axisangle);
  mju_copy(alt_.xyaxes, alt.xyaxes, 6);
  mju_copy3(alt_.zaxis, alt.zaxis);
  mju_copy3(alt_.euler, alt.euler);
}



void mjCFrame::Compile() {
  if (compiled) {
    return;
  }

  CopyFromSpec();
  const char* err = alt_.Set(quat, model->spec.degree, model->spec.euler);
  if (err) {
    throw mjCError(this, "orientation specification error '%s' in site %d", err, id);
  }

  // compile parents and accumulate result
  if (frame) {
    frame->Compile();
    mjuu_frameaccumChild(frame->pos, frame->quat, pos, quat);
  }

  mjuu_normvec(quat, 4);
  compiled = true;
}



//------------------ class mjCJoint implementation -------------------------------------------------

// initialize default joint
mjCJoint::mjCJoint(mjCModel* _model, mjCDef* _def) {
  mjm_defaultJoint(spec);

  // clear internal variables
  spec_userdata_.clear();
  body = 0;

  // reset to default if given
  if (_def) {
    *this = _def->joint;
  }

  // set model, def
  model = _model;
  def = (_def ? _def : (_model ? _model->defaults[0] : 0));

  // point to local
  PointToLocal();

  // in case this joint is not compiled
  CopyFromSpec();
}



mjCJoint::mjCJoint(const mjCJoint& other) {
  *this = other;
}



mjCJoint& mjCJoint::operator=(const mjCJoint& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjCJoint_*>(this) = static_cast<const mjCJoint_&>(other);
    *static_cast<mjmJoint*>(this) = static_cast<const mjmJoint&>(other);
  }
  PointToLocal();
  return *this;
}



bool mjCJoint::is_limited() const { return islimited(limited, range); }
bool mjCJoint::is_actfrclimited() const { return islimited(actfrclimited, actfrcrange); }



void mjCJoint::PointToLocal() {
  spec.element = (mjElement)this;
  spec.name = (mjString)&name;
  spec.classname = (mjString)&classname;
  spec.userdata = (mjDoubleVec)&spec_userdata_;
  spec.info = (mjString)&info;
}



void mjCJoint::CopyFromSpec() {
  *static_cast<mjmJoint*>(this) = spec;
  userdata_ = spec_userdata_;
  userdata = (mjDoubleVec)&spec_userdata_;
}



// compiler
int mjCJoint::Compile(void) {
  CopyFromSpec();

  // resize userdata
  if (userdata_.size() > model->nuser_jnt) {
    throw mjCError(this, "user has more values than nuser_jnt in joint '%s' (id = %d)",
                   name.c_str(), id);
  }
  userdata_.resize(model->nuser_jnt);

  // check springdamper
  if (springdamper[0] || springdamper[1]) {
    if (springdamper[0]<=0 || springdamper[1]<=0) {
      throw mjCError(this,
                     "when defined, springdamper values must be positive in joint '%s' (id = %d)",
                     name.c_str(), id);
    }
  }

  // free joints cannot be limited
  if (type==mjJNT_FREE) {
    limited = mjLIMITED_FALSE;
  }
  // otherwise if limited is auto, check consistency wrt auto-limits
  else if (limited == mjLIMITED_AUTO) {
    bool hasrange = !(range[0]==0 && range[1]==0);
    checklimited(this, model->autolimits, "joint", "", limited, hasrange);
  }

  // resolve limits
  if (is_limited()) {
    // check data
    if (range[0]>=range[1] && type!=mjJNT_BALL) {
      throw mjCError(this,
                     "range[0] should be smaller than range[1] in joint '%s' (id = %d)",
                     name.c_str(), id);
    }
    if (range[0] && type==mjJNT_BALL) {
      throw mjCError(this, "range[0] should be 0 in ball joint '%s' (id = %d)", name.c_str(), id);
    }

    // convert limits to radians
    if (model->degree && (type==mjJNT_HINGE || type==mjJNT_BALL)) {
      if (range[0]) {
        range[0] *= mjPI/180.0;
      }
      if (range[1]) {
        range[1] *= mjPI/180.0;
      }
    }
  }

  // actuator force range: none for free or ball joints
  if (type==mjJNT_FREE || type==mjJNT_BALL) {
    actfrclimited = mjLIMITED_FALSE;
  }
  // otherwise if actfrclimited is auto, check consistency wrt auto-limits
  else if (actfrclimited == mjLIMITED_AUTO) {
    bool hasrange = !(actfrcrange[0]==0 && actfrcrange[1]==0);
    checklimited(this, model->autolimits, "joint", "", actfrclimited, hasrange);
  }

  // resolve actuator force range limits
  if (is_actfrclimited()) {
    // check data
    if (actfrcrange[0]>=actfrcrange[1]) {
      throw mjCError(this,
                     "actfrcrange[0] should be smaller than actfrcrange[1] in joint '%s' (id = %d)",
                     name.c_str(), id);
    }
  }

  // frame
  if (frame) {
    double mat[9];
    mjuu_quat2mat(mat, frame->quat);
    mjuu_mulvecmat(axis, axis, mat);
  }

  // FREE or BALL: set axis to (0,0,1)
  if (type==mjJNT_FREE || type==mjJNT_BALL) {
    axis[0] = axis[1] = 0;
    axis[2] = 1;
  }

  // FREE: set pos to (0,0,0)
  if (type==mjJNT_FREE) {
    mjuu_zerovec(pos, 3);
  }

  // normalize axis, check norm
  if (mjuu_normvec(axis, 3)<mjEPS) {
    throw mjCError(this, "axis too small in joint '%s' (id = %d)", name.c_str(), id);
  }

  // check data
  if (type==mjJNT_FREE && limited == mjLIMITED_TRUE) {
    throw mjCError(this,
                   "limits should not be defined in free joint '%s' (id = %d)", name.c_str(), id);
  }

  // compute local position
  if (type == mjJNT_FREE) {
    mjuu_zerovec(pos, 3);
  } else if (frame) {
    double qunit[4] = {1, 0, 0, 0};
    mjuu_frameaccumChild(frame->pos, frame->quat, pos, qunit);
  }

  // convert reference angles to radians for hinge joints
  if (type==mjJNT_HINGE && model->degree) {
    ref *= mjPI/180.0;
    springref *= mjPI/180.0;
  }

  // return dofnum
  if (type==mjJNT_FREE) {
    return 6;
  } else if (type==mjJNT_BALL) {
    return 3;
  } else {
    return 1;
  }
}



//------------------ class mjCGeom implementation --------------------------------------------------

// initialize default geom
mjCGeom::mjCGeom(mjCModel* _model, mjCDef* _def) {
  mjm_defaultGeom(spec);

  mass_ = 0;
  body = 0;
  matid = -1;
  mesh = nullptr;
  hfield = nullptr;
  visual_ = false;
  mjuu_setvec(inertia, 0, 0, 0);
  inferinertia = true;
  spec_material_.clear();
  spec_userdata_.clear();
  spec_meshname_.clear();
  spec_hfieldname_.clear();
  spec_userdata_.clear();

  for (int i = 0; i < mjNFLUID; i++){
    fluid[i] = 0;
  }

  // reset to default if given
  if (_def) {
    *this = _def->geom;
  }

  // set model, def
  model = _model;
  def = (_def ? _def : (_model ? _model->defaults[0] : 0));

  // point to local
  PointToLocal();

  // in case this geom is not compiled
  CopyFromSpec();
}



mjCGeom::mjCGeom(const mjCGeom& other) {
  *this = other;
}



mjCGeom& mjCGeom::operator=(const mjCGeom& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjCGeom_*>(this) = static_cast<const mjCGeom_&>(other);
    *static_cast<mjmGeom*>(this) = static_cast<const mjmGeom&>(other);
  }
  PointToLocal();
  return *this;
}



// to be called after any default copy constructor
void mjCGeom::PointToLocal(void) {
  spec.element = (mjElement)this;
  spec.name = (mjString)&name;
  spec.info = (mjString)&info;
  spec.classname = (mjString)&classname;
  spec.userdata = (mjDoubleVec)&spec_userdata_;
  spec.material = (mjString)&spec_material_;
  spec.meshname = (mjString)&spec_meshname_;
  spec.hfieldname = (mjString)&spec_hfieldname_;
  spec.plugin.name = (mjString)&plugin_name;
  spec.plugin.instance_name = (mjString)&plugin_instance_name;
}



void mjCGeom::CopyFromSpec() {
  *static_cast<mjmGeom*>(this) = spec;
  userdata_ = spec_userdata_;
  hfieldname_ = spec_hfieldname_;
  meshname_ = spec_meshname_;
  material_ = spec_material_;
  userdata = (mjDoubleVec)&userdata_;
  hfieldname = (mjString)&hfieldname_;
  meshname = (mjString)&meshname_;
  material = (mjString)&material_;
  mju_copy4(alt_.axisangle, alt.axisangle);
  mju_copy(alt_.xyaxes, alt.xyaxes, 6);
  mju_copy3(alt_.zaxis, alt.zaxis);
  mju_copy3(alt_.euler, alt.euler);
  plugin.active = spec.plugin.active;
  plugin.instance = spec.plugin.instance;
  plugin.name = spec.plugin.name;
  plugin.instance_name = spec.plugin.instance_name;
}



// compute geom volume
double mjCGeom::GetVolume(void) {
  double height;

  // get from mesh
  if (type==mjGEOM_MESH || type==mjGEOM_SDF) {
    if (mesh->id<0 || !((std::size_t) mesh->id <= model->meshes.size())) {
      throw mjCError(this, "invalid mesh id in mesh geom '%s' (id = %d)", name.c_str(), id);
    }

    return mesh->GetVolumeRef(typeinertia);
  }

  // compute from geom shape
  else {
    switch (type) {
    case mjGEOM_SPHERE:
      return 4*mjPI*size[0]*size[0]*size[0]/3;

    case mjGEOM_CAPSULE:
      height = 2*size[1];
      return mjPI*(size[0]*size[0]*height + 4*size[0]*size[0]*size[0]/3);

    case mjGEOM_CYLINDER:
      height = 2*size[1];
      return mjPI*size[0]*size[0]*height;

    case mjGEOM_ELLIPSOID:
      return 4*mjPI*size[0]*size[1]*size[2]/3;

    case mjGEOM_HFIELD:
    case mjGEOM_BOX:
      return size[0]*size[1]*size[2]*8;

    default:
      return 0;
    }
  }
}



void mjCGeom::SetBoundingVolume(mjCBoundingVolume* bv) const {
  bv->SetId(&id);
  bv->contype = contype;
  bv->conaffinity = conaffinity;
  bv->aabb = aabb;
  bv->pos = pos;
  bv->quat = quat;
}



// set geom diagonal inertia given density
void mjCGeom::SetInertia(void) {
  double height;

  // get from mesh
  if (type==mjGEOM_MESH || type==mjGEOM_SDF) {
    if (mesh->id<0 || !((std::size_t) mesh->id <= model->meshes.size())) {
      throw mjCError(this, "invalid mesh id in mesh geom '%s' (id = %d)", name.c_str(), id);
    }

    double* boxsz = mesh->GetInertiaBoxPtr(typeinertia);
    inertia[0] = mass_*(boxsz[1]*boxsz[1] + boxsz[2]*boxsz[2]) / 3;
    inertia[1] = mass_*(boxsz[0]*boxsz[0] + boxsz[2]*boxsz[2]) / 3;
    inertia[2] = mass_*(boxsz[0]*boxsz[0] + boxsz[1]*boxsz[1]) / 3;
  }

  // compute from geom shape
  else {
    if (typeinertia)
      throw mjCError(this, "typeinertia currently only available for meshes'%s' (id = %d)",
                     name.c_str(), id);
    switch (type) {
    case mjGEOM_SPHERE:
      inertia[0] = inertia[1] = inertia[2] = 2*mass_*size[0]*size[0]/5;
      return;

    case mjGEOM_CAPSULE: {
      height = 2*size[1];
      double radius = size[0];
      double sphere_mass = mass_*4*radius/(4*radius + 3*height);  // mass*(sphere_vol/total_vol)
      double cylinder_mass = mass_ - sphere_mass;
      // cylinder part
      inertia[0] = inertia[1] = cylinder_mass*(3*radius*radius + height*height)/12;
      inertia[2] = cylinder_mass*radius*radius/2;
      // add two hemispheres, displace along third axis
      double sphere_inertia = 2*sphere_mass*radius*radius/5;
      inertia[0] += sphere_inertia + sphere_mass*height*(3*radius + 2*height)/8;
      inertia[1] += sphere_inertia + sphere_mass*height*(3*radius + 2*height)/8;
      inertia[2] += sphere_inertia;
      return;
    }

    case mjGEOM_CYLINDER:
      height = 2*size[1];
      inertia[0] = inertia[1] = mass_*(3*size[0]*size[0]+height*height)/12;
      inertia[2] = mass_*size[0]*size[0]/2;
      return;

    case mjGEOM_ELLIPSOID:
      inertia[0] = mass_*(size[1]*size[1]+size[2]*size[2])/5;
      inertia[1] = mass_*(size[0]*size[0]+size[2]*size[2])/5;
      inertia[2] = mass_*(size[0]*size[0]+size[1]*size[1])/5;
      return;

    case mjGEOM_HFIELD:
    case mjGEOM_BOX:
      inertia[0] = mass_*(size[1]*size[1]+size[2]*size[2])/3;
      inertia[1] = mass_*(size[0]*size[0]+size[2]*size[2])/3;
      inertia[2] = mass_*(size[0]*size[0]+size[1]*size[1])/3;
      return;

    default:
      inertia[0] = inertia[1] = inertia[2] = 0;
      return;
    }
  }
}



// compute radius of bounding sphere
double mjCGeom::GetRBound(void) {
  const double *aamm, *hsize;
  double haabb[3] = {0};

  switch (type) {
  case mjGEOM_HFIELD:
    hsize = hfield->size;
    return sqrt(hsize[0]*hsize[0] + hsize[1]*hsize[1] +
                mjMAX(hsize[2]*hsize[2], hsize[3]*hsize[3]));

  case mjGEOM_SPHERE:
    return size[0];

  case mjGEOM_CAPSULE:
    return size[0]+size[1];

  case mjGEOM_CYLINDER:
    return sqrt(size[0]*size[0]+size[1]*size[1]);

  case mjGEOM_ELLIPSOID:
    return mju_max(mju_max(size[0], size[1]), size[2]);

  case mjGEOM_BOX:
    return sqrt(size[0]*size[0]+size[1]*size[1]+size[2]*size[2]);

  case mjGEOM_MESH:
  case mjGEOM_SDF:
    aamm = mesh->aamm();
    haabb[0] = mju_max(fabs(aamm[0]), fabs(aamm[3]));
    haabb[1] = mju_max(fabs(aamm[1]), fabs(aamm[4]));
    haabb[2] = mju_max(fabs(aamm[2]), fabs(aamm[5]));
    return sqrt(haabb[0]*haabb[0] + haabb[1]*haabb[1] + haabb[2]*haabb[2]);

  default:
    return 0;
  }
}



// Compute the coefficients of the added inertia due to the surrounding fluid.
double mjCGeom::GetAddedMassKappa(double dx, double dy, double dz) {
  // Integration by Gauss–Kronrod quadrature on interval l in [0, infinity] of
  // f(l) = dx*dy*dz / np.sqrt((dx*dx+ l)**3 * (dy*dy+ l) * (dz*dz+ l))
  // 15-point Gauss–Kronrod quadrature (K15) points x in [0, 1].

  // static constexpr mjtNum kronrod_x[15] = [     // unused, left in comment for completeness
  //   0.00427231, 0.02544604, 0.06756779, 0.12923441, 0.20695638,
  //   0.29707742, 0.39610752, 0.50000000, 0.60389248, 0.70292258,
  //   0.79304362, 0.87076559, 0.93243221, 0.97455396, 0.99572769];
  // 15-point Gauss–Kronrod quadrature (K15) weights.
  static constexpr double kronrod_w[15] = {
    0.01146766, 0.03154605, 0.05239501, 0.07032663, 0.08450236,
    0.09517529, 0.10221647, 0.10474107, 0.10221647, 0.09517529,
    0.08450236, 0.07032663, 0.05239501, 0.03154605, 0.01146766};
  // Integrate from 0 to inf by change of variables:
  // l = x^3 / (1-x)^2. Exponents 3 and 2 found to minimize error.
  static constexpr double kronrod_l[15] = {
    7.865151709349917e-08, 1.7347976913907274e-05, 0.0003548008144506193,
    0.002846636252924549, 0.014094260903596077, 0.053063261727396636,
    0.17041978741317773, 0.5, 1.4036301548686991, 3.9353484827022642,
    11.644841677041734, 39.53187807410903, 177.5711362220801,
    1429.4772912937397, 54087.416549217705};
  // dl = dl/dx dx. The following are dl/dx(x).
  static constexpr double kronrod_d[15] = {
    5.538677720489877e-05, 0.002080868285293228, 0.016514126520723166,
    0.07261900344370877, 0.23985243401862602, 0.6868318249020725,
    1.8551129519182894, 5.0, 14.060031152313941, 43.28941239611009,
    156.58546376397112, 747.9826085305024, 5827.4042950027115,
    116754.0197944512, 25482945.327264845};

  const double invdx2 = 1.0 / (dx * dx);
  const double invdy2 = 1.0 / (dy * dy);
  const double invdz2 = 1.0 / (dz * dz);

  // for added numerical stability we non-dimensionalize x by scale
  // because 1 + l/d^2 in denom, l should be scaled by d^2
  const double scale = std::pow(dx*dx*dx * dy * dz, 0.4);  // ** (2/5)
  double kappa = 0.0;
  for (int i = 0; i < 15; ++i) {
    const double lambda = scale * kronrod_l[i];
    const double denom = (1 + lambda*invdx2) * std::sqrt(
      (1 + lambda*invdx2) * (1 + lambda*invdy2) * (1 + lambda*invdz2));
    kappa += scale * kronrod_d[i] / denom * kronrod_w[i];
  }
  return kappa * invdx2;
}



// Compute the kappa coefs of the added inertia due to the surrounding fluid.
void mjCGeom::SetFluidCoefs(void) {
  double dx, dy, dz;

  // get semiaxes
  switch (type) {
    case mjGEOM_SPHERE:
      dx = size[0];
      dy = size[0];
      dz = size[0];
      break;

    case mjGEOM_CAPSULE:
      dx = size[0];
      dy = size[0];
      dz = size[1] + size[0];
      break;

    case mjGEOM_CYLINDER:
      dx = size[0];
      dy = size[0];
      dz = size[1];
      break;

    default:
      dx = size[0];
      dy = size[1];
      dz = size[2];
  }

  // volume of equivalent ellipsoid
  const double volume = 4.0 / 3.0 * mjPI * dx * dy * dz;

  // GetAddedMassKappa is invariant to permutation of last two arguments
  const double kx = GetAddedMassKappa(dx, dy, dz);
  const double ky = GetAddedMassKappa(dy, dz, dx);
  const double kz = GetAddedMassKappa(dz, dx, dy);

  // coefficients of virtual moment of inertia. Note: if (kz-ky) in numerator
  // is negative, also the denom is negative. Abs both and clip to MINVAL
  const auto pow2 = [](const double val) { return val * val; };
  const double Ixfac = pow2(dy*dy - dz*dz) * std::fabs(kz - ky) / std::max(
    mjMINVAL, std::fabs(2*(dy*dy - dz*dz) + (dy*dy + dz*dz)*(ky - kz)));
  const double Iyfac = pow2(dz*dz - dx*dx) * std::fabs(kx - kz) / std::max(
    mjMINVAL, std::fabs(2*(dz*dz - dx*dx) + (dz*dz + dx*dx)*(kz - kx)));
  const double Izfac = pow2(dx*dx - dy*dy) * std::fabs(ky - kx) / std::max(
    mjMINVAL, std::fabs(2*(dx*dx - dy*dy) + (dx*dx + dy*dy)*(kx - ky)));

  const mjtNum virtual_mass[3] = {
      volume * kx / std::max(mjMINVAL, 2-kx),
      volume * ky / std::max(mjMINVAL, 2-ky),
      volume * kz / std::max(mjMINVAL, 2-kz)};
  const mjtNum virtual_inertia[3] = {volume*Ixfac/5, volume*Iyfac/5, volume*Izfac/5};

  writeFluidGeomInteraction(fluid, &fluid_ellipsoid, &fluid_coefs[0],
                            &fluid_coefs[1], &fluid_coefs[2],
                            &fluid_coefs[3], &fluid_coefs[4],
                            virtual_mass, virtual_inertia);
}


// compute bounding box
void mjCGeom::ComputeAABB(void) {
  double aamm[6]; // axis-aligned bounding box in (min, max) format
  switch (type) {
  case mjGEOM_HFIELD:
    aamm[0] = -hfield->size[0];
    aamm[1] = -hfield->size[1];
    aamm[2] = -hfield->size[3];
    aamm[3] = hfield->size[0];
    aamm[4] = hfield->size[1];
    aamm[5] = hfield->size[2];
    break;

  case mjGEOM_SPHERE:
    aamm[3] = aamm[4] = aamm[5] = size[0];
    mjuu_setvec(aamm, -aamm[3], -aamm[4], -aamm[5]);
    break;

  case mjGEOM_CAPSULE:
    aamm[3] = aamm[4] = size[0];
    aamm[5] = size[0] + size[1];
    mjuu_setvec(aamm, -aamm[3], -aamm[4], -aamm[5]);
    break;

  case mjGEOM_CYLINDER:
    aamm[3] = aamm[4] = size[0];
    aamm[5] = size[1];
    mjuu_setvec(aamm, -aamm[3], -aamm[4], -aamm[5]);
    break;

  case mjGEOM_MESH:
  case mjGEOM_SDF:
    mjuu_copyvec(aamm, mesh->aamm(), 6);
    break;

  case mjGEOM_PLANE:
    aamm[0] = aamm[1] = aamm[2] = -mjMAXVAL;
    aamm[3] = aamm[4] = mjMAXVAL;
    aamm[5] = 0;
    break;

  default:
    mjuu_copyvec(aamm+3, size, 3);
    mjuu_setvec(aamm, -size[0], -size[1], -size[2]);
    break;
  }

  // convert aamm to aabb (center, size) format
  double pos[] = {(aamm[3] + aamm[0]) / 2, (aamm[4] + aamm[1]) / 2,
                  (aamm[5] + aamm[2]) / 2};
  double size[] = {(aamm[3] - aamm[0]) / 2, (aamm[4] - aamm[1]) / 2,
                   (aamm[5] - aamm[2]) / 2};
  mjuu_copyvec(aabb, pos, 3);
  mjuu_copyvec(aabb+3, size, 3);
}



// compiler
void mjCGeom::Compile(void) {
  CopyFromSpec();

  // resize userdata
  if (userdata_.size() > model->nuser_geom) {
    throw mjCError(this, "user has more values than nuser_geom in geom '%s' (id = %d)",
                   name.c_str(), id);
  }
  userdata_.resize(model->nuser_geom);

  // check type
  if (type<0 || type>=mjNGEOMTYPES) {
    throw mjCError(this, "invalid type in geom '%s' (id = %d)", name.c_str(), id);
  }

  // check condim
  if (condim!=1 && condim!=3 && condim!=4 && condim!=6) {
    throw mjCError(this, "invalid condim in geom '%s' (id = %d)", name.c_str(), id);
  }

  // check mesh
  if ((type==mjGEOM_MESH || type==mjGEOM_SDF) && !mesh) {
    throw mjCError(this, "mesh geom '%s' (id = %d) must have valid meshid", name.c_str(), id);
  }

  // check hfield
  if ((type==mjGEOM_HFIELD && !hfield) || (type!=mjGEOM_HFIELD && hfield)) {
    throw mjCError(this, "hfield geom '%s' (id = %d) must have valid hfieldid", name.c_str(), id);
  }

  // plane only allowed in static bodies
  if (type==mjGEOM_PLANE && body->weldid!=0) {
    throw mjCError(this, "plane only allowed in static bodies: geom '%s' (id = %d)",
                   name.c_str(), id);
  }

  // check if can collide
  visual_ = !contype && !conaffinity;

  // normalize quaternion
  mjuu_normvec(quat, 4);

  // 'fromto': compute pos, quat, size
  if (mjuu_defined(fromto[0])) {
    // check type
    if (type!=mjGEOM_CAPSULE &&
        type!=mjGEOM_CYLINDER &&
        type!=mjGEOM_ELLIPSOID &&
        type!=mjGEOM_BOX) {
      throw mjCError(this,
                     "fromto requires capsule, cylinder, box or ellipsoid in geom '%s' (id = %d)",
                     name.c_str(), id);
    }

    // make sure pos is not defined; cannot use mjuu_defined because default is (0,0,0)
    if (pos[0] || pos[1] || pos[2]) {
      throw mjCError(this,
                     "both pos and fromto defined in geom '%s' (id = %d)",
                     name.c_str(), id);
    }

    // size[1] = length (for capsule and cylinder)
    double vec[3] = {
      fromto[0]-fromto[3],
      fromto[1]-fromto[4],
      fromto[2]-fromto[5]
    };
    size[1] = mjuu_normvec(vec, 3)/2;
    if (size[1]<mjEPS) {
      throw mjCError(this, "fromto points too close in geom '%s' (id = %d)", name.c_str(), id);
    }

    // adjust size for ellipsoid and box
    if (type==mjGEOM_ELLIPSOID || type==mjGEOM_BOX) {
      size[2] = size[1];
      size[1] = size[0];
    }

    // compute position
    pos[0] = (fromto[0]+fromto[3])/2;
    pos[1] = (fromto[1]+fromto[4])/2;
    pos[2] = (fromto[2]+fromto[5])/2;

    // compute orientation
    mjuu_z2quat(quat, vec);
  }

  // not 'fromto': try alternative
  else {
    const char* err = alt_.Set(quat, model->degree, model->euler);
    if (err) {
      throw mjCError(this, "orientation specification error '%s' in geom %d", err, id);
    }
  }

  // mesh: accumulate frame, fit geom if needed
  if (mesh) {
    // check for inapplicable fromto
    if (mjuu_defined(fromto[0])) {
      throw mjCError(this, "fromto cannot be used with mesh geom '%s' (id = %d)", name.c_str(), id);
    }

    // save reference in case this is not an mjGEOM_MESH
    mjCMesh* pmesh = mesh;

    // fit geom if type is not mjGEOM_MESH
    double meshpos[3];
    if (type!=mjGEOM_MESH && type!=mjGEOM_SDF) {
      mesh->FitGeom(this, meshpos);

      // remove reference to mesh
      meshname_.clear();
      mesh = nullptr;
    } else {
      mjuu_copyvec(meshpos, mesh->GetPosPtr(typeinertia), 3);
    }

    // apply geom pos/quat as offset
    mjuu_frameaccum(pos, quat, meshpos, pmesh->GetQuatPtr(typeinertia));
    mjuu_copyvec(pmesh->GetOffsetPosPtr(), meshpos, 3);
    mjuu_copyvec(pmesh->GetOffsetQuatPtr(), pmesh->GetQuatPtr(typeinertia), 4);
  }

  // check size parameters
  checksize(size, type, this, name.c_str(), id);

  // set hfield sizes in geom.size
  if (type==mjGEOM_HFIELD) {
    size[0] = hfield->size[0];
    size[1] = hfield->size[1];
    size[2] = 0.5*(0.5*hfield->size[2] +
                   hfield->size[3]);
  } else if (type==mjGEOM_MESH || type==mjGEOM_SDF) {
    const double* aamm = mesh->aamm();
    size[0] = mju_max(fabs(aamm[0]), fabs(aamm[3]));
    size[1] = mju_max(fabs(aamm[1]), fabs(aamm[4]));
    size[2] = mju_max(fabs(aamm[2]), fabs(aamm[5]));
  }

  for (double s : size) {
    if (std::isnan(s)) {
      throw mjCError(this, "nan size in geom '%s' (id = %d)", name.c_str(), id);
    }
  }
  // compute aabb
  ComputeAABB();

  // compute geom mass and inertia
  if (inferinertia) {
    if (mjuu_defined(mass)) {
      if (mass==0) {
        mass_ = 0;
        density = 0;
      } else if (GetVolume()>mjMINVAL) {
        mass_ = mass;
        density = mass / GetVolume();
        SetInertia();
      }
    } else {
      mass_ = density * GetVolume();
      SetInertia();
    }

    // check for negative values
    if (mass_<0 || inertia[0]<0 || inertia[1]<0 || inertia[2]<0 || density<0)
      throw mjCError(this, "mass, inertia or density are negative in geom '%s' (id = %d)",
                    name.c_str(), id);
  }

  // fluid-interaction coefficients, requires computed inertia and mass
  if (fluid_ellipsoid > 0) {
    SetFluidCoefs();
  }

  // plugin
  if (plugin.active) {
    if (plugin_name.empty() && plugin_instance_name.empty()) {
      throw mjCError(
          this, "neither 'plugin' nor 'instance' is specified for geom '%s', (id = %d)",
          name.c_str(), id);
    }

    mjCPlugin** plugin_instance = (mjCPlugin**)&plugin.instance;
    model->ResolvePlugin(this, plugin_name, plugin_instance_name, plugin_instance);
    const mjpPlugin* pplugin = mjp_getPluginAtSlot((*plugin_instance)->spec.plugin_slot);
    if (!(pplugin->capabilityflags & mjPLUGIN_SDF)) {
      throw mjCError(this, "plugin '%s' does not support sign distance fields", pplugin->name);
    }
  }

  // frame
  if (frame) {
    mjuu_frameaccumChild(frame->pos, frame->quat, pos, quat);
  }
}



//------------------ class mjCSite implementation --------------------------------------------------

// initialize default site
mjCSite::mjCSite(mjCModel* _model, mjCDef* _def) {
  mjm_defaultSite(spec);

  // clear internal variables
  body = 0;
  matid = -1;
  spec_material_.clear();
  spec_userdata_.clear();

  // reset to default if given
  if (_def) {
    *this = _def->site;
  }

  // point to local
  PointToLocal();

  // in case this site is not compiled
  CopyFromSpec();

  // set model, def
  model = _model;
  def = (_def ? _def : (_model ? _model->defaults[0] : 0));
}



mjCSite::mjCSite(const mjCSite& other) {
  *this = other;
}



mjCSite& mjCSite::operator=(const mjCSite& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjCSite_*>(this) = static_cast<const mjCSite_&>(other);
    *static_cast<mjmSite*>(this) = static_cast<const mjmSite&>(other);
  }
  PointToLocal();
  return *this;
}



void mjCSite::PointToLocal() {
  spec.element = (mjElement)this;
  spec.name = (mjString)&name;
  spec.info = (mjString)&info;
  spec.classname = (mjString)&classname;
  spec.material = (mjString)&spec_material_;
  spec.userdata = (mjDoubleVec)&spec_userdata_;
}



void mjCSite::CopyFromSpec() {
  *static_cast<mjmSite*>(this) = spec;
  userdata_ = spec_userdata_;
  material_ = spec_material_;
  userdata = (mjDoubleVec)&userdata_;
  material = (mjString)&material_;
  mju_copy4(alt_.axisangle, alt.axisangle);
  mju_copy(alt_.xyaxes, alt.xyaxes, 6);
  mju_copy3(alt_.zaxis, alt.zaxis);
  mju_copy3(alt_.euler, alt.euler);
}



// compiler
void mjCSite::Compile(void) {
  CopyFromSpec();

  // resize userdata
  if (userdata_.size() > model->nuser_site) {
    throw mjCError(this, "user has more values than nuser_site in site '%s' (id = %d)",
                   name.c_str(), id);
  }
  userdata_.resize(model->nuser_site);

  // check type
  if (type<0 || type>=mjNGEOMTYPES) {
    throw mjCError(this, "invalid type in site '%s' (id = %d)", name.c_str(), id);
  }

  // do not allow meshes, hfields and planes
  if (type==mjGEOM_MESH || type==mjGEOM_HFIELD || type==mjGEOM_PLANE) {
    throw mjCError(this, "meshes, hfields and planes not allowed in site '%s' (id = %d)",
                   name.c_str(), id);
  }

  // 'fromto': compute pos, quat, size
  if (mjuu_defined(fromto[0])) {
    // check type
    if (type!=mjGEOM_CAPSULE &&
        type!=mjGEOM_CYLINDER &&
        type!=mjGEOM_ELLIPSOID &&
        type!=mjGEOM_BOX) {
      throw mjCError(this,
                     "fromto requires capsule, cylinder, box or ellipsoid in geom '%s' (id = %d)",
                     name.c_str(), id);
    }

    // make sure pos is not defined; cannot use mjuu_defined because default is (0,0,0)
    if (pos[0] || pos[1] || pos[2]) {
      throw mjCError(this, "both pos and fromto defined in geom '%s' (id = %d)", name.c_str(), id);
    }

    // size[1] = length (for capsule and cylinder)
    double vec[3] = {
      fromto[0]-fromto[3],
      fromto[1]-fromto[4],
      fromto[2]-fromto[5]
    };
    size[1] = mjuu_normvec(vec, 3)/2;
    if (size[1]<mjEPS) {
      throw mjCError(this, "fromto points too close in geom '%s' (id = %d)", name.c_str(), id);
    }

    // adjust size for ellipsoid and box
    if (type==mjGEOM_ELLIPSOID || type==mjGEOM_BOX) {
      size[2] = size[1];
      size[1] = size[0];
    }

    // compute position
    pos[0] = (fromto[0]+fromto[3])/2;
    pos[1] = (fromto[1]+fromto[4])/2;
    pos[2] = (fromto[2]+fromto[5])/2;

    // compute orientation
    mjuu_z2quat(quat, vec);
  }

  // alternative orientation
  else {
    const char* err = alt_.Set(quat, model->degree, model->euler);
    if (err) {
      throw mjCError(this, "orientation specification error '%s' in site %d", err, id);
    }
  }

  // frame
  if (frame) {
    mjuu_frameaccumChild(frame->pos, frame->quat, pos, quat);
  }

  // normalize quaternion
  mjuu_normvec(quat, 4);

  // check size parameters
  checksize(size, type, this, name.c_str(), id);
}



//------------------ class mjCCamera implementation ------------------------------------------------

// initialize defaults
mjCCamera::mjCCamera(mjCModel* _model, mjCDef* _def) {
  mjm_defaultCamera(spec);

  // clear private variables
  body = 0;
  targetbodyid = -1;
  spec_targetbody_.clear();

  // reset to default if given
  if (_def) {
    *this = _def->camera;
  }

  // set model, def
  model = _model;
  def = (_def ? _def : (_model ? _model->defaults[0] : 0));

  // point to local
  PointToLocal();

  // in case this camera is not compiled
  CopyFromSpec();
}



mjCCamera::mjCCamera(const mjCCamera& other) {
  *this = other;
}



mjCCamera& mjCCamera::operator=(const mjCCamera& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjCCamera_*>(this) = static_cast<const mjCCamera_&>(other);
    *static_cast<mjmCamera*>(this) = static_cast<const mjmCamera&>(other);
  }
  PointToLocal();
  return *this;
}



void mjCCamera::PointToLocal() {
  spec.element = (mjElement)this;
  spec.name = (mjString)&name;
  spec.classname = (mjString)&classname;
  spec.userdata = (mjDoubleVec)&spec_userdata_;
  spec.targetbody = (mjString)&spec_targetbody_;
  spec.info = (mjString)&info;
}



void mjCCamera::CopyFromSpec() {
  *static_cast<mjmCamera*>(this) = spec;
  userdata_ = spec_userdata_;
  targetbody_ = spec_targetbody_;
  userdata = (mjDoubleVec)&userdata_;
  targetbody = (mjString)&targetbody_;
  mju_copy4(alt_.axisangle, alt.axisangle);
  mju_copy(alt_.xyaxes, alt.xyaxes, 6);
  mju_copy3(alt_.zaxis, alt.zaxis);
  mju_copy3(alt_.euler, alt.euler);
}



// compiler
void mjCCamera::Compile(void) {
  CopyFromSpec();

  // resize userdata
  if (userdata_.size() > model->nuser_cam) {
    throw mjCError(this, "user has more values than nuser_cam in camera '%s' (id = %d)",
                   name.c_str(), id);
  }
  userdata_.resize(model->nuser_cam);

  // process orientation specifications
  const char* err = alt_.Set(quat, model->degree, model->euler);
  if (err) {
    throw mjCError(this, "orientation specification error '%s' in camera %d", err, id);
  }

  // frame
  if (frame) {
    mjuu_frameaccumChild(frame->pos, frame->quat, pos, quat);
  }

  // normalize quaternion
  mjuu_normvec(quat, 4);

  // get targetbodyid
  if (!targetbody_.empty()) {
    mjCBody* tb = (mjCBody*)model->FindObject(mjOBJ_BODY, targetbody_);
    if (tb) {
      targetbodyid = tb->id;
    } else {
      throw mjCError(this, "unknown target body in camera '%s' (id = %d)", name.c_str(), id);
    }
  }

  // make sure it is not targeting parent body
  if (targetbodyid==body->id) {
    throw mjCError(this, "parent-targeting in camera '%s' (id = %d)", name.c_str(), id);
  }

  // make sure the image size is finite
  if (fovy >= 180) {
    throw mjCError(this, "fovy too large in camera '%s' (id = %d, value = %d)",
                   name.c_str(), id, fovy);
  }

  // check that specs are not duplicated
  if ((principal_length[0] && principal_pixel[0]) ||
      (principal_length[1] && principal_pixel[1])) {
    throw mjCError(this, "principal length duplicated in camera '%s' (id = %d)",
                   name.c_str(), id);
  }

  if ((focal_length[0] && focal_pixel[0]) ||
      (focal_length[1] && focal_pixel[1])) {
    throw mjCError(this, "focal length duplicated in camera '%s' (id = %d)",
                   name.c_str(), id);
  }

  // compute number of pixels per unit length
  if (sensor_size[0]>0 && sensor_size[1]>0) {
    float pixel_density[2] = {
      (float)resolution[0] / sensor_size[0],
      (float)resolution[1] / sensor_size[1],
    };

    // defaults are zero, so only one term in each sum is nonzero
    intrinsic[0] = focal_pixel[0] / pixel_density[0] + focal_length[0];
    intrinsic[1] = focal_pixel[1] / pixel_density[1] + focal_length[1];
    intrinsic[2] = principal_pixel[0] / pixel_density[0] + principal_length[0];
    intrinsic[3] = principal_pixel[1] / pixel_density[1] + principal_length[1];

    // fovy with principal point at (0, 0)
    fovy = mju_atan2((float)sensor_size[1]/2, intrinsic[1]) * 360.0 / mjPI;
  } else {
    intrinsic[0] = model->visual.map.znear;
    intrinsic[1] = model->visual.map.znear;
  }
}



//------------------ class mjCLight implementation -------------------------------------------------

// initialize defaults
mjCLight::mjCLight(mjCModel* _model, mjCDef* _def) {
  mjm_defaultLight(spec);

  // clear private variables
  body = 0;
  targetbodyid = -1;
  spec_targetbody_.clear();

  // reset to default if given
  if (_def) {
    *this = _def->light;
  }

  // set model, def
  model = _model;
  def = (_def ? _def : (_model ? _model->defaults[0] : 0));

  PointToLocal();
  CopyFromSpec();
}



mjCLight::mjCLight(const mjCLight& other) {
  *this = other;
}



mjCLight& mjCLight::operator=(const mjCLight& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjCLight_*>(this) = static_cast<const mjCLight_&>(other);
    *static_cast<mjmLight*>(this) = static_cast<const mjmLight&>(other);
  }
  PointToLocal();
  return *this;
}



void mjCLight::PointToLocal() {
  spec.element = (mjElement)this;
  spec.name = (mjString)&name;
  spec.classname = (mjString)&classname;
  spec.targetbody = (mjString)&spec_targetbody_;
  spec.info = (mjString)&info;
}



void mjCLight::CopyFromSpec() {
  *static_cast<mjmLight*>(this) = spec;
  targetbody_ = spec_targetbody_;
  targetbody = (mjString)&targetbody_;
}



// compiler
void mjCLight::Compile(void) {
  CopyFromSpec();

  double quat[4]= {1, 0, 0, 0};

  // frame
  if (frame) {
    mjuu_frameaccumChild(frame->pos, frame->quat, pos, quat);
  }

  // normalize direction, make sure it is not zero
  if (mjuu_normvec(dir, 3)<mjMINVAL) {
    throw mjCError(this, "zero direction in light '%s' (id = %d)", name.c_str(), id);
  }

  // get targetbodyid
  if (!targetbody_.empty()) {
    mjCBody* tb = (mjCBody*)model->FindObject(mjOBJ_BODY, targetbody_);
    if (tb) {
      targetbodyid = tb->id;
    } else {
      throw mjCError(this, "unknown target body in light '%s' (id = %d)", name.c_str(), id);
    }
  }

  // make sure it is not self-targeting
  if (targetbodyid==body->id) {
    throw mjCError(this, "parent-targeting in light '%s' (id = %d)", name.c_str(), id);
  }
}



//------------------------- class mjCHField --------------------------------------------------------

// constructor
mjCHField::mjCHField(mjCModel* _model) {
  mjm_defaultHField(spec);

  // set model pointer
  model = _model;

  // clear variables
  data = 0;
  spec_file_.clear();
  spec_userdata_.clear();

  // point to local
  PointToLocal();

  // copy from spec
  CopyFromSpec();
}



void mjCHField::PointToLocal() {
  spec.element = (mjElement)this;
  spec.name = (mjString)&name;
  spec.file = (mjString)&spec_file_;
  spec.content_type = (mjString)&spec_content_type_;
  spec.userdata = (mjFloatVec)&spec_userdata_;
  spec.info = (mjString)&info;
}



void mjCHField::CopyFromSpec() {
  *static_cast<mjmHField*>(this) = spec;
  file_ = spec_file_;
  content_type_ = spec_content_type_;
  userdata_ = spec_userdata_;
  file = (mjString)&file_;
  content_type = (mjString)&content_type_;
  userdata = (mjFloatVec)&userdata_;

  // clear precompiled asset. TODO: use asset cache
  if (data) {
    mju_free(data);
    data = 0;
  }
  if (!file_.empty()) {
    nrow = 0;
    ncol = 0;
  }
}



// destructor
mjCHField::~mjCHField() {
  if (data) {
    mju_free(data);
  }
  userdata_.clear();
  spec_userdata_.clear();
}



// load elevation data from custom format
void mjCHField::LoadCustom(mjResource* resource) {
  // get file data in buffer
  const void* buffer = 0;
  int buffer_sz = mju_readResource(resource, &buffer);

  if (buffer_sz < 1) {
    throw mjCError(this, "could not read hfield file '%s'", resource->name);
  } else if (!buffer_sz) {
    throw mjCError(this, "empty hfield file '%s'", resource->name);
  }


  if (buffer_sz < 2*sizeof(int)) {
    throw mjCError(this, "hfield missing header '%s'", resource->name);
  }

  // read dimensions
  int* pint = (int*)buffer;
  nrow = pint[0];
  ncol = pint[1];

  // check dimensions
  if (nrow<1 || ncol<1) {
    throw mjCError(this, "non-positive hfield dimensions in file '%s'", resource->name);
  }

  // check buffer size
  if (buffer_sz != nrow*ncol*sizeof(float)+8) {
    throw mjCError(this, "unexpected file size in file '%s'", resource->name);
  }

  // allocate
  data = (float*) mju_malloc(nrow*ncol*sizeof(float));
  if (!data) {
    throw mjCError(this, "could not allocate buffers in hfield");
  }

  // copy data
  memcpy(data, (void*)(pint+2), nrow*ncol*sizeof(float));
}



// load elevation data from PNG format
void mjCHField::LoadPNG(mjResource* resource) {
  // determine data source
  const void* inbuffer = 0;
  int inbuffer_sz = mju_readResource(resource, &inbuffer);

  if (inbuffer_sz < 1) {
    throw mjCError(this, "could not read hfield PNG file '%s'", resource->name);
  }

  if (!inbuffer_sz) {
    throw mjCError(this, "empty hfield PNG file '%s'", resource->name);
  }

  // load PNG from file or memory
  unsigned int w, h, err;
  std::vector<unsigned char> image;
  err = lodepng::decode(image, w, h, (const unsigned char*) inbuffer, inbuffer_sz, LCT_GREY, 8);

  // check
  if (err) {
    throw mjCError(this, "PNG load error '%s' in hfield id = %d", lodepng_error_text(err), id);
  }
  if (!w || !h) {
    throw mjCError(this, "Zero dimension in PNG hfield '%s' (id = %d)", resource->name, id);
  }

  // allocate
  data = (float*) mju_malloc(w*h*sizeof(float));
  if (!data) {
    throw mjCError(this, "could not allocate buffers in hfield");
  }

  // assign and copy
  ncol = w;
  nrow = h;
  for (int c=0; c<ncol; c++)
    for (int r=0; r<nrow; r++) {
      data[c+(nrow-1-r)*ncol] = (float)image[c+r*ncol];
    }
  image.clear();
}



// compiler
void mjCHField::Compile(const mjVFS* vfs) {
  CopyFromSpec();

  // copy userdata into data
  if (!userdata_.empty()) {
    data = (float*) mju_malloc(nrow*ncol*sizeof(float));
    if (!data) {
      throw mjCError(this, "could not allocate buffers in hfield");
    }
    memcpy(data, userdata_.data(), nrow*ncol*sizeof(float));
  }

  // check size parameters
  for (int i=0; i<4; i++)
    if (size[i]<=0)
      throw mjCError(this,
                     "size parameter is not positive in hfield '%s' (id = %d)", name.c_str(), id);

  // remove path from file if necessary
  if (model->strippath) {
    file_ = mjuu_strippath(file_);
  }

  // load from file if specified
  if (!file_.empty()) {
    // make sure hfield was not already specified manually
    if (nrow || ncol || data) {
      throw mjCError(this,
                     "hfield '%s' (id = %d) specified from file and manually", name.c_str(), id);
    }

    std::string asset_type = GetAssetContentType(file_, content_type_);

    // fallback to custom
    if (asset_type.empty()) {
      asset_type = "image/vnd.mujoco.hfield";
    }

    if (asset_type != "image/png" && asset_type != "image/vnd.mujoco.hfield") {
      throw mjCError(this, "unsupported content type: '%s'", asset_type.c_str());
    }

    string filename = mjuu_makefullname(model->modelfiledir_, model->meshdir_, file_);
    mjResource* resource = LoadResource(filename, vfs);

    try {
      if (asset_type == "image/png") {
        LoadPNG(resource);
      } else {
        LoadCustom(resource);
      }
      mju_closeResource(resource);
    } catch(mjCError err) {
      mju_closeResource(resource);
      throw err;
    }
  }

  // make sure hfield was specified (from file or manually)
  if (nrow<1 || ncol<1 || data==0) {
    throw mjCError(this, "hfield '%s' (id = %d) not specified", name.c_str(), id);
  }

  // set elevation data to [0-1] range
  float emin = 1E+10, emax = -1E+10;
  for (int i = 0; i<nrow*ncol; i++) {
    emin = mjMIN(emin, data[i]);
    emax = mjMAX(emax, data[i]);
  }
  if (emin>emax) {
    throw mjCError(this, "invalid data range in hfield '%s'", file_.c_str());
  }
  for (int i=0; i<nrow*ncol; i++) {
    data[i] -= emin;
    if (emax-emin>mjMINVAL) {
      data[i] /= (emax - emin);
    }
  }
}



//------------------ class mjCTexture implementation -----------------------------------------------

// initialize defaults
mjCTexture::mjCTexture(mjCModel* _model) {
  mjm_defaultTexture(spec);

  // set model pointer
  model = _model;

  // clear user settings: single file
  spec_file_.clear();
  spec_content_type_.clear();

  // clear user settings: separate file
  spec_cubefiles_.assign(6, "");

  // clear internal variables
  rgb = 0;

  // point to local
  PointToLocal();

  // in case this camera is not compiled
  CopyFromSpec();
}



void mjCTexture::PointToLocal() {
  spec.element = (mjElement)this;
  spec.name = (mjString)&name;
  spec.classname = (mjString)&classname;
  spec.file = (mjString)&spec_file_;
  spec.content_type = (mjString)&spec_content_type_;
  spec.cubefiles = (mjStringVec)&spec_cubefiles_;
  spec.info = (mjString)&info;
}



void mjCTexture::CopyFromSpec() {
  *static_cast<mjmTexture*>(this) = spec;
  file_ = spec_file_;
  content_type_ = spec_content_type_;
  cubefiles_ = spec_cubefiles_;
  file = (mjString)&file_;
  content_type = (mjString)&content_type_;
  cubefiles = (mjStringVec)&cubefiles_;

  // clear precompiled asset. TODO: use asset cache
  if (rgb) {
    mju_free(rgb);
    rgb = 0;
  }
}



// free data storage allocated by lodepng
mjCTexture::~mjCTexture() {
  if (rgb) {
    mju_free(rgb);
    rgb = 0;
  }
}



// insert random dots
static void randomdot(unsigned char* rgb, const double* markrgb,
                      int width, int height, double probability) {
  for (int r=0; r<height; r++) {
    for (int c=0; c<width; c++) {
      if (rand()<probability*RAND_MAX) {
        for (int j=0; j<3; j++) {
          rgb[3*(r*width+c)+j] = (mjtByte)(255*markrgb[j]);
        }
      }
    }
  }
}



// interpolate between colors based on value in (-1, +1)
static void interp(unsigned char* rgb, const double* rgb1, const double* rgb2, double pos) {
  const double correction = 1.0/sqrt(2);
  double alpha = 0.5*(1 + pos/sqrt(1+pos*pos)/correction);
  if (alpha<0) {
    alpha = 0;
  } else if (alpha>1) {
    alpha = 1;
  }

  for (int j=0; j<3; j++) {
    rgb[j] = (mjtByte)(255*(alpha*rgb1[j] + (1-alpha)*rgb2[j]));
  }
}



// make checker pattern for one side
static void checker(unsigned char* rgb, const unsigned char* RGB1, const unsigned char* RGB2,
                    int width, int height) {
  for (int r=0; r<height/2; r++) {
    for (int c=0; c<width/2; c++) {
      memcpy(rgb+3*(r*width+c), RGB1, 3);
    }
  }
  for (int r=height/2; r<height; r++) {
    for (int c=width/2; c<width; c++) {
      memcpy(rgb+3*(r*width+c), RGB1, 3);
    }
  }
  for (int r=0; r<height/2; r++) {
    for (int c=width/2; c<width; c++) {
      memcpy(rgb+3*(r*width+c), RGB2, 3);
    }
  }
  for (int r=height/2; r<height; r++) {
    for (int c=0; c<width/2; c++) {
      memcpy(rgb+3*(r*width+c), RGB2, 3);
    }
  }
}



// make builtin: 2D
void mjCTexture::Builtin2D(void) {
  unsigned char RGB1[3], RGB2[3], RGBm[3];
  // convert fixed colors
  for (int j=0; j<3; j++) {
    RGB1[j] = (mjtByte)(255*rgb1[j]);
    RGB2[j] = (mjtByte)(255*rgb2[j]);
    RGBm[j] = (mjtByte)(255*markrgb[j]);
  }

  //------------------ face

  // gradient
  if (builtin==mjBUILTIN_GRADIENT) {
    for (int r=0; r<height; r++) {
      for (int c=0; c<width; c++) {
        // compute normalized coordinates and radius
        double x = 2*c/((double)(width-1)) - 1;
        double y = 1 - 2*r/((double)(height-1));
        double pos = 2*sqrt(x*x+y*y) - 1;

        // interpolate through sigmoid
        interp(rgb + 3*(r*width+c), rgb2, rgb1, pos);
      }
    }
  }

  // checker
  else if (builtin==mjBUILTIN_CHECKER) {
    checker(rgb, RGB1, RGB2, width, height);
  }

  // flat
  else if (builtin==mjBUILTIN_FLAT) {
    for (int r=0; r<height; r++) {
      for (int c=0; c<width; c++) {
        memcpy(rgb+3*(r*width+c), RGB1, 3);
      }
    }
  }

  //------------------ marks

  // edge
  if (mark==mjMARK_EDGE) {
    for (int r=0; r<height; r++) {
      memcpy(rgb+3*(r*width+0), RGBm, 3);
      memcpy(rgb+3*(r*width+width-1), RGBm, 3);
    }
    for (int c=0; c<width; c++) {
      memcpy(rgb+3*(0*width+c), RGBm, 3);
      memcpy(rgb+3*((height-1)*width+c), RGBm, 3);
    }
  }

  // cross
  else if (mark==mjMARK_CROSS) {
    for (int r=0; r<height; r++) {
      memcpy(rgb+3*(r*width+width/2), RGBm, 3);
    }
    for (int c=0; c<width; c++) {
      memcpy(rgb+3*(height/2*width+c), RGBm, 3);
    }
  }

  // random dots
  else if (mark==mjMARK_RANDOM && random>0) {
    randomdot(rgb, markrgb, width, height, random);
  }
}



// make builtin: Cube
void mjCTexture::BuiltinCube(void) {
  unsigned char RGB1[3], RGB2[3], RGBm[3], RGBi[3];

  // convert fixed colors
  for (int j=0; j<3; j++) {
    RGB1[j] = (mjtByte)(255*rgb1[j]);
    RGB2[j] = (mjtByte)(255*rgb2[j]);
    RGBm[j] = (mjtByte)(255*markrgb[j]);
  }

  //------------------ faces

  // gradient
  if (builtin==mjBUILTIN_GRADIENT) {
    for (int r=0; r<width; r++) {
      for (int c=0; c<width; c++) {
        // compute normalized pixel coordinates
        double x = 2*c/((double)(width-1)) - 1;
        double y = 1 - 2*r/((double)(width-1));

        // compute normalized elevation for sides and up/down
        double elside = asin(y/sqrt(1+x*x+y*y)) / (0.5*mjPI);
        double elup = 1 - acos(1.0/sqrt(1+x*x+y*y)) / (0.5*mjPI);

        // set sides
        interp(RGBi, rgb1, rgb2, elside);
        memcpy(rgb+0*3*width*width+3*(r*width+c), RGBi, 3);     // 0: right
        memcpy(rgb+1*3*width*width+3*(r*width+c), RGBi, 3);     // 1: left
        memcpy(rgb+4*3*width*width+3*(r*width+c), RGBi, 3);     // 4: front
        memcpy(rgb+5*3*width*width+3*(r*width+c), RGBi, 3);     // 5: back

        // set up and down
        interp(rgb+2*3*width*width+3*(r*width+c), rgb1, rgb2, elup);    // 2: up
        interp(rgb+3*3*width*width+3*(r*width+c), rgb1, rgb2, -elup);   // 3: down
      }
    }
  }

  // checker
  else if (builtin==mjBUILTIN_CHECKER) {
    checker(rgb+0*3*width*width, RGB1, RGB2, width, width);
    checker(rgb+1*3*width*width, RGB1, RGB2, width, width);
    checker(rgb+2*3*width*width, RGB1, RGB2, width, width);
    checker(rgb+3*3*width*width, RGB1, RGB2, width, width);
    checker(rgb+4*3*width*width, RGB2, RGB1, width, width);
    checker(rgb+5*3*width*width, RGB2, RGB1, width, width);
  }

  // flat
  else if (builtin==mjBUILTIN_FLAT) {
    for (int r=0; r<width; r++) {
      for (int c=0; c<width; c++) {
        // set sides and up
        memcpy(rgb+0*3*width*width+3*(r*width+c), RGB1, 3);
        memcpy(rgb+1*3*width*width+3*(r*width+c), RGB1, 3);
        memcpy(rgb+2*3*width*width+3*(r*width+c), RGB1, 3);
        memcpy(rgb+4*3*width*width+3*(r*width+c), RGB1, 3);
        memcpy(rgb+5*3*width*width+3*(r*width+c), RGB1, 3);

        // set down
        memcpy(rgb+3*3*width*width+3*(r*width+c), RGB2, 3);
      }
    }
  }

  //------------------ marks

  // edge
  if (mark==mjMARK_EDGE) {
    for (int j=0; j<6; j++) {
      for (int r=0; r<width; r++) {
        memcpy(rgb+j*3*width*width+3*(r*width+0), RGBm, 3);
        memcpy(rgb+j*3*width*width+3*(r*width+width-1), RGBm, 3);
      }
      for (int c=0; c<width; c++) {
        memcpy(rgb+j*3*width*width+3*(0*width+c), RGBm, 3);
        memcpy(rgb+j*3*width*width+3*((width-1)*width+c), RGBm, 3);
      }
    }
  }

  // cross
  else if (mark==mjMARK_CROSS) {
    for (int j=0; j<6; j++) {
      for (int r=0; r<width; r++) {
        memcpy(rgb+j*3*width*width+3*(r*width+width/2), RGBm, 3);
      }
      for (int c=0; c<width; c++) {
        memcpy(rgb+j*3*width*width+3*(width/2*width+c), RGBm, 3);
      }
    }
  }

  // random dots
  else if (mark==mjMARK_RANDOM && random>0) {
    randomdot(rgb, markrgb, width, height, random);
  }
}



// load PNG file
void mjCTexture::LoadPNG(mjResource* resource,
                         std::vector<unsigned char>& image,
                         unsigned int& w, unsigned int& h) {
  const void* inbuffer = 0;
  int inbuffer_sz = mju_readResource(resource, &inbuffer);

  // still not found
  if (inbuffer_sz < 1) {
    throw mjCError(this, "could not read PNG texture file '%s'", resource->name);
  } else if (!inbuffer_sz) {
    throw mjCError(this, "PNG texture file '%s' is empty", resource->name);
  }


  // load PNG from file or memory
  unsigned int err = lodepng::decode(image, w, h, (const unsigned char*) inbuffer, inbuffer_sz, LCT_RGB, 8);

  // check
  if (err) {
    throw mjCError(this,
                   "PNG file load error '%s' in texture id = %d", lodepng_error_text(err), id);
  }
  if (w<1 || h<1) {
    throw mjCError(this, "Empty PNG file in texture '%s' (id %d)", resource->name, id);
  }
}



// load custom file
void mjCTexture::LoadCustom(mjResource* resource,
                            std::vector<unsigned char>& image,
                            unsigned int& w, unsigned int& h) {
  const void* buffer = 0;
  int buffer_sz = mju_readResource(resource, &buffer);

  // still not found
  if (buffer_sz < 0) {
    throw mjCError(this, "could not read texture file '%s'", resource->name);
  } else if (!buffer_sz) {
    throw mjCError(this, "texture file is empty: '%s'", resource->name);
  }


  // read dimensions
  int* pint = (int*)buffer;
  w = pint[0];
  h = pint[1];

  // check dimensions
  if (w<1 || h<1) {
    throw mjCError(this, "Non-PNG texture, assuming custom binary file format,\n"
                         "non-positive texture dimensions in file '%s'", resource->name);
  }

  // check buffer size
  if (buffer_sz != 2*sizeof(int) + w*h*3*sizeof(char)) {
    throw mjCError(this, "Non-PNG texture, assuming custom binary file format,\n"
                         "unexpected file size in file '%s'", resource->name);
  }

  // allocate and copy
  image.resize(w*h*3);
  memcpy(image.data(), (void*)(pint+2), w*h*3*sizeof(char));
}



// load from PNG or custom file, flip if specified
void mjCTexture::LoadFlip(string filename, const mjVFS* vfs,
                          std::vector<unsigned char>& image,
                          unsigned int& w, unsigned int& h) {
  std::string asset_type = GetAssetContentType(filename, content_type_);

  // fallback to custom
  if (asset_type.empty()) {
    asset_type = "image/vnd.mujoco.texture";
  }

  if (asset_type != "image/png" && asset_type != "image/vnd.mujoco.texture") {
    throw mjCError(this, "unsupported content type: '%s'", asset_type.c_str());
  }

  mjResource* resource = LoadResource(filename, vfs);

  try {
    if (asset_type == "image/png") {
      LoadPNG(resource, image, w, h);
    } else {
      LoadCustom(resource, image, w, h);
    }
    mju_closeResource(resource);
  } catch(mjCError err) {
    mju_closeResource(resource);
    throw err;
  }

  // horizontal flip
  if (hflip) {
    for (int r=0; r<h; r++) {
      for (int c=0; c<w/2; c++) {
        int c1 = w-1-c;
        unsigned char tmp[3] = {
          image[3*(r*w+c)],
          image[3*(r*w+c)+1],
          image[3*(r*w+c)+2]
        };

        image[3*(r*w+c)]   = image[3*(r*w+c1)];
        image[3*(r*w+c)+1] = image[3*(r*w+c1)+1];
        image[3*(r*w+c)+2] = image[3*(r*w+c1)+2];

        image[3*(r*w+c1)]   = tmp[0];
        image[3*(r*w+c1)+1] = tmp[1];
        image[3*(r*w+c1)+2] = tmp[2];
      }
    }
  }

  // vertical flip
  if (vflip) {
    for (int r=0; r<h/2; r++) {
      for (int c=0; c<w; c++) {
        int r1 = h-1-r;
        unsigned char tmp[3] = {
          image[3*(r*w+c)],
          image[3*(r*w+c)+1],
          image[3*(r*w+c)+2]
        };

        image[3*(r*w+c)]   = image[3*(r1*w+c)];
        image[3*(r*w+c)+1] = image[3*(r1*w+c)+1];
        image[3*(r*w+c)+2] = image[3*(r1*w+c)+2];

        image[3*(r1*w+c)]   = tmp[0];
        image[3*(r1*w+c)+1] = tmp[1];
        image[3*(r1*w+c)+2] = tmp[2];
      }
    }
  }
}



// load 2D
void mjCTexture::Load2D(string filename, const mjVFS* vfs) {
  // load PNG or custom
  unsigned int w, h;
  std::vector<unsigned char> image;
  LoadFlip(filename, vfs, image, w, h);

  // assign size
  width = w;
  height = h;

  // allocate and copy data
  rgb = (mjtByte*) mju_malloc(3*width*height);
  if (!rgb) {
    throw mjCError(this, "Could not allocate memory for texture '%s' (id %d)",
                   (const char*)file_.c_str(), id);
  }
  memcpy(rgb, image.data(), 3*width*height);
  image.clear();
}



// load cube or skybox from single file (repeated or grid)
void mjCTexture::LoadCubeSingle(string filename, const mjVFS* vfs) {
  // check gridsize
  if (gridsize[0]<1 || gridsize[1]<1 || gridsize[0]*gridsize[1]>12) {
    throw mjCError(this,
                   "gridsize must be non-zero and no more than 12 squares in texture '%s' (id %d)",
                   (const char*)name.c_str(), id);
  }

  // load PNG or custom
  unsigned int w, h;
  std::vector<unsigned char> image;
  LoadFlip(filename, vfs, image, w, h);

  // check gridsize for compatibility
  if (w/gridsize[1]!=h/gridsize[0] || (w%gridsize[1]) || (h%gridsize[0])) {
    throw mjCError(this,
                   "PNG size must be integer multiple of gridsize in texture '%s' (id %d)",
                   (const char*)file_.c_str(), id);
  }

  // assign size: repeated or full
  if (gridsize[0]==1 && gridsize[1]==1) {
    width = height = w;
  } else {
    width = w/gridsize[1];
    height = 6*width;
  }

  // allocate data
  rgb = (mjtByte*) mju_malloc(3*width*height);
  if (!rgb) {
    throw mjCError(this,
                   "Could not allocate memory for texture '%s' (id %d)",
                   (const char*)file_.c_str(), id);
  }

  // copy: repeated
  if (gridsize[0]==1 && gridsize[1]==1) {
    memcpy(rgb, image.data(), 3*width*width);
  }

  // copy: grid
  else {
    // keep track of which faces were defined
    int loaded[6] = {0, 0, 0, 0, 0, 0};

    // process grid
    for (int k=0; k<gridsize[0]*gridsize[1]; k++) {
      // decode face symbol
      int i = -1;
      if (gridlayout[k]=='R') {
        i = 0;
      } else if (gridlayout[k]=='L') {
        i = 1;
      } else if (gridlayout[k]=='U') {
        i = 2;
      } else if (gridlayout[k]=='D') {
        i = 3;
      } else if (gridlayout[k]=='F') {
        i = 4;
      } else if (gridlayout[k]=='B') {
        i = 5;
      } else if (gridlayout[k]!='.')
        throw mjCError(this, "gridlayout symbol is not among '.RLUDFB' in texture '%s' (id %d)",
                       (const char*)file_.c_str(), id);

      // load if specified
      if (i>=0) {
        // extract sub-image
        int rstart = width*(k/gridsize[1]);
        int cstart = width*(k%gridsize[1]);
        for (int j=0; j<width; j++) {
          memcpy(rgb+i*3*width*width+j*3*width, image.data()+(j+rstart)*3*w+3*cstart, 3*width);
        }

        // mark as defined
        loaded[i] = 1;
      }
    }

    // set undefined faces to rgb1
    for (int i=0; i<6; i++) {
      if (!loaded[i]) {
        for (int k=0; k<width; k++) {
          for (int s=0; s<width; s++) {
            for (int j=0; j<3; j++) {
              rgb[i*3*width*width + 3*(k*width+s) + j] = (mjtByte)(255*rgb1[j]);
            }
          }
        }
      }
    }
  }

  image.clear();
}



// load cube or skybox from separate file
void mjCTexture::LoadCubeSeparate(const mjVFS* vfs) {
  // keep track of which faces were defined
  int loaded[6] = {0, 0, 0, 0, 0, 0};

  // process nonempty files
  for (int i=0; i<6; i++) {
    if (!cubefiles_[i].empty()) {
      // remove path from file if necessary
      if (model->strippath) {
        cubefiles_[i] = mjuu_strippath(cubefiles_[i]);
      }

      // make filename
      string filename = mjuu_makefullname(model->modelfiledir_, model->texturedir_, cubefiles_[i]);

      // load PNG or custom
      unsigned int w, h;
      std::vector<unsigned char> image;
      LoadFlip(filename, vfs, image, w, h);

      // PNG must be square
      if (w!=h) {
        throw mjCError(this,
                       "Non-square PNG file '%s' in cube or skybox id %d",
                       (const char*)cubefiles_[i].c_str(), id);
      }

      // first file: set size and allocate data
      if (!rgb) {
        width = w;
        height = 6*width;
        rgb = (mjtByte*) mju_malloc(3*width*height);
        if (!rgb) {
          throw mjCError(this,
                         "Could not allocate memory for texture '%s' (id %d)",
                         (const char*)name.c_str(), id);
        }
      }

      // otherwise check size
      else if (width!=w) {
        throw mjCError(this,
                       "PNG file '%s' has incompatible size in texture id %d",
                       (const char*)cubefiles_[i].c_str(), id);
      }

      // copy data
      memcpy(rgb+i*3*width*width, image.data(), 3*width*width);
      image.clear();

      // mark as defined
      loaded[i] = 1;
    }
  }

  // set undefined faces to rgb1
  for (int i=0; i<6; i++) {
    if (!loaded[i]) {
      for (int k=0; k<width; k++) {
        for (int s=0; s<width; s++) {
          for (int j=0; j<3; j++) {
            rgb[i*3*width*width + 3*(k*width+s) + j] = (mjtByte)(255*rgb1[j]);
          }
        }
      }
    }
  }
}



// compiler
void mjCTexture::Compile(const mjVFS* vfs) {
  CopyFromSpec();

  // builtin
  if (builtin!=mjBUILTIN_NONE) {
    // check size
    if (width<1 || height<1) {
      throw mjCError(this,
                     "Invalid width or height of builtin texture '%s' (id %d)",
                     (const char*)name.c_str(), id);
    }

    // adjust height of cube texture
    if (type!=mjTEXTURE_2D) {
      height = 6*width;
    }

    // allocate data
    rgb = (mjtByte*) mju_malloc(3*width*height);
    if (!rgb) {
      throw mjCError(this,
                     "Could not allocate memory for texture '%s' (id %d)",
                     (const char*)name.c_str(), id);
    }

    // dispatch
    if (type==mjTEXTURE_2D) {
      Builtin2D();
    } else {
      BuiltinCube();
    }
  }

  // single file
  else if (!file_.empty()) {
    // remove path from file if necessary
    if (model->strippath) {
      file_ = mjuu_strippath(file_);
    }

    // make filename
    string filename = mjuu_makefullname(model->modelfiledir_, model->texturedir_, file_);

    // dispatch
    if (type==mjTEXTURE_2D) {
      Load2D(filename, vfs);
    } else {
      LoadCubeSingle(filename, vfs);
    }
  }

  // separate files
  else {
    // 2D not allowed
    if (type==mjTEXTURE_2D) {
      throw mjCError(this,
                     "Cannot load 2D texture from separate files, texture '%s' (id %d)",
                     (const char*)name.c_str(), id);
    }

    // at least one cubefile must be defined
    bool defined = false;
    for (int i=0; i<6; i++) {
      if (!cubefiles_[i].empty()) {
        defined = true;
        break;
      }
    }
    if (!defined) {
      throw mjCError(this,
                     "No cubefiles_ defined in cube or skybox texture '%s' (id %d)",
                     (const char*)name.c_str(), id);
    }

    // only cube and skybox
    LoadCubeSeparate(vfs);
  }

  // make sure someone allocated data; SHOULD NOT OCCUR
  if (!rgb) {
    throw mjCError(this,
                   "texture '%s' (id %d) was not specified", (const char*)name.c_str(), id);
  }
}



//------------------ class mjCMaterial implementation ----------------------------------------------

// initialize defaults
mjCMaterial::mjCMaterial(mjCModel* _model, mjCDef* _def) {
  mjm_defaultMaterial(spec);

  // clear internal
  spec_texture_.clear();
  texid = -1;

  // reset to default if given
  if (_def) {
    *this = _def->material;
  }

  // set model, def
  model = _model;
  def = (_def ? _def : (_model ? _model->defaults[0] : 0));

  // point to local
  PointToLocal();

  // in case this camera is not compiled
  CopyFromSpec();
}



mjCMaterial::mjCMaterial(const mjCMaterial& other) {
  *this = other;
}



mjCMaterial& mjCMaterial::operator=(const mjCMaterial& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjCMaterial_*>(this) = static_cast<const mjCMaterial_&>(other);
    *static_cast<mjmMaterial*>(this) = static_cast<const mjmMaterial&>(other);
  }
  PointToLocal();
  return *this;
}



void mjCMaterial::PointToLocal() {
  spec.element = (mjElement)this;
  spec.name = (mjString)&name;
  spec.classname = (mjString)&classname;
  spec.texture = (mjString)&spec_texture_;
  spec.info = (mjString)&info;
}



void mjCMaterial::CopyFromSpec() {
  *static_cast<mjmMaterial*>(this) = spec;
  texture_ = spec_texture_;
  texture = (mjString)&texture_;
}



// compiler
void mjCMaterial::Compile(void) {
  CopyFromSpec();
}



//------------------ class mjCPair implementation --------------------------------------------------

// constructor
mjCPair::mjCPair(mjCModel* _model, mjCDef* _def) {
  mjm_defaultPair(spec);

  // set defaults
  spec_geomname1_.clear();
  spec_geomname2_.clear();

  // clear internal variables
  geom1 = nullptr;
  geom2 = nullptr;
  signature = -1;

  // reset to default if given
  if (_def) {
    *this = _def->pair;
  }

  // set model, def
  model = _model;
  def = (_def ? _def : (_model ? _model->defaults[0] : 0));

  // point to local
  PointToLocal();

  // in case this camera is not compiled
  CopyFromSpec();
}



mjCPair::mjCPair(const mjCPair& other) {
  *this = other;
}



mjCPair& mjCPair::operator=(const mjCPair& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjCPair_*>(this) = static_cast<const mjCPair_&>(other);
    *static_cast<mjmPair*>(this) = static_cast<const mjmPair&>(other);
    this->geom1 = other.geom1;
    this->geom2 = other.geom2;
  }
  PointToLocal();
  return *this;
}



void mjCPair::PointToLocal() {
  spec.element = (mjElement)this;
  spec.name = (mjString)&name;
  spec.classname = (mjString)&classname;
  spec.geomname1 = (mjString)&spec_geomname1_;
  spec.geomname2 = (mjString)&spec_geomname2_;
  spec.info = (mjString)&info;
}



void mjCPair::CopyFromSpec() {
  *static_cast<mjmPair*>(this) = spec;
  geomname1_ = spec_geomname1_;
  geomname2_ = spec_geomname2_;
  geomname1 = (mjString)&geomname1_;
  geomname2 = (mjString)&geomname2_;
}



// compiler
void mjCPair::Compile(void) {
  CopyFromSpec();

  // check condim
  if (condim!=1 && condim!=3 && condim!=4 && condim!=6) {
    throw mjCError(this, "invalid condim in collision %d", "", id);
  }

  // find geom 1
  geom1 = (mjCGeom*)model->FindObject(mjOBJ_GEOM, geomname1_);
  if (!geom1) {
    throw mjCError(this, "geom '%s' not found in collision %d", geomname1_.c_str(), id);
  }

  // find geom 2
  geom2 = (mjCGeom*)model->FindObject(mjOBJ_GEOM, geomname2_);
  if (!geom2) {
    throw mjCError(this, "geom '%s' not found in collision %d", geomname2_.c_str(), id);
  }

  // mark geoms as not visual
  geom1->SetNotVisual();
  geom2->SetNotVisual();

  // swap if body1 > body2
  if (geom1->body->id > geom2->body->id) {
    string nametmp = geomname1_;
    geomname1_ = geomname2_;
    geomname2_ = nametmp;

    mjCGeom* geomtmp = geom1;
    geom1 = geom2;
    geom2 = geomtmp;
  }

  // get geom ids and body signature
  signature = ((geom1->body->id)<<16) + geom2->body->id;

  // set undefined margin: max
  if (!mjuu_defined(margin)) {
    margin = mjMAX(geom1->margin, geom2->margin);
  }

  // set undefined gap: max
  if (!mjuu_defined(gap)) {
    gap = mjMAX(geom1->gap, geom2->gap);
  }

  // set undefined condim, friction, solref, solimp: different priority
  if (geom1->priority!=geom2->priority) {
    mjCGeom* pgh = (geom1->priority>geom2->priority ? geom1 : geom2);

    // condim
    if (condim<0) {
      condim = pgh->condim;
    }

    // friction
    if (!mjuu_defined(friction[0])) {
      friction[0] = friction[1] = pgh->friction[0];
      friction[2] =               pgh->friction[1];
      friction[3] = friction[4] = pgh->friction[2];
    }

    // reference
    if (!mjuu_defined(solref[0])) {
      for (int i=0; i<mjNREF; i++) {
        solref[i] = pgh->solref[i];
      }
    }

    // impedance
    if (!mjuu_defined(solimp[0])) {
      for (int i=0; i<mjNIMP; i++) {
        solimp[i] = pgh->solimp[i];
      }
    }
  }

  // set undefined condim, friction, solref, solimp: same priority
  else {
    // condim: max
    if (condim<0) {
      condim = mjMAX(geom1->condim, geom2->condim);
    }

    // friction: max
    if (!mjuu_defined(friction[0])) {
      friction[0] = friction[1] = mju_max(geom1->friction[0], geom2->friction[0]);
      friction[2] =               mju_max(geom1->friction[1], geom2->friction[1]);
      friction[3] = friction[4] = mju_max(geom1->friction[2], geom2->friction[2]);
    }

    // solver mix factor
    double mix;
    if (geom1->solmix>=mjMINVAL && geom2->solmix>=mjMINVAL) {
      mix = geom1->solmix / (geom1->solmix + geom2->solmix);
    } else if (geom1->solmix<mjMINVAL && geom2->solmix<mjMINVAL) {
      mix = 0.5;
    } else if (geom1->solmix<mjMINVAL) {
      mix = 0.0;
    } else {
      mix = 1.0;
    }

    // reference
    if (!mjuu_defined(solref[0])) {
      // standard: mix
      if (solref[0]>0) {
        for (int i=0; i<mjNREF; i++) {
          solref[i] = mix*geom1->solref[i] + (1-mix)*geom2->solref[i];
        }
      }

      // direct: min
      else {
        for (int i=0; i<mjNREF; i++) {
          solref[i] = mju_min(geom1->solref[i], geom2->solref[i]);
        }
      }
    }

    // impedance
    if (!mjuu_defined(solimp[0])) {
      for (int i=0; i<mjNIMP; i++) {
        solimp[i] = mix*geom1->solimp[i] + (1-mix)*geom2->solimp[i];
      }
    }
  }
}



//------------------ class mjCBodyPair implementation ----------------------------------------------

// constructor
mjCBodyPair::mjCBodyPair(mjCModel* _model) {
  // set model pointer
  model = _model;

  // set defaults
  spec_bodyname1_.clear();
  spec_bodyname2_.clear();

  // clear internal variables
  body1 = body2 = signature = -1;

  PointToLocal();
  CopyFromSpec();
}



void mjCBodyPair::PointToLocal() {
  spec.element = (mjElement)this;
  spec.name = (mjString)&name;
  spec.bodyname1 = (mjString)&spec_bodyname1_;
  spec.bodyname2 = (mjString)&spec_bodyname2_;
  spec.info = (mjString)&info;
}



void mjCBodyPair::CopyFromSpec() {
  *static_cast<mjmExclude*>(this) = spec;
  bodyname1_ = spec_bodyname1_;
  bodyname2_ = spec_bodyname2_;
  bodyname1 = (mjString)&bodyname1_;
  bodyname2 = (mjString)&bodyname2_;
}



// compiler
void mjCBodyPair::Compile(void) {
  CopyFromSpec();

  // find body 1
  mjCBody* pb1 = (mjCBody*)model->FindObject(mjOBJ_BODY, bodyname1_);
  if (!pb1) {
    throw mjCError(this, "body '%s' not found in bodypair %d", bodyname1_.c_str(), id);
  }

  // find body 2
  mjCBody* pb2 = (mjCBody*)model->FindObject(mjOBJ_BODY, bodyname2_);
  if (!pb2) {
    throw mjCError(this, "body '%s' not found in bodypair %d", bodyname2_.c_str(), id);
  }

  // swap if body1 > body2
  if (pb1->id > pb2->id) {
    string nametmp = bodyname1_;
    bodyname1_ = bodyname2_;
    bodyname2_ = nametmp;

    mjCBody* bodytmp = pb1;
    pb1 = pb2;
    pb2 = bodytmp;
  }

  // get body ids and body signature
  body1 = pb1->id;
  body2 = pb2->id;
  signature = (body1<<16) + body2;
}



//------------------ class mjCEquality implementation ----------------------------------------------

// initialize default constraint
mjCEquality::mjCEquality(mjCModel* _model, mjCDef* _def) {
  mjm_defaultEquality(spec);

  // clear internal variables
  spec_name1_.clear();
  spec_name2_.clear();
  obj1id = obj2id = -1;

  // reset to default if given
  if (_def) {
    *this = _def->equality;
  }

  // set model, def
  model = _model;
  def = (_def ? _def : (_model ? _model->defaults[0] : 0));

  // point to local
  PointToLocal();

  // in case this camera is not compiled
  CopyFromSpec();
}



mjCEquality::mjCEquality(const mjCEquality& other) {
  *this = other;
}



mjCEquality& mjCEquality::operator=(const mjCEquality& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjCEquality_*>(this) = static_cast<const mjCEquality_&>(other);
    *static_cast<mjmEquality*>(this) = static_cast<const mjmEquality&>(other);
  }
  PointToLocal();
  return *this;
}



void mjCEquality::PointToLocal() {
  spec.element = (mjElement)this;
  spec.name = (mjString)&name;
  spec.classname = (mjString)&classname;
  spec.name1 = (mjString)&spec_name1_;
  spec.name2 = (mjString)&spec_name2_;
  spec.info = (mjString)&info;
}


void mjCEquality::CopyFromSpec() {
  *static_cast<mjmEquality*>(this) = spec;
  name1_ = spec_name1_;
  name2_ = spec_name2_;
  name1 = (mjString)&name1_;
  name2 = (mjString)&name2_;
}


// compiler
void mjCEquality::Compile(void) {
  CopyFromSpec();

  mjtObj objtype;
  mjCBase *px1, *px2;
  mjtJoint jt1, jt2;

  // determine object type
  if (type==mjEQ_CONNECT || type==mjEQ_WELD) {
    objtype = mjOBJ_BODY;
  } else if (type==mjEQ_JOINT) {
    objtype = mjOBJ_JOINT;
  } else if (type==mjEQ_TENDON) {
    objtype = mjOBJ_TENDON;
  } else if (type==mjEQ_FLEX) {
    objtype = mjOBJ_FLEX;
  } else {
    throw mjCError(this, "invalid type in equality constraint '%s' (id = %d)'", name.c_str(), id);
  }

  // find object 1, get id
  px1 = model->FindObject(objtype, name1_);
  if (!px1) {
    throw mjCError(this, "unknown element '%s' in equality constraint %d", name1_.c_str(), id);
  }
  obj1id = px1->id;

  // find object 2, get id
  if (!name2_.empty()) {
    px2 = model->FindObject(objtype, name2_);
    if (!px2) {
      throw mjCError(this, "unknown element '%s' in equality constraint %d", name2_.c_str(), id);
    }
    obj2id = px2->id;
  }

  // object 2 unspecified: set to -1
  else {
    if (objtype==mjOBJ_GEOM) {
      throw mjCError(this, "both geom are required in equality constraint '%s' (id = %d)",
                     name.c_str(), id);
    } else {
      obj2id = -1;
      px2 = 0;
    }
  }

  // set missing body = world
  if (objtype==mjOBJ_BODY && obj2id==-1) {
    obj2id = 0;
  }

  // make sure flex is not rigid
  if (type==mjEQ_FLEX && model->flexes[obj1id]->rigid) {
    throw mjCError(this, "rigid flex '%s' in equality constraint %d", name1_.c_str(), id);
  }

  // make sure the two objects are different
  if (obj1id==obj2id) {
    throw mjCError(this, "element '%s' is repeated in equality constraint %d", name1_.c_str(), id);
  }

  // make sure joints are scalar
  if (type==mjEQ_JOINT) {
    jt1 = ((mjCJoint*)px1)->type;
    jt2 = (px2 ? ((mjCJoint*)px2)->type : mjJNT_HINGE);
    if ((jt1!=mjJNT_HINGE && jt1!=mjJNT_SLIDE) ||
        (jt2!=mjJNT_HINGE && jt2!=mjJNT_SLIDE)) {
      throw mjCError(this, "only HINGE and SLIDE joint allowed in constraint '%s' (id = %d)",
                     name.c_str(), id);
    }
  }
}



//------------------ class mjCTendon implementation ------------------------------------------------

// constructor
mjCTendon::mjCTendon(mjCModel* _model, mjCDef* _def) {
  mjm_defaultTendon(spec);

  // clear internal variables
  spec_material_.clear();
  spec_userdata_.clear();
  path.clear();
  matid = -1;

  // reset to default if given
  if (_def) {
    *this = _def->tendon;
  }

  // set model, def
  model = _model;
  def = (_def ? _def : (_model ? _model->defaults[0] : 0));

  // point to local
  PointToLocal();

  // in case this camera is not compiled
  CopyFromSpec();
}



mjCTendon::mjCTendon(const mjCTendon& other) {
  *this = other;
}



mjCTendon& mjCTendon::operator=(const mjCTendon& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjmTendon*>(this) = static_cast<const mjmTendon&>(other);
    *static_cast<mjmTendon*>(this) = static_cast<const mjmTendon&>(other);
  }
  PointToLocal();
  return *this;
}



bool mjCTendon::is_limited() const { return islimited(limited, range); }


void mjCTendon::PointToLocal() {
  spec.element = (mjElement)this;
  spec.name = (mjString)&name;
  spec.classname = (mjString)&classname;
  spec.material = (mjString)&spec_material_;
  spec.userdata = (mjDoubleVec)&spec_userdata_;
  spec.info = (mjString)&info;
}



void mjCTendon::CopyFromSpec() {
  *static_cast<mjmTendon*>(this) = spec;
  material_ = spec_material_;
  userdata_ = spec_userdata_;
  material = (mjString)&material_;
  userdata = (mjDoubleVec)&userdata_;

  // clear precompiled
  for (int i=0; i<path.size(); i++) {
    if (path[i]->type==mjWRAP_CYLINDER) {
      path[i]->type = mjWRAP_SPHERE;
    }
  }
}



// desctructor
mjCTendon::~mjCTendon() {
  // delete objects allocated here
  for (unsigned int i=0; i<path.size(); i++) {
    delete path[i];
  }

  path.clear();
}



// add site as wrap object
void mjCTendon::WrapSite(string name, std::string_view info) {
  // create wrap object
  mjCWrap* wrap = new mjCWrap(model, this);
  wrap->info = info;

  // set parameters, add to path
  wrap->type = mjWRAP_SITE;
  wrap->name = name;
  wrap->id = (int)path.size();
  path.push_back(wrap);
}



// add geom (with side site) as wrap object
void mjCTendon::WrapGeom(string name, string sidesite, std::string_view info) {
  // create wrap object
  mjCWrap* wrap = new mjCWrap(model, this);
  wrap->info = info;

  // set parameters, add to path
  wrap->type = mjWRAP_SPHERE;         // replace with cylinder later if needed
  wrap->name = name;
  wrap->sidesite = sidesite;
  wrap->id = (int)path.size();
  path.push_back(wrap);
}



// add joint as wrap object
void mjCTendon::WrapJoint(string name, double coef, std::string_view info) {
  // create wrap object
  mjCWrap* wrap = new mjCWrap(model, this);
  wrap->info = info;

  // set parameters, add to path
  wrap->type = mjWRAP_JOINT;
  wrap->name = name;
  wrap->prm = coef;
  wrap->id = (int)path.size();
  path.push_back(wrap);
}



// add pulley
void mjCTendon::WrapPulley(double divisor, std::string_view info) {
  // create wrap object
  mjCWrap* wrap = new mjCWrap(model, this);
  wrap->info = info;

  // set parameters, add to path
  wrap->type = mjWRAP_PULLEY;
  wrap->prm = divisor;
  wrap->id = (int)path.size();
  path.push_back(wrap);
}



// get number of wraps
int mjCTendon::NumWraps(void) {
  return (int)path.size();
}



// get pointer to specified wrap
mjCWrap* mjCTendon::GetWrap(int id) {
  if (id>=0 && id<(int)path.size()) {
    return path[id];
  } else {
    return 0;
  }
}



// compiler
void mjCTendon::Compile(void) {
  CopyFromSpec();

  // resize userdata
  if (userdata_.size() > model->nuser_tendon) {
    throw mjCError(this, "user has more values than nuser_tendon in tendon '%s' (id = %d)",
                   name.c_str(), id);
  }
  userdata_.resize(model->nuser_tendon);

  // check for empty path
  int sz = (int)path.size();
  if (!sz) {
    throw mjCError(this,
                   "tendon '%s' (id = %d): path cannot be empty",
                   name.c_str(), id);
  }

  // determine type
  bool spatial = (path[0]->type!=mjWRAP_JOINT);

  // require at least two objects in spatial path
  if (spatial && sz<2) {
    throw mjCError(this, "tendon '%s' (id = %d): spatial path must contain at least two objects",
                   name.c_str(), id);
  }

  // require positive width
  if (spatial && width<=0) {
    throw mjCError(this, "tendon '%s' (id = %d) must have positive width", name.c_str(), id);
  }

  // compile objects in path
  for (int i=0; i<sz; i++) {
    path[i]->Compile();
  }

  // check path
  for (int i=0; i<sz; i++) {
    // fixed
    if (!spatial) {
      // make sure all objects are joints
      if (path[i]->type!=mjWRAP_JOINT) {
        throw mjCError(this, "tendon '%s' (id = %d): spatial object found in fixed path at pos %d",
                       name.c_str(), id, i);
      }
    }

    // spatial path
    else {
      switch (path[i]->type) {
      case mjWRAP_PULLEY:
        // pulley should not follow other pulley
        if (i>0 && path[i-1]->type==mjWRAP_PULLEY) {
          throw mjCError(this, "tendon '%s' (id = %d): consequtive pulleys (pos %d)",
                         name.c_str(), id, i);
        }

        // pulley should not be last
        if (i==sz-1) {
          throw mjCError(this, "tendon '%s' (id = %d): path ends with pulley", name.c_str(), id);
        }
        break;

      case mjWRAP_SITE:
        // site needs a neighbor that is not a pulley
        if ((i==0 || path[i-1]->type==mjWRAP_PULLEY) &&
            (i==sz-1 || path[i+1]->type==mjWRAP_PULLEY)) {
          throw mjCError(this,
                         "tendon '%s' (id = %d): site %d needs a neighbor that is not a pulley",
                         name.c_str(), id, i);
        }

        // site cannot be repeated
        if (i<sz-1 && path[i+1]->type==mjWRAP_SITE && path[i]->obj->id==path[i+1]->obj->id) {
          throw mjCError(this,
                         "tendon '%s' (id = %d): site %d is repeated",
                         name.c_str(), id, i);
        }

        break;

      case mjWRAP_SPHERE:
      case mjWRAP_CYLINDER:
        // geom must be bracketed by sites
        if (i==0 || i==sz-1 || path[i-1]->type!=mjWRAP_SITE || path[i+1]->type!=mjWRAP_SITE) {
          throw mjCError(this,
                         "tendon '%s' (id = %d): geom at pos %d not bracketed by sites",
                         name.c_str(), id, i);
        }

        // mark geoms as non visual
        model->geoms[path[i]->obj->id]->SetNotVisual();
        break;

      case mjWRAP_JOINT:
        throw mjCError(this,
                       "tendon '%s (id = %d)': joint wrap found in spatial path at pos %d",
                       name.c_str(), id, i);

      default:
        throw mjCError(this,
                       "tendon '%s (id = %d)': invalid wrap object at pos %d",
                       name.c_str(), id, i);
      }
    }
  }

  // if limited is auto, set to 1 if range is specified, otherwise unlimited
  if (limited == mjLIMITED_AUTO) {
    bool hasrange = !(range[0]==0 && range[1]==0);
    checklimited(this, model->autolimits, "tendon", "", limited, hasrange);
  }

  // check limits
  if (range[0]>=range[1] && is_limited()) {
    throw mjCError(this, "invalid limits in tendon '%s (id = %d)'", name.c_str(), id);
  }

  // check springlength
  if (springlength[0] > springlength[1]) {
    throw mjCError(this, "invalid springlength in tendon '%s (id = %d)'", name.c_str(), id);
  }
}



//------------------ class mjCWrap implementation --------------------------------------------------

// constructor
mjCWrap::mjCWrap(mjCModel* _model, mjCTendon* _tendon) {
  // set model and tendon pointer
  model = _model;
  tendon = _tendon;

  // clear variables
  type = mjWRAP_NONE;
  obj = nullptr;
  sideid = -1;
  prm = 0;
  sidesite.clear();

  // point to local
  PointToLocal();
}



void mjCWrap::PointToLocal() {
  spec.element = (mjElement)this;
  spec.info = (mjString)&info;
}



// compiler
void mjCWrap::Compile(void) {
  mjCBase *pside;

  // handle wrap object types
  switch (type) {
  case mjWRAP_JOINT:                          // joint
    // find joint by name
    obj = model->FindObject(mjOBJ_JOINT, name);
    if (!obj) {
      throw mjCError(this,
                     "joint '%s' not found in tendon %d, wrap %d",
                     name.c_str(), tendon->id, id);
    }

    break;

  case mjWRAP_SPHERE:                         // geom (cylinder type set here)
    // find geom by name
    obj = model->FindObject(mjOBJ_GEOM, name);
    if (!obj) {
      throw mjCError(this,
                     "geom '%s' not found in tendon %d, wrap %d",
                     name.c_str(), tendon->id, id);
    }

    // set/check geom type
    if (((mjCGeom*)obj)->type == mjGEOM_CYLINDER) {
      type = mjWRAP_CYLINDER;
    } else if (((mjCGeom*)obj)->type != mjGEOM_SPHERE) {
      throw mjCError(this,
                     "geom '%s' in tendon %d, wrap %d is not sphere or cylinder",
                     name.c_str(), tendon->id, id);
    }

    // process side site
    if (!sidesite.empty()) {
      // find site by name
      pside = model->FindObject(mjOBJ_SITE, sidesite);
      if (!pside) {
        throw mjCError(this,
                       "side site '%s' not found in tendon %d, wrap %d",
                       sidesite.c_str(), tendon->id, id);
      }

      // save side site id
      sideid = pside->id;
    }
    break;

  case mjWRAP_PULLEY:                         // pulley
    // make sure divisor is non-negative
    if (prm<0) {
      throw mjCError(this,
                     "pulley has negative divisor in tendon %d, wrap %d",
                     0, tendon->id, id);
    }

    break;

  case mjWRAP_SITE:                           // site
    // find site by name
    obj = model->FindObject(mjOBJ_SITE, name);
    if (!obj) {
      throw mjCError(this, "site '%s' not found in wrap %d", name.c_str(), id);
    }
    break;

  default:                                    // SHOULD NOT OCCUR
    throw mjCError(this, "unknown wrap type in tendon %d, wrap %d", 0, tendon->id, id);
  }
}



//------------------ class mjCActuator implementation ----------------------------------------------

// initialize defaults
mjCActuator::mjCActuator(mjCModel* _model, mjCDef* _def) {
  mjm_defaultActuator(spec);

  // clear private variables
  spec_target_.clear();
  spec_slidersite_.clear();
  spec_refsite_.clear();
  spec_userdata_.clear();
  trnid[0] = trnid[1] = -1;

  // reset to default if given
  if (_def) {
    *this = _def->actuator;
  }

  // set model, def
  model = _model;
  def = (_def ? _def : (_model ? _model->defaults[0] : 0));

  // in case this actuator is not compiled
  CopyFromSpec();

  // point to local
  PointToLocal();
}



mjCActuator::mjCActuator(const mjCActuator& other) {
  *this = other;
}



mjCActuator& mjCActuator::operator=(const mjCActuator& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjCActuator_*>(this) = static_cast<const mjCActuator_&>(other);
    *static_cast<mjmActuator*>(this) = static_cast<const mjmActuator&>(other);
  }
  PointToLocal();
  return *this;
}



bool mjCActuator::is_ctrllimited() const { return islimited(ctrllimited, ctrlrange); }
bool mjCActuator::is_forcelimited() const { return islimited(forcelimited, forcerange); }
bool mjCActuator::is_actlimited() const { return islimited(actlimited, actrange); }



void mjCActuator::PointToLocal() {
  spec.element = (mjElement)this;
  spec.name = (mjString)&name;
  spec.classname = (mjString)&classname;
  spec.userdata = (mjDoubleVec)&spec_userdata_;
  spec.target = (mjString)&spec_target_;
  spec.refsite = (mjString)&spec_refsite_;
  spec.slidersite = (mjString)&spec_slidersite_;
  spec.plugin.name = (mjString)&plugin_name;
  spec.plugin.instance_name = (mjString)&plugin_instance_name;
  spec.info = (mjString)&info;
}



void mjCActuator::CopyFromSpec() {
  *static_cast<mjmActuator*>(this) = spec;
  userdata_ = spec_userdata_;
  target_ = spec_target_;
  refsite_ = spec_refsite_;
  slidersite_ = spec_slidersite_;
  userdata = (mjDoubleVec)&userdata_;
  target = (mjString)&target_;
  refsite = (mjString)&refsite_;
  slidersite = (mjString)&slidersite_;
  plugin.active = spec.plugin.active;
  plugin.instance = spec.plugin.instance;
  plugin.name = spec.plugin.name;
  plugin.instance_name = spec.plugin.instance_name;
}



// compiler
void mjCActuator::Compile(void) {
  CopyFromSpec();
  mjCJoint* pjnt;

  // resize userdata
  if (userdata_.size() > model->nuser_actuator) {
    throw mjCError(this, "user has more values than nuser_actuator in actuator '%s' (id = %d)",
                   name.c_str(), id);
  }
  userdata_.resize(model->nuser_actuator);

  // check for missing target name
  if (target_.empty()) {
    throw mjCError(this,
                   "missing transmission target for actuator '%s' (id = %d)", name.c_str(), id);
  }

  // find transmission target in object arrays
  mjCBase* ptarget = 0;
  switch (trntype) {
  case mjTRN_JOINT:
  case mjTRN_JOINTINPARENT:
    // get joint
    ptarget = model->FindObject(mjOBJ_JOINT, target_);
    if (!ptarget) {
      throw mjCError(this,
                     "unknown transmission target '%s' for actuator id = %d", target_.c_str(), id);
    }
    pjnt = (mjCJoint*) ptarget;

    // apply urdfeffort
    if (pjnt->spec.urdfeffort>0) {
      forcerange[0] = -pjnt->spec.urdfeffort;
      forcerange[1] = pjnt->spec.urdfeffort;
      forcelimited = mjLIMITED_TRUE;
    }
    break;

  case mjTRN_SLIDERCRANK:
    // get slidersite, copy in trnid[1]
    if (slidersite_.empty()) {
      throw mjCError(this, "missing base site for slider-crank '%s' (id = %d)", name.c_str(), id);
    }
    ptarget = model->FindObject(mjOBJ_SITE, slidersite_);
    if (!ptarget) {
      throw mjCError(this, "base site '%s' not found for actuator %d", slidersite_.c_str(), id);
    }
    trnid[1] = ptarget->id;

    // check cranklength
    if (cranklength<=0) {
      throw mjCError(this,
                     "crank length must be positive in actuator '%s' (id = %d)", name.c_str(), id);
    }

    // proceed with regular target
    ptarget = model->FindObject(mjOBJ_SITE, target_);
    break;

  case mjTRN_TENDON:
    // get tendon
    ptarget = model->FindObject(mjOBJ_TENDON, target_);
    break;

  case mjTRN_SITE:
    // get refsite, copy into trnid[1]
    if (!refsite_.empty()) {
      ptarget = model->FindObject(mjOBJ_SITE, refsite_);
      if (!ptarget) {
        throw mjCError(this, "reference site '%s' not found for actuator %d", refsite_.c_str(), id);
      }
      trnid[1] = ptarget->id;
    }

    // proceed with regular site target
    ptarget = model->FindObject(mjOBJ_SITE, target_);
    break;

  case mjTRN_BODY:
    // get body
    ptarget = model->FindObject(mjOBJ_BODY, target_);
    break;

  default:
    throw mjCError(this, "invalid transmission type in actuator '%s' (id = %d)", name.c_str(), id);
  }

  // assign and check
  if (!ptarget) {
    throw mjCError(this, "transmission target '%s' not found in actuator %d", target_.c_str(), id);
  } else {
    trnid[0] = ptarget->id;
  }

  // handle inheritrange
  if (gaintype == mjGAIN_FIXED && biastype == mjBIAS_AFFINE &&
      gainprm[0] == -biasprm[1] && inheritrange > 0) {
    // semantic of actuator is the same as transmission, inheritrange is applicable
    double* range;
    if (dyntype == mjDYN_NONE) {
      // position actuator
      range = ctrlrange;
    } else if (dyntype == mjDYN_INTEGRATOR) {
      // intvelocity actuator
      range = actrange;
    } else {
      throw mjCError(this, "inheritrange only available for position "
                     "and intvelocity actuators '%s' (id = %d)", name.c_str(), id);
    }

    const double* target_range;
    if (trntype == mjTRN_JOINT) {
      pjnt = (mjCJoint*) ptarget;
      if (pjnt->spec.type != mjJNT_HINGE && pjnt->spec.type != mjJNT_SLIDE) {
        throw mjCError(this, "inheritrange can only be used with hinge and slide joints, "
                       "actuator '%s' (id = %d)", name.c_str(), id);
      }
      target_range = pjnt->get_range();
    } else if (trntype == mjTRN_TENDON) {
      mjCTendon* pten = (mjCTendon*) ptarget;
      target_range = pten->get_range();
    } else {
      throw mjCError(this, "inheritrange can only be used with joint and tendon transmission, "
                     "actuator '%s' (id = %d)", name.c_str(), id);
    }

    if (target_range[0] == target_range[1]) {
      throw mjCError(this, "inheritrange used but target '%s' has no range defined in actuator %d",
                     target_.c_str(), id);
    }

    // set range automatically
    double mean   = 0.5*(target_range[1] + target_range[0]);
    double radius = 0.5*(target_range[1] - target_range[0]) * inheritrange;
    range[0] = mean - radius;
    range[1] = mean + radius;
  }

  // if limited is auto, check for inconsistency wrt to autolimits
  if (forcelimited == mjLIMITED_AUTO) {
    bool hasrange = !(forcerange[0]==0 && forcerange[1]==0);
    checklimited(this, model->autolimits, "actuator", "force", forcelimited, hasrange);
  }
  if (ctrllimited == mjLIMITED_AUTO) {
    bool hasrange = !(ctrlrange[0]==0 && ctrlrange[1]==0);
    checklimited(this, model->autolimits, "actuator", "ctrl", ctrllimited, hasrange);
  }
  if (actlimited == mjLIMITED_AUTO) {
    bool hasrange = !(actrange[0]==0 && actrange[1]==0);
    checklimited(this, model->autolimits, "actuator", "act", actlimited, hasrange);
  }

  // check limits
  if (forcerange[0]>=forcerange[1] && is_forcelimited()) {
    throw mjCError(this, "invalid force range for actuator '%s' (id = %d)", name.c_str(), id);
  }
  if (ctrlrange[0]>=ctrlrange[1] && is_ctrllimited()) {
    throw mjCError(this, "invalid control range for actuator '%s' (id = %d)", name.c_str(), id);
  }
  if (actrange[0]>=actrange[1] && is_actlimited()) {
    throw mjCError(this, "invalid actrange for actuator '%s' (id = %d)", name.c_str(), id);
  }
  if (is_actlimited() && dyntype == mjDYN_NONE) {
    throw mjCError(this, "actrange specified but dyntype is 'none' in actuator '%s' (id = %d)",
                   name.c_str(), id);
  }

  // check and set actdim
  if (actdim > 1 && dyntype != mjDYN_USER) {
    throw mjCError(this, "actdim > 1 is only allowed for dyntype 'user' in actuator '%s' (id = %d)",
                   name.c_str(), id);
  }
  if (actdim == 1 && dyntype == mjDYN_NONE) {
    throw mjCError(this, "invalid actdim 1 in stateless actuator '%s' (id = %d)", name.c_str(), id);
  }
  if (actdim == 0 && dyntype != mjDYN_NONE) {
    throw mjCError(this, "invalid actdim 0 in stateful actuator '%s' (id = %d)", name.c_str(), id);
  }

  // set actdim
  if (actdim < 0) {
    actdim = (dyntype != mjDYN_NONE);
  }

  // check muscle parameters
  for (int i=0; i<2; i++) {
    // select gain or bias
    double* prm = NULL;
    if (i==0 && gaintype==mjGAIN_MUSCLE) {
      prm = gainprm;
    } else if (i==1 && biastype==mjBIAS_MUSCLE) {
      prm = biasprm;
    }

    // nothing to check
    if (!prm) {
      continue;
    }

    // range
    if (prm[0]>=prm[1]) {
      throw mjCError(this, "range[0]<range[1] required in muscle '%s' (id = %d)", name.c_str(), id);
    }

    // lmin<1<lmax
    if (prm[4]>=1 || prm[5]<=1) {
      throw mjCError(this, "lmin<1<lmax required in muscle '%s' (id = %d)", name.c_str(), id);
    }

    // scale, vmax, fpmax, fvmax>0
    if (prm[3]<=0 || prm[6]<=0 || prm[7]<=0 || prm[8]<=0) {
      throw mjCError(this,
                     "positive scale, vmax, fpmax, fvmax required in muscle '%s' (id = %d)",
                     name.c_str(), id);
    }
  }

  // plugin
  if (plugin.active) {
    if (plugin_name.empty() && plugin_instance_name.empty()) {
      throw mjCError(
          this, "neither 'plugin' nor 'instance' is specified for actuator '%s', (id = %d)",
          name.c_str(), id);
    }

    mjCPlugin** plugin_instance = (mjCPlugin**)&plugin.instance;
    model->ResolvePlugin(this, plugin_name, plugin_instance_name, plugin_instance);
    const mjpPlugin* pplugin = mjp_getPluginAtSlot((*plugin_instance)->spec.plugin_slot);
    if (!(pplugin->capabilityflags & mjPLUGIN_ACTUATOR)) {
      throw mjCError(this, "plugin '%s' does not support actuators", pplugin->name);
    }
  }
}



//------------------ class mjCSensor implementation ------------------------------------------------

// initialize defaults
mjCSensor::mjCSensor(mjCModel* _model) {
  mjm_defaultSensor(spec);

  // set model
  model = _model;

  // clear private variables
  spec_objname_.clear();
  spec_refname_.clear();
  spec_userdata_.clear();
  obj = nullptr;
  refid = -1;

  // in case this sensor is not compiled
  CopyFromSpec();

  // point to local
  MakePointerLocal();
}



void mjCSensor::MakePointerLocal() {
  spec.element = (mjElement)this;
  spec.name = (mjString)&name;
  spec.classname = (mjString)&classname;
  spec.userdata = (mjDoubleVec)&spec_userdata_;
  spec.objname = (mjString)&spec_objname_;
  spec.refname = (mjString)&spec_refname_;
  spec.plugin.name = (mjString)&plugin_name;
  spec.plugin.instance_name = (mjString)&plugin_instance_name;
  spec.info = (mjString)&info;
}



void mjCSensor::CopyFromSpec() {
  *static_cast<mjmSensor*>(this) = spec;
  userdata_ = spec_userdata_;
  objname_ = spec_objname_;
  refname_ = spec_refname_;
  userdata = (mjDoubleVec)&userdata_;
  objname = (mjString)&objname_;
  refname = (mjString)&refname_;
  plugin.active = spec.plugin.active;
  plugin.instance = spec.plugin.instance;
  plugin.name = spec.plugin.name;
  plugin.instance_name = spec.plugin.instance_name;
}



// compiler
void mjCSensor::Compile(void) {
  CopyFromSpec();

  // resize userdata
  if (userdata_.size() > model->nuser_sensor) {
    throw mjCError(this, "user has more values than nuser_sensor in sensor '%s' (id = %d)",
                   name.c_str(), id);
  }
  userdata_.resize(model->nuser_sensor);

  // require non-negative noise
  if (noise<0) {
    throw mjCError(this, "negative noise in sensor '%s' (id = %d)", name.c_str(), id);
  }

  // require non-negative cutoff
  if (cutoff<0) {
    throw mjCError(this, "negative cutoff in sensor '%s' (id = %d)", name.c_str(), id);
  }

  // get objid from objtype and objname
  if (objtype!=mjOBJ_UNKNOWN) {
    // check for missing object name
    if (objname_.empty()) {
      throw mjCError(this,
                     "missing name of sensorized object in sensor '%s' (id = %d)",
                     name.c_str(), id);
    }

    // find name
    obj = model->FindObject(objtype, objname_);
    if (!obj) {
      throw mjCError(this,
                     "unrecognized name of sensorized object in sensor '%s' (id = %d)",
                     name.c_str(), id);
    }

    // if geom mark it as non visual
    if (objtype == mjOBJ_GEOM) {
      ((mjCGeom*)obj)->SetNotVisual();
    }

    // get sensorized object id
  } else if (type != mjSENS_CLOCK && type != mjSENS_PLUGIN && type != mjSENS_USER) {
    throw mjCError(this, "invalid type in sensor '%s' (id = %d)", name.c_str(), id);
  }

  // get refid from reftype and refname
  if (reftype!=mjOBJ_UNKNOWN) {
    // check for missing object name
    if (refname_.empty()) {
      throw mjCError(this,
                     "missing name of reference frame object in sensor '%s' (id = %d)",
                     name.c_str(), id);
    }

    // find name
    mjCBase* pref = model->FindObject(reftype, refname_);
    if (!pref) {
      throw mjCError(this,
                     "unrecognized name of reference frame object in sensor '%s' (id = %d)",
                     name.c_str(), id);
    }

    // must be attached to object with spatial frame
    if (reftype!=mjOBJ_BODY && reftype!=mjOBJ_XBODY &&
        reftype!=mjOBJ_GEOM && reftype!=mjOBJ_SITE && reftype!=mjOBJ_CAMERA) {
      throw mjCError(this,
                     "reference frame object must be (x)body, geom, site or camera:"
                     " sensor '%s' (id = %d)", name.c_str(), id);
    }

    // get sensorized object id
    refid = pref->id;
  }

  // process according to sensor type
  switch (type) {
  case mjSENS_TOUCH:
  case mjSENS_ACCELEROMETER:
  case mjSENS_VELOCIMETER:
  case mjSENS_GYRO:
  case mjSENS_FORCE:
  case mjSENS_TORQUE:
  case mjSENS_MAGNETOMETER:
  case mjSENS_RANGEFINDER:
  case mjSENS_CAMPROJECTION:
    // must be attached to site
    if (objtype!=mjOBJ_SITE) {
      throw mjCError(this,
                     "sensor must be attached to site: sensor '%s' (id = %d)", name.c_str(), id);
    }

    // set dim and datatype
    if (type==mjSENS_TOUCH || type==mjSENS_RANGEFINDER) {
      dim = 1;
      datatype = mjDATATYPE_POSITIVE;
    } else if (type==mjSENS_CAMPROJECTION) {
      dim = 2;
      datatype = mjDATATYPE_REAL;
    } else {
      dim = 3;
      datatype = mjDATATYPE_REAL;
    }

    // set stage
    if (type==mjSENS_MAGNETOMETER || type==mjSENS_RANGEFINDER || type==mjSENS_CAMPROJECTION) {
      needstage = mjSTAGE_POS;
    } else if (type==mjSENS_GYRO || type==mjSENS_VELOCIMETER) {
      needstage = mjSTAGE_VEL;
    } else {
      needstage = mjSTAGE_ACC;
    }

    // check for camera resolution for camera projection sensor
    if (type==mjSENS_CAMPROJECTION) {
      mjCCamera* camref = (mjCCamera*) model->FindObject(mjOBJ_CAMERA, refname_);
      if (!camref->resolution[0] || !camref->resolution[1]) {
        throw mjCError(this,
                       "camera projection sensor requires camera resolution '%s' (id = %d)",
                       name.c_str(), id);
      }
    }
    break;

  case mjSENS_JOINTPOS:
  case mjSENS_JOINTVEL:
  case mjSENS_JOINTACTFRC:
    // must be attached to joint
    if (objtype!=mjOBJ_JOINT) {
      throw mjCError(this,
                     "sensor must be attached to joint: sensor '%s' (id = %d)", name.c_str(), id);
    }

    // make sure joint is slide or hinge
    if (((mjCJoint*)obj)->type!=mjJNT_SLIDE && ((mjCJoint*)obj)->type!=mjJNT_HINGE) {
      throw mjCError(this,
                     "joint must be slide or hinge in sensor '%s' (id = %d)", name.c_str(), id);
    }

    // set
    dim = 1;
    datatype = mjDATATYPE_REAL;
    if (type==mjSENS_JOINTPOS) {
      needstage = mjSTAGE_POS;
    } else if (type==mjSENS_JOINTVEL) {
      needstage = mjSTAGE_VEL;
    } else if (type==mjSENS_JOINTACTFRC) {
      needstage = mjSTAGE_ACC;
    }
    break;

  case mjSENS_TENDONPOS:
  case mjSENS_TENDONVEL:
    // must be attached to tendon
    if (objtype!=mjOBJ_TENDON) {
      throw mjCError(this,
                     "sensor must be attached to tendon: sensor '%s' (id = %d)", name.c_str(), id);
    }

    // set
    dim = 1;
    datatype = mjDATATYPE_REAL;
    if (type==mjSENS_TENDONPOS) {
      needstage = mjSTAGE_POS;
    } else {
      needstage = mjSTAGE_VEL;
    }
    break;

  case mjSENS_ACTUATORPOS:
  case mjSENS_ACTUATORVEL:
  case mjSENS_ACTUATORFRC:
    // must be attached to actuator
    if (objtype!=mjOBJ_ACTUATOR) {
      throw mjCError(this,
                     "sensor must be attached to actuator: sensor '%s' (id = %d)", name.c_str(), id);
    }

    // set
    dim = 1;
    datatype = mjDATATYPE_REAL;
    if (type==mjSENS_ACTUATORPOS) {
      needstage = mjSTAGE_POS;
    } else if (type==mjSENS_ACTUATORVEL) {
      needstage = mjSTAGE_VEL;
    } else {
      needstage = mjSTAGE_ACC;
    }
    break;

  case mjSENS_BALLQUAT:
  case mjSENS_BALLANGVEL:
    // must be attached to joint
    if (objtype!=mjOBJ_JOINT) {
      throw mjCError(this,
                     "sensor must be attached to joint: sensor '%s' (id = %d)", name.c_str(), id);
    }

    // make sure joint is ball
    if (((mjCJoint*)obj)->type!=mjJNT_BALL) {
      throw mjCError(this,
                     "joint must be ball in sensor '%s' (id = %d)", name.c_str(), id);
    }

    // set
    if (type==mjSENS_BALLQUAT) {
      dim = 4;
      datatype = mjDATATYPE_QUATERNION;
      needstage = mjSTAGE_POS;
    } else {
      dim = 3;
      datatype = mjDATATYPE_REAL;
      needstage = mjSTAGE_VEL;
    }
    break;

  case mjSENS_JOINTLIMITPOS:
  case mjSENS_JOINTLIMITVEL:
  case mjSENS_JOINTLIMITFRC:
    // must be attached to joint
    if (objtype!=mjOBJ_JOINT) {
      throw mjCError(this,
                     "sensor must be attached to joint: sensor '%s' (id = %d)", name.c_str(), id);
    }

    // make sure joint has limit
    if (!((mjCJoint*)obj)->is_limited()) {
      throw mjCError(this, "joint must be limited in sensor '%s' (id = %d)", name.c_str(), id);
    }

    // set
    dim = 1;
    datatype = mjDATATYPE_REAL;
    if (type==mjSENS_JOINTLIMITPOS) {
      needstage = mjSTAGE_POS;
    } else if (type==mjSENS_JOINTLIMITVEL) {
      needstage = mjSTAGE_VEL;
    } else {
      needstage = mjSTAGE_ACC;
    }
    break;

  case mjSENS_TENDONLIMITPOS:
  case mjSENS_TENDONLIMITVEL:
  case mjSENS_TENDONLIMITFRC:
    // must be attached to tendon
    if (objtype!=mjOBJ_TENDON) {
      throw mjCError(this,
                     "sensor must be attached to tendon: sensor '%s' (id = %d)", name.c_str(), id);
    }

    // make sure tendon has limit
    if (!((mjCTendon*)obj)->is_limited()) {
      throw mjCError(this, "tendon must be limited in sensor '%s' (id = %d)", name.c_str(), id);
    }

    // set
    dim = 1;
    datatype = mjDATATYPE_REAL;
    if (type==mjSENS_TENDONLIMITPOS) {
      needstage = mjSTAGE_POS;
    } else if (type==mjSENS_TENDONLIMITVEL) {
      needstage = mjSTAGE_VEL;
    } else {
      needstage = mjSTAGE_ACC;
    }
    break;

  case mjSENS_FRAMEPOS:
  case mjSENS_FRAMEQUAT:
  case mjSENS_FRAMEXAXIS:
  case mjSENS_FRAMEYAXIS:
  case mjSENS_FRAMEZAXIS:
  case mjSENS_FRAMELINVEL:
  case mjSENS_FRAMEANGVEL:
  case mjSENS_FRAMELINACC:
  case mjSENS_FRAMEANGACC:
    // must be attached to object with spatial frame
    if (objtype!=mjOBJ_BODY && objtype!=mjOBJ_XBODY &&
        objtype!=mjOBJ_GEOM && objtype!=mjOBJ_SITE && objtype!=mjOBJ_CAMERA) {
      throw mjCError(this,
                     "sensor must be attached to (x)body, geom, site or camera:"
                     " sensor '%s' (id = %d)", name.c_str(), id);
    }

    // set dim
    if (type==mjSENS_FRAMEQUAT) {
      dim = 4;
    } else {
      dim = 3;
    }

    // set datatype
    if (type==mjSENS_FRAMEQUAT) {
      datatype = mjDATATYPE_QUATERNION;
    } else if (type==mjSENS_FRAMEXAXIS || type==mjSENS_FRAMEYAXIS || type==mjSENS_FRAMEZAXIS) {
      datatype = mjDATATYPE_AXIS;
    } else {
      datatype = mjDATATYPE_REAL;
    }

    // set needstage
    if (type==mjSENS_FRAMELINACC || type==mjSENS_FRAMEANGACC) {
      needstage = mjSTAGE_ACC;
    } else if (type==mjSENS_FRAMELINVEL || type==mjSENS_FRAMEANGVEL) {
      needstage = mjSTAGE_VEL;
    } else {
      needstage = mjSTAGE_POS;
    }
    break;

  case mjSENS_SUBTREECOM:
  case mjSENS_SUBTREELINVEL:
  case mjSENS_SUBTREEANGMOM:
    // must be attached to body
    if (objtype!=mjOBJ_BODY) {
      throw mjCError(this,
                     "sensor must be attached to body: sensor '%s' (id = %d)", name.c_str(), id);
    }

    // set
    dim = 3;
    datatype = mjDATATYPE_REAL;
    if (type==mjSENS_SUBTREECOM) {
      needstage = mjSTAGE_POS;
    } else {
      needstage = mjSTAGE_VEL;
    }
    break;

  case mjSENS_CLOCK:
    dim = 1;
    needstage = mjSTAGE_POS;
    datatype = mjDATATYPE_REAL;
    break;

  case mjSENS_USER:
    // check for negative dim
    if (dim<0) {
      throw mjCError(this, "sensor dim must be positive: sensor '%s' (id = %d)", name.c_str(), id);
    }

    // make sure dim is consistent with datatype
    if (datatype==mjDATATYPE_AXIS && dim!=3) {
      throw mjCError(this,
                     "datatype AXIS requires dim=3 in sensor '%s' (id = %d)", name.c_str(), id);
    }
    if (datatype==mjDATATYPE_QUATERNION && dim!=4) {
      throw mjCError(this,
                     "datatype QUATERNION requires dim=4 in sensor '%s' (id = %d)",
                     name.c_str(), id);
    }
    break;

  case mjSENS_PLUGIN:
    dim = 0;  // to be filled in by the plugin later
    datatype = mjDATATYPE_REAL;  // no noise added to plugin sensors, this attribute is unused

    if (plugin_name.empty() && plugin_instance_name.empty()) {
      throw mjCError(
          this, "neither 'plugin' nor 'instance' is specified for sensor '%s', (id = %d)",
          name.c_str(), id);
    }

    // resolve plugin instance, or create one if using the "plugin" attribute shortcut
    {
      mjCPlugin** plugin_instance = (mjCPlugin**)&plugin.instance;
      model->ResolvePlugin(this, plugin_name, plugin_instance_name, plugin_instance);
      const mjpPlugin* pplugin = mjp_getPluginAtSlot((*plugin_instance)->spec.plugin_slot);
      if (!(pplugin->capabilityflags & mjPLUGIN_SENSOR)) {
        throw mjCError(this, "plugin '%s' does not support sensors", pplugin->name);
      }
      needstage = static_cast<mjtStage>(pplugin->needstage);
    }

    break;

  default:
    throw mjCError(this, "invalid type in sensor '%s' (id = %d)", name.c_str(), id);
  }

  // check cutoff for incompatible data types
  if (cutoff>0 && (datatype==mjDATATYPE_AXIS || datatype==mjDATATYPE_QUATERNION)) {
    throw mjCError(this,
                   "cutoff applied to axis or quaternion datatype in sensor '%s' (id = %d)",
                   name.c_str(), id);
  }
}



//------------------ class mjCNumeric implementation -----------------------------------------------

// constructor
mjCNumeric::mjCNumeric(mjCModel* _model) {
  mjm_defaultNumeric(spec);

  // set model pointer
  model = _model;

  // clear variables
  spec_data_.clear();

  // point to local
  PointToLocal();

  // in case this numeric is not compiled
  CopyFromSpec();
}



void mjCNumeric::PointToLocal() {
  spec.element = (mjElement)this;
  spec.name = (mjString)&name;
  spec.data = (mjDoubleVec)&spec_data_;
  spec.info = (mjString)&info;
}



void mjCNumeric::CopyFromSpec() {
  *static_cast<mjmNumeric*>(this) = spec;
  data_ = spec_data_;
  data = (mjDoubleVec)&data_;
}



// destructor
mjCNumeric::~mjCNumeric() {
  spec_data_.clear();
  data_.clear();
}



// compiler
void mjCNumeric::Compile(void) {
  CopyFromSpec();

  // check for size conflict
  if (size && !data_.empty() && size<(int)data_.size()) {
    throw mjCError(this,
                   "numeric '%s' (id = %d): specified size smaller than initialization array",
                   name.c_str(), id);
  }

  // set size if left unspecified
  if (!size) {
    size = (int)data_.size();
  }

  // size cannot be zero
  if (!size) {
    throw mjCError(this, "numeric '%s' (id = %d): size cannot be zero", name.c_str(), id);
  }
}



//------------------ class mjCText implementation --------------------------------------------------

// constructor
mjCText::mjCText(mjCModel* _model) {
  mjm_defaultText(spec);

  // set model pointer
  model = _model;

  // clear variables
  spec_data_.clear();

  // point to local
  PointToLocal();

  // in case this text is not compiled
  CopyFromSpec();
}



void mjCText::PointToLocal() {
  spec.element = (mjElement)this;
  spec.name = (mjString)&name;
  spec.data = (mjString)&spec_data_;
  spec.info = (mjString)&info;
}



void mjCText::CopyFromSpec() {
  *static_cast<mjmText*>(this) = spec;
  data_ = spec_data_;
  data = (mjString)&data_;
}



// destructor
mjCText::~mjCText() {
  data_.clear();
  spec_data_.clear();
}



// compiler
void mjCText::Compile(void) {
  CopyFromSpec();

  // size cannot be zero
  if (data_.empty()) {
    throw mjCError(this, "text '%s' (id = %d): size cannot be zero", name.c_str(), id);
  }
}



//------------------ class mjCTuple implementation -------------------------------------------------

// constructor
mjCTuple::mjCTuple(mjCModel* _model) {
  mjm_defaultTuple(spec);

  // set model pointer
  model = _model;

  // clear variables
  spec_objtype_.clear();
  spec_objname_.clear();
  spec_objprm_.clear();
  obj.clear();

  // point to local
  PointToLocal();

  // in case this tuple is not compiled
  CopyFromSpec();
}



void mjCTuple::PointToLocal() {
  spec.element = (mjElement)this;
  spec.name = (mjString)&name;
  spec.objtype = (mjIntVec)&spec_objtype_;
  spec.objname = (mjStringVec)&spec_objname_;
  spec.objprm = (mjDoubleVec)&spec_objprm_;
  spec.info = (mjString)&info;
}



void mjCTuple::CopyFromSpec() {
  *static_cast<mjmTuple*>(this) = spec;
  objtype_ = spec_objtype_;
  objname_ = spec_objname_;
  objprm_ = spec_objprm_;
  objtype = (mjIntVec)&objtype_;
  objname = (mjStringVec)&objname_;
  objprm = (mjDoubleVec)&objprm_;
}



// destructor
mjCTuple::~mjCTuple() {
  objtype_.clear();
  objname_.clear();
  objprm_.clear();
  spec_objtype_.clear();
  spec_objname_.clear();
  spec_objprm_.clear();
  obj.clear();
}



// compiler
void mjCTuple::Compile(void) {
  CopyFromSpec();

  // check for empty tuple
  if (objtype_.empty()) {
    throw mjCError(this, "tuple '%s' (id = %d) is empty", name.c_str(), id);
  }

  // check for size conflict
  if (objtype_.size()!=objname_.size() || objtype_.size()!=objprm_.size()) {
    throw mjCError(this,
                   "tuple '%s' (id = %d) has object arrays with different sizes", name.c_str(), id);
  }

  // resize objid to correct size
  obj.resize(objtype_.size());

  // find objects, fill in ids
  for (int i=0; i<objtype_.size(); i++) {
    // find object by type and name
    mjCBase* res = model->FindObject(objtype_[i], objname_[i]);
    if (!res) {
      throw mjCError(this, "unrecognized object '%s' in tuple %d", objname_[i].c_str(), id);
    }

    // if geom mark it as non visual
    if (objtype_[i] == mjOBJ_GEOM) {
      ((mjCGeom*)res)->SetNotVisual();
    }

    // assign id
    obj[i] = res;
  }
}



//------------------ class mjCKey implementation ---------------------------------------------------

// constructor
mjCKey::mjCKey(mjCModel* _model) {
  mjm_defaultKey(spec);

  // set model pointer
  model = _model;

  // clear variables
  spec_qpos_.clear();
  spec_qvel_.clear();
  spec_act_.clear();
  spec_mpos_.clear();
  spec_mquat_.clear();
  spec_ctrl_.clear();

  // point to local
  PointToLocal();

  // in case this keyframe is not compiled
  CopyFromSpec();
}



void mjCKey::PointToLocal() {
  spec.element = (mjElement)this;
  spec.name = (mjString)&name;
  spec.qpos = (mjDoubleVec)&spec_qpos_;
  spec.qvel = (mjDoubleVec)&spec_qvel_;
  spec.act = (mjDoubleVec)&spec_act_;
  spec.mpos = (mjDoubleVec)&spec_mpos_;
  spec.mquat = (mjDoubleVec)&spec_mquat_;
  spec.ctrl = (mjDoubleVec)&spec_ctrl_;
  spec.info = (mjString)&info;
}



void mjCKey::CopyFromSpec() {
  *static_cast<mjmKey*>(this) = spec;
  qpos_ = spec_qpos_;
  qvel_ = spec_qvel_;
  act_ = spec_act_;
  mpos_ = spec_mpos_;
  mquat_ = spec_mquat_;
  ctrl_ = spec_ctrl_;
  qpos = (mjDoubleVec)&qpos_;
  qvel = (mjDoubleVec)&qvel_;
  act = (mjDoubleVec)&act_;
  mpos = (mjDoubleVec)&mpos_;
  mquat = (mjDoubleVec)&mquat_;
  ctrl = (mjDoubleVec)&ctrl_;
}



// destructor
mjCKey::~mjCKey() {
  qpos_.clear();
  qvel_.clear();
  act_.clear();
  mpos_.clear();
  mquat_.clear();
  ctrl_.clear();
  spec_qpos_.clear();
  spec_qvel_.clear();
  spec_act_.clear();
  spec_mpos_.clear();
  spec_mquat_.clear();
  spec_ctrl_.clear();
}



// compiler
void mjCKey::Compile(const mjModel* m) {
  CopyFromSpec();

  // qpos: allocate or check size
  if (qpos_.empty()) {
    qpos_.resize(m->nq);
    for (int i=0; i<m->nq; i++) {
      qpos_[i] = (double)m->qpos0[i];
    }
  } else if (qpos_.size()!=m->nq) {
    throw mjCError(this, "key %d: invalid qpos size, expected length %d", nullptr, id, m->nq);
  }

  // qvel: allocate or check size
  if (qvel_.empty()) {
    qvel_.resize(m->nv);
    for (int i=0; i<m->nv; i++) {
      qvel_[i] = 0;
    }
  } else if (qvel_.size()!=m->nv) {
    throw mjCError(this, "key %d: invalid qvel size, expected length %d", nullptr, id, m->nv);
  }

  // act: allocate or check size
  if (act_.empty()) {
    act_.resize(m->na);
    for (int i=0; i<m->na; i++) {
      act_[i] = 0;
    }
  } else if (act_.size()!=m->na) {
    throw mjCError(this, "key %d: invalid act size, expected length %d", nullptr, id, m->na);
  }

  // mpos: allocate or check size
  if (mpos_.empty()) {
    mpos_.resize(3*m->nmocap);
    if (m->nmocap) {
      for (int i=0; i<m->nbody; i++) {
        if (m->body_mocapid[i]>=0) {
          int mocapid = m->body_mocapid[i];
          mpos_[3*mocapid]   = m->body_pos[3*i];
          mpos_[3*mocapid+1] = m->body_pos[3*i+1];
          mpos_[3*mocapid+2] = m->body_pos[3*i+2];
        }
      }
    }
  } else if (mpos_.size()!=3*m->nmocap) {
    throw mjCError(this, "key %d: invalid mpos size, expected length %d", nullptr, id, 3*m->nmocap);
  }

  // mquat: allocate or check size
  if (mquat_.empty()) {
    mquat_.resize(4*m->nmocap);
    if (m->nmocap) {
      for (int i=0; i<m->nbody; i++) {
        if (m->body_mocapid[i]>=0) {
          int mocapid = m->body_mocapid[i];
          mquat_[4*mocapid]   = m->body_quat[4*i];
          mquat_[4*mocapid+1] = m->body_quat[4*i+1];
          mquat_[4*mocapid+2] = m->body_quat[4*i+2];
          mquat_[4*mocapid+3] = m->body_quat[4*i+3];
        }
      }
    }
  } else if (mquat_.size()!=4*m->nmocap) {
    throw mjCError(this, "key %d: invalid mquat size, expected length %d", nullptr, id, 4*m->nmocap);
  }

  // ctrl: allocate or check size
  if (ctrl_.empty()) {
    ctrl_.resize(m->nu);
    for (int i=0; i<m->nu; i++) {
      ctrl_[i] = 0;
    }
  } else if (ctrl_.size()!=m->nu) {
    throw mjCError(this, "key %d: invalid ctrl size, expected length %d", nullptr, id, m->nu);
  }
}


//------------------ class mjCPlugin implementation ------------------------------------------------

// initialize defaults
mjCPlugin::mjCPlugin(mjCModel* _model) {
  name = "";
  nstate = -1;
  parent = this;
  model = _model;
  name.clear();
  instance_name.clear();

  // public interface
  mjm_defaultPlugin(spec);
  spec.name = (mjString)&name;
  spec.instance_name = (mjString)&instance_name;
  spec.info = (mjString)&info;
}



// compiler
void mjCPlugin::Compile(void) {
  const mjpPlugin* plugin = mjp_getPluginAtSlot(spec.plugin_slot);

  // clear precompiled
  flattened_attributes.clear();
  std::map<std::string, std::string, std::less<>> config_attribs_copy = config_attribs;

  // concatenate all of the plugin's attribute values (as null-terminated strings) into
  // flattened_attributes, in the order declared in the mjpPlugin
  // each valid attribute found is appended to flattened_attributes and removed from xml_attributes
  for (int i = 0; i < plugin->nattribute; ++i) {
    std::string_view attr(plugin->attributes[i]);
    auto it = config_attribs_copy.find(attr);
    if (it == config_attribs_copy.end()) {
      flattened_attributes.push_back('\0');
    } else {
      auto original_size = flattened_attributes.size();
      flattened_attributes.resize(original_size + it->second.size() + 1);
      std::memcpy(&flattened_attributes[original_size], it->second.c_str(),
                  it->second.size() + 1);
      config_attribs_copy.erase(it);
    }
  }

  // anything left in xml_attributes at this stage is not a valid attribute
  if (!config_attribs_copy.empty()) {
    std::string error =
        "unrecognized attribute 'plugin:" + config_attribs_copy.begin()->first +
        "' for plugin " + std::string(plugin->name) + "'";
    throw mjCError(parent, "%s", error.c_str());
  }
}
