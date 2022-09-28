#pragma once

#include <OgrePrerequisites.h>

#include <OgreSimpleRenderable.h>

namespace ogre_primitives
{
using namespace Ogre;

class Frustum2D : public SimpleRenderable
{
protected:
  bool fill_;
  float radius_;
  void initCircularSectorGround();

public:
  Frustum2D(bool fill);
  Frustum2D(const String& name, bool fill);
  ~Frustum2D();

  void setParameters(float angle_left, float angle_right, float range, float height);

  Real getSquaredViewDepth(const Camera* cam) const;

  Real getBoundingRadius(void) const
  {
    return radius_;
  }
};

}  // namespace ogre_primitives