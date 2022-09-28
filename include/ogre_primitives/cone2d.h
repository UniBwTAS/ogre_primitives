#pragma once

#include <OgrePrerequisites.h>

#include <OgreSimpleRenderable.h>

namespace ogre_primitives
{
using namespace Ogre;

class Cone2D : public SimpleRenderable
{
protected:
  bool fill_;
  float range_min_;
  float radius_;
  void initCircularSectorGround();

public:
  Cone2D(bool fill, float range_min = 0);
  Cone2D(const String& name, bool fill, float range_min = 0);
  ~Cone2D();

  void setParameters(float angle_min, float angle_max, float range);

  Real getSquaredViewDepth(const Camera* cam) const;

  Real getBoundingRadius(void) const
  {
    return radius_;
  }
};

}  // namespace ogre_primitives