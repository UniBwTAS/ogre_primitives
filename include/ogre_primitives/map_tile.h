#pragma once

#include <OgrePrerequisites.h>

#include <OgreSimpleRenderable.h>

namespace ogre_primitives
{
using namespace Ogre;

class MapTile : public SimpleRenderable
{
protected:
  float radius_;
  void initMapTile();

public:
  MapTile();
  MapTile(const String& name);
  ~MapTile();

  void setParameters(float top_left_x, float top_left_y, float bottom_left_x, float bottom_left_y, float bottom_right_x,
                     float bottom_right_y, float top_right_x, float top_right_y, float tile_elevation);

  Real getSquaredViewDepth(const Camera* cam) const;

  Real getBoundingRadius(void) const
  {
    return radius_;
  }
};

}  // namespace ogre_primitives