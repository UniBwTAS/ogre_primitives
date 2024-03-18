#ifndef SOLID_BOUNDING_BOX_H
#define SOLID_BOUNDING_BOX_H

#include <OgrePrerequisites.h>

#include <OgreSimpleRenderable.h>

namespace rviz
{
/** \addtogroup Core
 *  @{
 */
/** \addtogroup Scene
 *  @{
 */
/** Allows the rendering of a solid bounding box.
@remarks
    This class builds a solid renderable from a given aabb. A pointer to this class can be
                added to a render queue to display the bounding box of an object.
*/

using namespace Ogre;

class _OgreExport SolidBoundingBox : public SimpleRenderable
{
protected:
  /** Builds the solid triangle list.
   */
  void setupBoundingBoxVertices(const AxisAlignedBox& aab);

  Real mRadius;

  void _initSolidBoundingBox();

public:
  SolidBoundingBox();
  SolidBoundingBox(const String& name);
  ~SolidBoundingBox();

  /** Builds the solid triangle list.
      @param
          aabb bounding box to build a solid from.
  */
  void setupBoundingBox(const AxisAlignedBox& aabb);

  Real getSquaredViewDepth(const Camera* cam) const;

  Real getBoundingRadius(void) const
  {
    return mRadius;
  }
};
/** @} */
/** @} */

}  // namespace rviz

#endif