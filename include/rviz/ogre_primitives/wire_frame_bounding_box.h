#ifndef WIRE_FRAME_BOUNDING_BOX_H
#define WIRE_FRAME_BOUNDING_BOX_H

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
/** Allows the rendering of a wireframe bounding box.
@remarks
    This class builds a wireframe renderable from a given aabb. A pointer to this class can be
                added to a render queue to display the bounding box of an object.
*/

using namespace Ogre;

class _OgreExport WireBoundingBox : public SimpleRenderable
{

  protected:

    /** Builds the wireframe line list.
     */
    void setupBoundingBoxVertices( const AxisAlignedBox& aab );

    Real mRadius;

    void _initWireBoundingBox();

  public:
    WireBoundingBox();
    WireBoundingBox( const String& name );
    ~WireBoundingBox();

    /** Builds the wireframe line list.
        @param
            aabb bounding box to build a wireframe from.
    */
    void setupBoundingBox( const AxisAlignedBox& aabb );

    Real getSquaredViewDepth( const Camera* cam ) const;

    Real getBoundingRadius( void ) const
    {
        return mRadius;
    }
};
/** @} */
/** @} */

} // namespace rviz

#endif