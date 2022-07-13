#include <OgreCamera.h>
#include <OgreHardwareBufferManager.h>
#include <OgreSimpleRenderable.h>
#include <OgreMaterialManager.h>

#include <rviz/ogre_primitives/map_tile.h>

namespace rviz
{
#define BINDING 0

MapTile::MapTile() : radius_(0), SimpleRenderable()
{
  initMapTile();
}

MapTile::MapTile(const String& name) : radius_(0), SimpleRenderable(name)
{
  initMapTile();
}

MapTile::~MapTile()
{
  delete mRenderOp.vertexData;
}

void MapTile::initMapTile()
{
  mRenderOp.vertexData = new VertexData();

  mRenderOp.indexData = nullptr;
  mRenderOp.vertexData->vertexCount = 6;
  mRenderOp.vertexData->vertexStart = 0;
  mRenderOp.operationType = RenderOperation::OT_TRIANGLE_LIST;
  mRenderOp.useIndexes = false;
  mRenderOp.useGlobalInstancingVertexBufferIsAvailable = false;

  VertexDeclaration* decl = mRenderOp.vertexData->vertexDeclaration;
  VertexBufferBinding* bind = mRenderOp.vertexData->vertexBufferBinding;

  unsigned long declSize = 0;
  decl->addElement(BINDING, declSize, VET_FLOAT3, VES_POSITION);
  declSize += VertexElement::getTypeSize(VET_FLOAT3);
  decl->addElement(BINDING, declSize, VET_FLOAT2, VES_TEXTURE_COORDINATES);
  declSize += VertexElement::getTypeSize(VET_FLOAT2);
  decl->addElement(BINDING, declSize, VET_FLOAT3, VES_NORMAL);
  declSize += VertexElement::getTypeSize(VET_FLOAT3);

  HardwareVertexBufferSharedPtr vbuf = HardwareBufferManager::getSingleton().createVertexBuffer(
      declSize, mRenderOp.vertexData->vertexCount, HardwareBuffer::HBU_STATIC_WRITE_ONLY);

  // Bind buffer
  bind->setBinding(BINDING, vbuf);

  // set basic white material
  auto material = Ogre::MaterialManager::getSingleton().getByName("BaseWhiteNoLighting");
  this->setMaterial(material);
}

void MapTile::setParameters(float top_left_x, float top_left_y, float bottom_left_x, float bottom_left_y,
                            float bottom_right_x, float bottom_right_y, float top_right_x, float top_right_y,
                            float tile_elevation)
{
  HardwareVertexBufferSharedPtr vbuf = mRenderOp.vertexData->vertexBufferBinding->getBuffer(BINDING);

  auto* pPos = static_cast<float*>(vbuf->lock(HardwareBuffer::HBL_DISCARD));

  // triangle 1
  // point 1
  *pPos++ = bottom_left_x;  // position
  *pPos++ = bottom_left_y;
  *pPos++ = tile_elevation;
  *pPos++ = 0;  // texture coords
  *pPos++ = 0;
  *pPos++ = 0;  // normal
  *pPos++ = 0;
  *pPos++ = 1;
  // point 2
  *pPos++ = top_right_x;  // position
  *pPos++ = top_right_y;
  *pPos++ = tile_elevation;
  *pPos++ = 1;  // texture coords
  *pPos++ = 1;
  *pPos++ = 0;  // normal
  *pPos++ = 0;
  *pPos++ = 1;
  // point 3
  *pPos++ = top_left_x;  // position
  *pPos++ = top_left_y;
  *pPos++ = tile_elevation;
  *pPos++ = 0;  // texture coords
  *pPos++ = 1;
  *pPos++ = 0;  // normal
  *pPos++ = 0;
  *pPos++ = 1;

  // triangle 2
  // point 1
  *pPos++ = bottom_left_x;  // position
  *pPos++ = bottom_left_y;
  *pPos++ = tile_elevation;
  *pPos++ = 0;  // texture coords
  *pPos++ = 0;
  *pPos++ = 0;  // normal
  *pPos++ = 0;
  *pPos++ = 1;
  // point 2
  *pPos++ = bottom_right_x;  // position
  *pPos++ = bottom_right_y;
  *pPos++ = tile_elevation;
  *pPos++ = 1;  // texture coords
  *pPos++ = 0;
  *pPos++ = 0;  // normal
  *pPos++ = 0;
  *pPos++ = 1;
  // point 3
  *pPos++ = top_right_x;  // position
  *pPos++ = top_right_y;
  *pPos++ = tile_elevation;
  *pPos++ = 1;  // texture coords
  *pPos++ = 1;
  *pPos++ = 0;  // normal
  *pPos++ = 0;
  *pPos++ = 1;

  vbuf->unlock();

  float min_x = std::min(top_left_x, bottom_left_x);
  float min_y = std::min(bottom_left_y, bottom_right_y);
  float max_x = std::max(top_right_x, bottom_right_x);
  float max_y = std::max(top_left_y, top_right_y);

  setBoundingBox(Ogre::AxisAlignedBox(min_x, min_y, tile_elevation, max_x, max_y, tile_elevation));

  radius_ = (mBox.getMaximum() - mBox.getMinimum()).length();
}

//-----------------------------------------------------------------------
Real MapTile::getSquaredViewDepth(const Camera* cam) const
{
  Vector3 min, max, mid, dist;
  min = mBox.getMinimum();
  max = mBox.getMaximum();
  mid = ((max - min) * 0.5) + min;
  dist = cam->getDerivedPosition() - mid;

  return dist.squaredLength();
}

}  // namespace rviz