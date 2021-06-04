#include <OgreCamera.h>
#include <OgreHardwareBufferManager.h>
#include <OgreSimpleRenderable.h>
#include <OgreTechnique.h>

#include <rviz/ogre_primitives/solid_bounding_box.h>

namespace rviz
{
#define BINDING 0

SolidBoundingBox::SolidBoundingBox() : SimpleRenderable()
{
  _initSolidBoundingBox();
}

SolidBoundingBox::SolidBoundingBox(const String& name) : SimpleRenderable(name)
{
  _initSolidBoundingBox();
}

void SolidBoundingBox::_initSolidBoundingBox()
{
  mRenderOp.vertexData = OGRE_NEW VertexData();

  mRenderOp.indexData = nullptr;
  mRenderOp.vertexData->vertexCount = 36;  // 6 faces * 2 triangles * 3 vertices = 36
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
  this->setMaterial("BaseWhiteNoLighting");
}

SolidBoundingBox::~SolidBoundingBox()
{
  OGRE_DELETE mRenderOp.vertexData;
}

void SolidBoundingBox::setupBoundingBox(const AxisAlignedBox& aabb)
{
  // init the vertices to the aabb
  setupBoundingBoxVertices(aabb);

  // setup the bounding box of this SimpleRenderable
  setBoundingBox(aabb);
}

//-----------------------------------------------------------------------
void SolidBoundingBox::setupBoundingBoxVertices(const AxisAlignedBox& aab)
{
  Vector3 vmax = aab.getMaximum();
  Vector3 vmin = aab.getMinimum();

  Real sqLen = std::max(vmax.squaredLength(), vmin.squaredLength());
  mRadius = Math::Sqrt(sqLen);

  Real maxx = vmax.x;
  Real maxy = vmax.y;
  Real maxz = vmax.z;

  Real minx = vmin.x;
  Real miny = vmin.y;
  Real minz = vmin.z;

  // fill in the Vertex buffer
  HardwareVertexBufferSharedPtr vbuf = mRenderOp.vertexData->vertexBufferBinding->getBuffer(BINDING);

  auto* pPos = static_cast<float*>(vbuf->lock(HardwareBuffer::HBL_DISCARD));

  // top
  *pPos++ = minx, *pPos++ = maxy, *pPos++ = maxz;
  *pPos++ = 1, *pPos++ = 0;
  *pPos++ = 0, *pPos++ = 0, *pPos++ = 1;
  *pPos++ = minx, *pPos++ = miny, *pPos++ = maxz;
  *pPos++ = 0, *pPos++ = 0;
  *pPos++ = 0, *pPos++ = 0, *pPos++ = 1;
  *pPos++ = maxx, *pPos++ = maxy, *pPos++ = maxz;
  *pPos++ = 1, *pPos++ = 1;
  *pPos++ = 0, *pPos++ = 0, *pPos++ = 1;
  *pPos++ = maxx, *pPos++ = maxy, *pPos++ = maxz;
  *pPos++ = 1, *pPos++ = 1;
  *pPos++ = 0, *pPos++ = 0, *pPos++ = 1;
  *pPos++ = minx, *pPos++ = miny, *pPos++ = maxz;
  *pPos++ = 0, *pPos++ = 0;
  *pPos++ = 0, *pPos++ = 0, *pPos++ = 1;
  *pPos++ = maxx, *pPos++ = miny, *pPos++ = maxz;
  *pPos++ = 0, *pPos++ = 1;
  *pPos++ = 0, *pPos++ = 0, *pPos++ = 1;

  // bottom
  *pPos++ = minx, *pPos++ = maxy, *pPos++ = minz;  // position
  *pPos++ = 1, *pPos++ = 0;                        // texture
  *pPos++ = 0, *pPos++ = 0, *pPos++ = -1;          // normal
  *pPos++ = maxx, *pPos++ = maxy, *pPos++ = minz;
  *pPos++ = 0, *pPos++ = 0;
  *pPos++ = 0, *pPos++ = 0, *pPos++ = -1;
  *pPos++ = minx, *pPos++ = miny, *pPos++ = minz;
  *pPos++ = 1, *pPos++ = 1;
  *pPos++ = 0, *pPos++ = 0, *pPos++ = -1;
  *pPos++ = maxx, *pPos++ = maxy, *pPos++ = minz;
  *pPos++ = 1, *pPos++ = 1;
  *pPos++ = 0, *pPos++ = 0, *pPos++ = -1;
  *pPos++ = maxx, *pPos++ = miny, *pPos++ = minz;
  *pPos++ = 0, *pPos++ = 0;
  *pPos++ = 0, *pPos++ = 0, *pPos++ = -1;
  *pPos++ = minx, *pPos++ = miny, *pPos++ = minz;
  *pPos++ = 0, *pPos++ = 1;
  *pPos++ = 0, *pPos++ = 0, *pPos++ = -1;

  // back
  *pPos++ = maxx, *pPos++ = maxy, *pPos++ = maxz;
  *pPos++ = 1, *pPos++ = 0;
  *pPos++ = 1, *pPos++ = 0, *pPos++ = 0;
  *pPos++ = maxx, *pPos++ = miny, *pPos++ = maxz;
  *pPos++ = 0, *pPos++ = 0;
  *pPos++ = 1, *pPos++ = 0, *pPos++ = 0;
  *pPos++ = maxx, *pPos++ = maxy, *pPos++ = minz;
  *pPos++ = 1, *pPos++ = 1;
  *pPos++ = 1, *pPos++ = 0, *pPos++ = 0;
  *pPos++ = maxx, *pPos++ = maxy, *pPos++ = minz;
  *pPos++ = 1, *pPos++ = 1;
  *pPos++ = 1, *pPos++ = 0, *pPos++ = 0;
  *pPos++ = maxx, *pPos++ = miny, *pPos++ = maxz;
  *pPos++ = 0, *pPos++ = 0;
  *pPos++ = 1, *pPos++ = 0, *pPos++ = 0;
  *pPos++ = maxx, *pPos++ = miny, *pPos++ = minz;
  *pPos++ = 0, *pPos++ = 1;
  *pPos++ = 1, *pPos++ = 0, *pPos++ = 0;

  // front
  *pPos++ = minx, *pPos++ = maxy, *pPos++ = maxz;
  *pPos++ = 1, *pPos++ = 0;
  *pPos++ = -1, *pPos++ = 0, *pPos++ = 0;
  *pPos++ = minx, *pPos++ = maxy, *pPos++ = minz;
  *pPos++ = 0, *pPos++ = 0;
  *pPos++ = -1, *pPos++ = 0, *pPos++ = 0;
  *pPos++ = minx, *pPos++ = miny, *pPos++ = maxz;
  *pPos++ = 1, *pPos++ = 1;
  *pPos++ = -1, *pPos++ = 0, *pPos++ = 0;
  *pPos++ = minx, *pPos++ = maxy, *pPos++ = minz;
  *pPos++ = 1, *pPos++ = 1;
  *pPos++ = -1, *pPos++ = 0, *pPos++ = 0;
  *pPos++ = minx, *pPos++ = miny, *pPos++ = minz;
  *pPos++ = 0, *pPos++ = 0;
  *pPos++ = -1, *pPos++ = 0, *pPos++ = 0;
  *pPos++ = minx, *pPos++ = miny, *pPos++ = maxz;
  *pPos++ = 0, *pPos++ = 1;
  *pPos++ = -1, *pPos++ = 0, *pPos++ = 0;

  // left
  *pPos++ = minx, *pPos++ = maxy, *pPos++ = minz;
  *pPos++ = 1, *pPos++ = 0;
  *pPos++ = 0, *pPos++ = 1, *pPos++ = 0;
  *pPos++ = minx, *pPos++ = maxy, *pPos++ = maxz;
  *pPos++ = 0, *pPos++ = 0;
  *pPos++ = 0, *pPos++ = 1, *pPos++ = 0;
  *pPos++ = maxx, *pPos++ = maxy, *pPos++ = minz;
  *pPos++ = 1, *pPos++ = 1;
  *pPos++ = 0, *pPos++ = 1, *pPos++ = 0;
  *pPos++ = maxx, *pPos++ = maxy, *pPos++ = minz;
  *pPos++ = 1, *pPos++ = 1;
  *pPos++ = 0, *pPos++ = 1, *pPos++ = 0;
  *pPos++ = minx, *pPos++ = maxy, *pPos++ = maxz;
  *pPos++ = 0, *pPos++ = 0;
  *pPos++ = 0, *pPos++ = 1, *pPos++ = 0;
  *pPos++ = maxx, *pPos++ = maxy, *pPos++ = maxz;
  *pPos++ = 0, *pPos++ = 1;
  *pPos++ = 0, *pPos++ = 1, *pPos++ = 0;

  // right
  *pPos++ = minx, *pPos++ = miny, *pPos++ = minz;
  *pPos++ = 1, *pPos++ = 0;
  *pPos++ = 0, *pPos++ = -1, *pPos++ = 0;
  *pPos++ = maxx, *pPos++ = miny, *pPos++ = minz;
  *pPos++ = 0, *pPos++ = 0;
  *pPos++ = 0, *pPos++ = -1, *pPos++ = 0;
  *pPos++ = minx, *pPos++ = miny, *pPos++ = maxz;
  *pPos++ = 1, *pPos++ = 1;
  *pPos++ = 0, *pPos++ = -1, *pPos++ = 0;
  *pPos++ = maxx, *pPos++ = miny, *pPos++ = minz;
  *pPos++ = 1, *pPos++ = 1;
  *pPos++ = 0, *pPos++ = -1, *pPos++ = 0;
  *pPos++ = maxx, *pPos++ = miny, *pPos++ = maxz;
  *pPos++ = 0, *pPos++ = 0;
  *pPos++ = 0, *pPos++ = -1, *pPos++ = 0;
  *pPos++ = minx, *pPos++ = miny, *pPos++ = maxz;
  *pPos++ = 0, *pPos++ = 1;
  *pPos++ = 0, *pPos++ = -1, *pPos++ = 0;

  vbuf->unlock();
}

//-----------------------------------------------------------------------
Real SolidBoundingBox::getSquaredViewDepth(const Camera* cam) const
{
  Vector3 min, max, mid, dist;
  min = mBox.getMinimum();
  max = mBox.getMaximum();
  mid = ((max - min) * 0.5) + min;
  dist = cam->getDerivedPosition() - mid;

  return dist.squaredLength();
}

}  // namespace rviz