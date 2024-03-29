#include <OgreCamera.h>
#include <OgreHardwareBufferManager.h>
#include <OgreSimpleRenderable.h>

#include <ogre_primitives/wire_frame_bounding_box.h>

namespace rviz
{
#define POSITION_BINDING 0

WireBoundingBox::WireBoundingBox() : SimpleRenderable()
{
  _initWireBoundingBox();
}

WireBoundingBox::WireBoundingBox(const String& name) : SimpleRenderable(name)
{
  _initWireBoundingBox();
}

void WireBoundingBox::_initWireBoundingBox()
{
  mRenderOp.vertexData = OGRE_NEW VertexData();

  mRenderOp.indexData = 0;
  mRenderOp.vertexData->vertexCount = 24;
  mRenderOp.vertexData->vertexStart = 0;
  mRenderOp.operationType = RenderOperation::OT_LINE_LIST;
  mRenderOp.useIndexes = false;
  mRenderOp.useGlobalInstancingVertexBufferIsAvailable = false;

  VertexDeclaration* decl = mRenderOp.vertexData->vertexDeclaration;
  VertexBufferBinding* bind = mRenderOp.vertexData->vertexBufferBinding;

  decl->addElement(POSITION_BINDING, 0, VET_FLOAT3, VES_POSITION);

  HardwareVertexBufferSharedPtr vbuf = HardwareBufferManager::getSingleton().createVertexBuffer(
      decl->getVertexSize(POSITION_BINDING), mRenderOp.vertexData->vertexCount, HardwareBuffer::HBU_STATIC_WRITE_ONLY);

  // Bind buffer
  bind->setBinding(POSITION_BINDING, vbuf);

  // set basic white material
  this->setMaterial("BaseWhiteNoLighting");
}

WireBoundingBox::~WireBoundingBox()
{
  OGRE_DELETE mRenderOp.vertexData;
}

void WireBoundingBox::setupBoundingBox(const AxisAlignedBox& aabb)
{
  // init the vertices to the aabb
  setupBoundingBoxVertices(aabb);

  // setup the bounding box of this SimpleRenderable
  setBoundingBox(aabb);
}

//-----------------------------------------------------------------------
void WireBoundingBox::setupBoundingBoxVertices(const AxisAlignedBox& aab)
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

  // fill in the Vertex buffer: 12 lines with 2 endpoints each make up a box
  HardwareVertexBufferSharedPtr vbuf = mRenderOp.vertexData->vertexBufferBinding->getBuffer(POSITION_BINDING);

  auto* pPos = static_cast<float*>(vbuf->lock(HardwareBuffer::HBL_DISCARD));

  // line 0
  *pPos++ = minx;
  *pPos++ = miny;
  *pPos++ = minz;
  *pPos++ = maxx;
  *pPos++ = miny;
  *pPos++ = minz;
  // line 1
  *pPos++ = minx;
  *pPos++ = miny;
  *pPos++ = minz;
  *pPos++ = minx;
  *pPos++ = miny;
  *pPos++ = maxz;
  // line 2
  *pPos++ = minx;
  *pPos++ = miny;
  *pPos++ = minz;
  *pPos++ = minx;
  *pPos++ = maxy;
  *pPos++ = minz;
  // line 3
  *pPos++ = minx;
  *pPos++ = maxy;
  *pPos++ = minz;
  *pPos++ = minx;
  *pPos++ = maxy;
  *pPos++ = maxz;
  // line 4
  *pPos++ = minx;
  *pPos++ = maxy;
  *pPos++ = minz;
  *pPos++ = maxx;
  *pPos++ = maxy;
  *pPos++ = minz;
  // line 5
  *pPos++ = maxx;
  *pPos++ = miny;
  *pPos++ = minz;
  *pPos++ = maxx;
  *pPos++ = miny;
  *pPos++ = maxz;
  // line 6
  *pPos++ = maxx;
  *pPos++ = miny;
  *pPos++ = minz;
  *pPos++ = maxx;
  *pPos++ = maxy;
  *pPos++ = minz;
  // line 7
  *pPos++ = minx;
  *pPos++ = maxy;
  *pPos++ = maxz;
  *pPos++ = maxx;
  *pPos++ = maxy;
  *pPos++ = maxz;
  // line 8
  *pPos++ = minx;
  *pPos++ = maxy;
  *pPos++ = maxz;
  *pPos++ = minx;
  *pPos++ = miny;
  *pPos++ = maxz;
  // line 9
  *pPos++ = maxx;
  *pPos++ = maxy;
  *pPos++ = minz;
  *pPos++ = maxx;
  *pPos++ = maxy;
  *pPos++ = maxz;
  // line 10
  *pPos++ = maxx;
  *pPos++ = miny;
  *pPos++ = maxz;
  *pPos++ = maxx;
  *pPos++ = maxy;
  *pPos++ = maxz;
  // line 11
  *pPos++ = minx;
  *pPos++ = miny;
  *pPos++ = maxz;
  *pPos++ = maxx;
  *pPos++ = miny;
  *pPos++ = maxz;
  vbuf->unlock();
}

//-----------------------------------------------------------------------
Real WireBoundingBox::getSquaredViewDepth(const Camera* cam) const
{
  Vector3 min, max, mid, dist;
  min = mBox.getMinimum();
  max = mBox.getMaximum();
  mid = ((max - min) * 0.5) + min;
  dist = cam->getDerivedPosition() - mid;

  return dist.squaredLength();
}

}  // namespace rviz