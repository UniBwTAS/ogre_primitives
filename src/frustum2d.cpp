#include <OgreCamera.h>
#include <OgreHardwareBufferManager.h>
#include <OgreSimpleRenderable.h>
#include <OgreMaterialManager.h>

#include <ogre_primitives/frustum2d.h>

namespace ogre_primitives
{
#define POSITION_BINDING 0

Frustum2D::Frustum2D(bool fill) : radius_(0), fill_(fill), SimpleRenderable()
{
  initCircularSectorGround();
}

Frustum2D::Frustum2D(const String& name, bool fill) : radius_(0), fill_(fill), SimpleRenderable(name)
{
  initCircularSectorGround();
}

Frustum2D::~Frustum2D()
{
  delete mRenderOp.vertexData;
}

void Frustum2D::initCircularSectorGround()
{
  mRenderOp.vertexData = new VertexData();

  mRenderOp.indexData = nullptr;
  mRenderOp.vertexData->vertexCount = fill_ ? 3 : 4;
  mRenderOp.vertexData->vertexStart = 0;
  mRenderOp.operationType = fill_ ? RenderOperation::OT_TRIANGLE_STRIP : RenderOperation::OT_LINE_STRIP;
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
  auto material = Ogre::MaterialManager::getSingleton().getByName("BaseWhiteNoLighting");
  this->setMaterial(material);
}

void Frustum2D::setParameters(float angle_left, float angle_right, float range, float height)
{
  float x_left = std::tan(angle_left) * range;
  float x_right = std::tan(angle_right) * range;

  radius_ = range;

  HardwareVertexBufferSharedPtr vbuf = mRenderOp.vertexData->vertexBufferBinding->getBuffer(POSITION_BINDING);

  auto* pPos = static_cast<float*>(vbuf->lock(HardwareBuffer::HBL_DISCARD));

  *pPos++ = 0;        // x
  *pPos++ = -height;  // y
  *pPos++ = 0;        // z
  *pPos++ = x_right;  // x
  *pPos++ = -height;  // y
  *pPos++ = range;    // z
  *pPos++ = x_left;   // x
  *pPos++ = -height;  // y
  *pPos++ = range;    // z
  if (!fill_)
  {
    *pPos++ = 0;        // x
    *pPos++ = -height;  // y
    *pPos++ = 0;        // z
  }
  vbuf->unlock();

  setBoundingBox(Ogre::AxisAlignedBox(std::min(x_left, 0.f), -height, 0, std::max(x_right, 0.f), -height, range));
}

//-----------------------------------------------------------------------
Real Frustum2D::getSquaredViewDepth(const Camera* cam) const
{
  Vector3 min, max, mid, dist;
  min = mBox.getMinimum();
  max = mBox.getMaximum();
  mid = ((max - min) * 0.5) + min;
  dist = cam->getDerivedPosition() - mid;

  return dist.squaredLength();
}

}  // namespace ogre_primitives