#include <cmath>
#include <OgreCamera.h>
#include <OgreHardwareBufferManager.h>
#include <OgreSimpleRenderable.h>
#include <OgreMaterialManager.h>

#include <rviz/ogre_primitives/cone2d.h>

namespace rviz
{
#define POSITION_BINDING 0

Cone2D::Cone2D(bool fill, float range_min) : radius_(0), fill_(fill), range_min_(range_min), SimpleRenderable()
{
  initCircularSectorGround();
}

Cone2D::Cone2D(const String& name, bool fill, float range_min)
  : radius_(0), fill_(fill), range_min_(range_min), SimpleRenderable(name)
{
  initCircularSectorGround();
}

Cone2D::~Cone2D()
{
  delete mRenderOp.vertexData;
}

void Cone2D::initCircularSectorGround()
{
  mRenderOp.vertexData = new VertexData();

  mRenderOp.indexData = nullptr;
  int num_vertices = 3;
  if (!fill_)
    ++num_vertices;
  if (range_min_ > 0)
    ++num_vertices;
  mRenderOp.vertexData->vertexCount = num_vertices;
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

void Cone2D::setParameters(float angle_min, float angle_max, float range)
{
  float sin_angle_min = std::sin(angle_min);
  float cos_angle_min = std::cos(angle_min);
  float sin_angle_max = std::sin(angle_max);
  float cos_angle_max = std::cos(angle_max);
  float x_min = cos_angle_min * range;
  float y_min = sin_angle_min * range;
  float x_max = cos_angle_max * range;
  float y_max = sin_angle_max * range;

  radius_ = range;

  HardwareVertexBufferSharedPtr vbuf = mRenderOp.vertexData->vertexBufferBinding->getBuffer(POSITION_BINDING);

  auto* pPos = static_cast<float*>(vbuf->lock(HardwareBuffer::HBL_DISCARD));

  if (range_min_ == 0)
  {
    *pPos++ = 0;      // x
    *pPos++ = 0;      // y
    *pPos++ = 0;      // z
    *pPos++ = x_min;  // x
    *pPos++ = y_min;  // y
    *pPos++ = 0;      // z
    *pPos++ = x_max;  // x
    *pPos++ = y_max;  // y
    *pPos++ = 0;      // z
    if (!fill_)
    {
      *pPos++ = 0;  // x
      *pPos++ = 0;  // y
      *pPos++ = 0;  // z
    }
  }
  else
  {
    float x_min_range_min = cos_angle_min * range_min_;
    float y_min_range_min = sin_angle_min * range_min_;
    float x_max_range_min = cos_angle_max * range_min_;
    float y_max_range_min = sin_angle_max * range_min_;

    if (fill_)
    {
      *pPos++ = x_min;            // x
      *pPos++ = y_min;            // y
      *pPos++ = 0;                // z
      *pPos++ = x_max;            // x
      *pPos++ = y_max;            // y
      *pPos++ = 0;                // z
      *pPos++ = x_min_range_min;  // x
      *pPos++ = y_min_range_min;  // y
      *pPos++ = 0;                // z
      *pPos++ = x_max_range_min;  // x
      *pPos++ = y_max_range_min;  // y
      *pPos++ = 0;                // z
    }
    else
    {
      *pPos++ = x_min;            // x
      *pPos++ = y_min;            // y
      *pPos++ = 0;                // z
      *pPos++ = x_max;            // x
      *pPos++ = y_max;            // y
      *pPos++ = 0;                // z
      *pPos++ = x_max_range_min;  // x
      *pPos++ = y_max_range_min;  // y
      *pPos++ = 0;                // z
      *pPos++ = x_min_range_min;  // x
      *pPos++ = y_min_range_min;  // y
      *pPos++ = 0;                // z
      *pPos++ = x_min;            // x
      *pPos++ = y_min;            // y
      *pPos++ = 0;                // z
    }
  }
  vbuf->unlock();

  setBoundingBox(Ogre::AxisAlignedBox(std::min(std::min(0.f, x_min), x_max), std::min(std::min(0.f, y_min), y_max),
                                      -0.1f, std::max(std::max(0.f, x_min), x_max),
                                      std::max(std::max(0.f, y_min), y_max), 0.1f));
}

//-----------------------------------------------------------------------
Real Cone2D::getSquaredViewDepth(const Camera* cam) const
{
  Vector3 min, max, mid, dist;
  min = mBox.getMinimum();
  max = mBox.getMaximum();
  mid = ((max - min) * 0.5) + min;
  dist = cam->getDerivedPosition() - mid;

  return dist.squaredLength();
}

}  // namespace rviz