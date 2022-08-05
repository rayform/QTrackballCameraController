/****************************************************************************
**
** Copyright (C) 2017 Klaralvdalens Datakonsult AB (KDAB).
** Contact: http://www.qt-project.org/legal
**
** This file is part of the Qt3D module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:LGPL3$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see http://www.qt.io/terms-conditions. For further
** information use the contact form at http://www.qt.io/contact-us.
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 3 as published by the Free Software
** Foundation and appearing in the file LICENSE.LGPLv3 included in the
** packaging of this file. Please review the following information to
** ensure the GNU Lesser General Public License version 3 requirements
** will be met: https://www.gnu.org/licenses/lgpl.html.
**
** GNU General Public License Usage
** Alternatively, this file may be used under the terms of the GNU
** General Public License version 2.0 or later as published by the Free
** Software Foundation and appearing in the file LICENSE.GPL included in
** the packaging of this file. Please review the following information to
** ensure the GNU General Public License version 2.0 requirements will be
** met: http://www.gnu.org/licenses/gpl-2.0.html.
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include "QTrackballCameraController.h"

#include <QVector3D>
#include <Qt3DCore/QTransform>
#include <Qt3DInput/QAction>
#include <Qt3DInput/QActionInput>
#include <Qt3DInput/QAnalogAxisInput>
#include <Qt3DInput/QAxis>
#include <Qt3DInput/QKeyboardDevice>
#include <Qt3DInput/QKeyboardHandler>
#include <Qt3DInput/QLogicalDevice>
#include <Qt3DInput/QMouseDevice>
#include <Qt3DInput/QMouseHandler>
#include <Qt3DLogic/QFrameAction>
#include <Qt3DRender/QCamera>
#include <QtCore/QtGlobal>

#include <cmath>

QT_BEGIN_NAMESPACE

namespace Qt3DExtras
{

namespace
{
QMatrix4x4 viewMatrix(const QMatrix4x4 &worldTransform)
{
  const QVector4D position = worldTransform * QVector4D(0.0f, 0.0f, 0.0f, 1.0f);
  // OpenGL convention is looking down -Z
  const QVector4D viewDirection = worldTransform * QVector4D(0.0f, 0.0f, -1.0f, 0.0f);
  const QVector4D upVector = worldTransform * QVector4D(0.0f, 1.0f, 0.0f, 0.0f);

  QMatrix4x4 m;
  m.lookAt(position.toVector3D(), (position + viewDirection).toVector3D(), upVector.toVector3D());
  return m;
}
} // namespace

QTrackballCameraController::QTrackballCameraController(Qt3DCore::QNode *parent)
    : Qt3DCore::QEntity(parent), camera_(nullptr), leftMouseButtonAction_(new Qt3DInput::QAction()),
      rightMouseButtonAction_(new Qt3DInput::QAction()), altKeyAction_(new Qt3DInput::QAction()),
      wheelAxis_(new Qt3DInput::QAxis()), leftMouseButtonInput_(new Qt3DInput::QActionInput()),
      rightMouseButtonInput_(new Qt3DInput::QActionInput()),
      altKeyInput_(new Qt3DInput::QActionInput()), panXInput_(new Qt3DInput::QAnalogAxisInput()),
      panYInput_(new Qt3DInput::QAnalogAxisInput()),
      wheelXInput_(new Qt3DInput::QAnalogAxisInput()),
      wheelYInput_(new Qt3DInput::QAnalogAxisInput()), mouseDevice_(new Qt3DInput::QMouseDevice()),
      keyboardDevice_(new Qt3DInput::QKeyboardDevice()),
      mouseHandler_(new Qt3DInput::QMouseHandler()),
      logicalDevice_(new Qt3DInput::QLogicalDevice()), frameAction_(new Qt3DLogic::QFrameAction()),
      windowSize_(QSize(1920, 1080)), trackballCenter_(QPoint(0, 0)), trackballRadius_(1.0f),
      panSpeed_(1.0), zoomSpeed_(1.0), rotationSpeed_(1.0), zoomCameraLimit_(1.0)
{
  init();
}

void QTrackballCameraController::init()
{
  //// Actions

  // Left button action (triggers rotation)
  leftMouseButtonInput_->setButtons(QVector<int>() << Qt::LeftButton);
  leftMouseButtonInput_->setSourceDevice(mouseDevice_);
  leftMouseButtonAction_->addInput(leftMouseButtonInput_);

  // Right button action (triggers panning)
  rightMouseButtonInput_->setButtons(QVector<int>() << Qt::RightButton);
  rightMouseButtonInput_->setSourceDevice(mouseDevice_);
  rightMouseButtonAction_->addInput(rightMouseButtonInput_);

  // Alt key (triggers panning)
  altKeyInput_->setButtons(QVector<int>() << Qt::Key_Shift);
  altKeyInput_->setSourceDevice(keyboardDevice_);
  altKeyAction_->addInput(altKeyInput_);

  //// Axes

  // Mouse wheel X
  wheelXInput_->setAxis(Qt3DInput::QMouseDevice::WheelY);
  wheelXInput_->setSourceDevice(mouseDevice_);
  wheelAxis_->addInput(wheelXInput_);

  wheelYInput_->setAxis(Qt3DInput::QMouseDevice::WheelX);
  wheelYInput_->setSourceDevice(mouseDevice_);
  wheelAxis_->addInput(wheelYInput_);

  //// Logical device
  logicalDevice_->addAction(leftMouseButtonAction_);
  logicalDevice_->addAction(rightMouseButtonAction_);
  logicalDevice_->addAction(altKeyAction_);
  logicalDevice_->addAxis(wheelAxis_);

  //// Mouse handler
  Qt3DInput::QMouseHandler *mouseHandler = new Qt3DInput::QMouseHandler();
  mouseHandler->setSourceDevice(mouseDevice_);

  QObject::connect(mouseHandler, &Qt3DInput::QMouseHandler::pressed,
                   [this](Qt3DInput::QMouseEvent *pressedEvent) {
                     pressedEvent->setAccepted(true);
                     this->mouseLastPosition_ = QPoint(pressedEvent->x(), pressedEvent->y());
                     this->mouseCurrentPosition_ = this->mouseLastPosition_;
                     tryToAlign_ = false;
                     xAxisAlignedMove_ = 0;
                     yAxisAlignedMove_ = 0;
                   });

  QObject::connect(
      mouseHandler, &Qt3DInput::QMouseHandler::positionChanged,
      [this](Qt3DInput::QMouseEvent *positionChangedEvent) {
        positionChangedEvent->setAccepted(true);
        this->mouseCurrentPosition_ = QPoint(positionChangedEvent->x(), positionChangedEvent->y());
        tryToAlign_ = positionChangedEvent->modifiers() & Qt3DInput::QMouseEvent::ControlModifier;
      });

  QObject::connect(mouseHandler, &Qt3DInput::QMouseHandler::released,
                   [this](Qt3DInput::QMouseEvent *releasedEvent) {
                     releasedEvent->setAccepted(true);
                     tryToAlign_ = false;
                     xAxisAlignedMove_ = 0;
                     yAxisAlignedMove_ = 0;
                   });

  //// FrameAction

  QObject::connect(frameAction_, &Qt3DLogic::QFrameAction::triggered,
                   [this](float dt) { this->onTriggered(dt); });

  // Disable the logical device when the entity is disabled
  QObject::connect(this, &Qt3DCore::QEntity::enabledChanged, logicalDevice_,
                   &Qt3DInput::QLogicalDevice::setEnabled);

  addComponent(frameAction_);
  addComponent(logicalDevice_);
  addComponent(mouseHandler);
}

bool QTrackballCameraController::xAlignedMove() const
{
  return xAxisAlignedMove_ >= AxisAlignedThreshold;
}

bool QTrackballCameraController::yAlignedMove() const
{
  return yAxisAlignedMove_ >= AxisAlignedThreshold;
}

QVector3D QTrackballCameraController::screenToCamLocal(const QPoint &point, float distance)
{
  float viewportX =
      2.0f * static_cast<float>(point.x()) / static_cast<float>(windowSize().width()) - 1.0f;
  float viewportY =
      2.0f * static_cast<float>(point.y()) / static_cast<float>(windowSize().height()) - 1.0f;
  float worldNearHalfHeight = std::tan((camera_->fieldOfView() * M_PI / 180.0f) * 0.5f);
  float worldNearHalfWidth = worldNearHalfHeight * camera_->aspectRatio();
  QVector3D localCoord(distance * worldNearHalfWidth * viewportX,
                       -distance * worldNearHalfHeight * viewportY, -distance);
  return localCoord;
}

QVector3D QTrackballCameraController::projectScreenToTrackball(const QPoint &screenCoords,
                                                               const QSize &windowSize,
                                                               const QPoint &trackballCenter,
                                                               float trackballRadius) const
{
  float normalizeValue = static_cast<float>(qMin(windowSize.width(), windowSize.height()));
  QVector3D p3D(
      2 * (screenCoords.x() - trackballCenter.x()) / normalizeValue,
      2 * ((windowSize.height() - screenCoords.y()) - trackballCenter.y()) / normalizeValue, 0.0f);

  float r = trackballRadius;
  float z0 = r * 0.5f;

  float z = 0;

  if (r * r - p3D.lengthSquared() >= z0 * z0)
    z = sqrt(r * r - p3D.lengthSquared());

  else
  {
    // Original (hyperbolic):
    // z = r*r / (2 * v.length());

    // Consistent (hyperbolic):
    z = z0 * std::sqrt(r * r - z0 * z0) / p3D.length();
  }

  return QVector3D(p3D[0], p3D[1], z);
}

std::pair<QVector3D, float> QTrackballCameraController::createRotation(
    const QPoint &firstPoint, const QPoint &nextPoint, const QSize &windowSize,
    const QPoint &trackballCenter, const float trackBallRadius) const
{
  QVector3D lastPos3D =
      projectScreenToTrackball(firstPoint, windowSize, trackballCenter, trackBallRadius);
  QVector3D currentPos3D =
      projectScreenToTrackball(nextPoint, windowSize, trackballCenter, trackBallRadius);

  lastPos3D.normalize();
  currentPos3D.normalize();

  QVector3D posAvg = (lastPos3D + currentPos3D) / 2;

  // Compute axis of rotation:
  QVector3D dir = QVector3D::crossProduct(currentPos3D, lastPos3D);

  // Approximate rotation angle:
  float t = dir.length() / posAvg.length();
  float angle = t * M_1_PI * 180.0;

  return std::make_pair(dir, angle);
}

QTrackballCameraController::~QTrackballCameraController()
{
}

Qt3DRender::QCamera *QTrackballCameraController::camera() const
{
  return camera_;
}

void QTrackballCameraController::setCamera(Qt3DRender::QCamera *camera)
{
  if (camera_ != camera)
  {
    camera->blockSignals(true);

    if (camera && !camera->parent())
      camera->setParent(this);

    camera_ = camera;

    camera_->blockSignals(false);

    emit cameraChanged();
  }
}

float QTrackballCameraController::panSpeed() const
{
  return panSpeed_;
}

float QTrackballCameraController::zoomSpeed() const
{
  return zoomSpeed_;
}

float QTrackballCameraController::rotationSpeed() const
{
  return rotationSpeed_;
}

float QTrackballCameraController::zoomCameraLimit() const
{
  return zoomCameraLimit_;
}

float QTrackballCameraController::trackballRadius() const
{
  return trackballRadius_;
}

QPoint QTrackballCameraController::trackballCenter() const
{
  return trackballCenter_;
}

QSize QTrackballCameraController::windowSize() const
{
  return windowSize_;
}

void QTrackballCameraController::setPanSpeed(float v)
{
  if (panSpeed_ != v)
  {
    panSpeed_ = v;
    emit panSpeedChanged();
  }
}

void QTrackballCameraController::setZoomSpeed(float v)
{
  if (zoomSpeed_ != v)
  {
    zoomSpeed_ = v;
    emit zoomSpeedChanged();
  }
}

void QTrackballCameraController::setRotationSpeed(float v)
{
  if (rotationSpeed_ != v)
  {
    rotationSpeed_ = v;
    emit rotationSpeedChanged();
  }
}

void QTrackballCameraController::setZoomCameraLimit(float v)
{
  if (zoomCameraLimit_ != v)
  {
    zoomCameraLimit_ = v;
    emit zoomCameraLimitChanged();
  }
}

void QTrackballCameraController::setTrackballRadius(float v)
{
  if (trackballRadius_ != v)
  {
    trackballRadius_ = v;
    emit trackballRadiusChanged();
  }
}

void QTrackballCameraController::setTrackballCenter(const QPoint &v)
{
  if (trackballCenter_ != v)
  {
    trackballCenter_ = v;
    emit trackballCenterChanged();
  }
}

void QTrackballCameraController::setWindowSize(const QSize &v)
{
  if (windowSize_ != v)
  {
    windowSize_ = v;
    emit windowSizeChanged();
  }
}

void QTrackballCameraController::onTriggered(float)
{
  // Perform the control based on the input received during the frame
  if (camera_ != nullptr)
  {
    QPoint lastPoint = mouseLastPosition_;
    QPoint currentPoint = mouseCurrentPosition_;
    if (tryToAlign_)
    {
      if (xAlignedMove())
      {
        lastPoint.setY(windowSize_.height() / 2);
        currentPoint.setY(windowSize_.height() / 2);
      }
      else if (yAlignedMove())
      {
        lastPoint.setX(windowSize_.width() / 2);
        currentPoint.setX(windowSize_.width() / 2);
      }
      else
      {
        xAxisAlignedMove_ += abs(currentPoint.x() - lastPoint.x());
        yAxisAlignedMove_ += abs(currentPoint.y() - lastPoint.y());
      }
    }

    if (leftMouseButtonAction_->isActive())
    {
      QVector3D rotationAxis;
      float angleDegrees;
      std::tie(rotationAxis, angleDegrees) =
          createRotation(lastPoint, currentPoint, windowSize_, trackballCenter_, trackballRadius_);

      Qt3DCore::QTransform xform;
      xform.setMatrix(viewMatrix(camera_->transform()->matrix()));
      QQuaternion currentRotation = xform.rotation();
      QQuaternion currentRotationInversed = currentRotation.conjugated();

      QVector3D rotatedAxis = currentRotationInversed.rotatedVector(rotationAxis);
      float rotSpeed = (altKeyAction_->isActive() ? 3 : 1) * rotationSpeed_;
      float angle = rotSpeed * angleDegrees;

      camera_->rotateAboutViewCenter(QQuaternion::fromAxisAndAngle(rotatedAxis, angle));
      mouseLastPosition_ = mouseCurrentPosition_;
    }
    else if (rightMouseButtonAction_->isActive())
    {
      float panSpeed = (altKeyAction_->isActive() ? 3 : 1) * panSpeed_;
      float distance = camera_->viewVector().length();
      QVector3D localDelta =
          screenToCamLocal(currentPoint, distance) - screenToCamLocal(lastPoint, distance);
      camera_->translate(panSpeed * QVector3D(-localDelta.x(), -localDelta.y(), 0),
                         Qt3DRender::QCamera::TranslateViewCenter);
      mouseLastPosition_ = mouseCurrentPosition_;
    }
    else
    {
      float zoomSpeed = (altKeyAction_->isActive() ? 3 : 1) * zoomSpeed_;
      float d = (camera_->position() - camera_->viewCenter()).length();

      if (std::fabs(d - zoomSpeed * wheelAxis_->value() - zoomCameraLimit_) < 1e-4f)
        return;

      if (d - zoomSpeed * wheelAxis_->value() < zoomCameraLimit_)
        zoomSpeed = -1 * (zoomCameraLimit_ + 1e-4f - d) / wheelAxis_->value();

      camera_->translate(zoomSpeed * QVector3D(0, 0, wheelAxis_->value()),
                         Qt3DRender::QCamera::DontTranslateViewCenter);
    }
  }
}

} // namespace Qt3DExtras

QT_END_NAMESPACE
