/*! @file MiniCheetah.h
 *  @brief Utility function to build a Mini Cheetah Quadruped object
 *
 * This file is based on MiniCheetahFullRotorModel_mex.m and builds a model
 * of the Mini Cheetah robot.  The inertia parameters of all bodies are
 * determined from CAD.
 *
 */

#ifndef PROJECT_MINICHEETAH_H
#define PROJECT_MINICHEETAH_H

#include "FloatingBaseModel.h"
#include "Quadruped.h"

/*!
 * Generate a Quadruped model of Mini Cheetah
 */
template <typename T>
Quadruped<T> buildMiniCheetah() {
  Quadruped<T> cheetah;
  cheetah._robotType = RobotType::MINI_CHEETAH;

//   cheetah._bodyMass = 3.3;
//   cheetah._bodyLength = 0.230*2;
//   cheetah._bodyWidth = 0.052*2;
//   cheetah._bodyHeight = 0.05 * 2;
//   cheetah._abadGearRatio = 6;
//   cheetah._hipGearRatio = 6;
//   cheetah._kneeGearRatio = 6;
//   cheetah._abadLinkLength = 0.086;
//   cheetah._hipLinkLength = 0.205;
//   cheetah._kneeLinkY_offset = 0;
//   cheetah._kneeLinkLength = 0.205;
//   cheetah._maxLegLength = 0.39;


  cheetah._bodyMass = 4.2;
  cheetah._bodyLength = 0.154*2;
  cheetah._bodyWidth = 0.15*2;
  cheetah._bodyHeight = 0.05 * 2;
  cheetah._abadGearRatio = 7.75;
  cheetah._hipGearRatio = 7.75;
  cheetah._kneeGearRatio = 7.75;
  cheetah._abadLinkLength = 0.092;
  cheetah._hipLinkLength = 0.195;
  cheetah._kneeLinkY_offset = 0;
  cheetah._kneeLinkLength = 0.255;
  cheetah._maxLegLength = 0.450;

  cheetah._motorTauMax = 33.f;
  cheetah._batteryV = 24;
  cheetah._motorKT = .05;  // this is flux linkage * pole pairs
  cheetah._motorR = 0.173;
  cheetah._jointDamping = .01;
  cheetah._jointDryFriction = .2;

  // rotor inertia if the rotor is oriented so it spins around the z-axis
  Mat3<T> rotorRotationalInertiaZ;
  rotorRotationalInertiaZ << 33, 0, 0, 0, 33, 0, 0, 0, 63;
  rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;

  Mat3<T> RY = coordinateRotation<T>(CoordinateAxis::Y, M_PI / 2);
  Mat3<T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
  Mat3<T> rotorRotationalInertiaX =
      RY * rotorRotationalInertiaZ * RY.transpose();
  Mat3<T> rotorRotationalInertiaY =
      RX * rotorRotationalInertiaZ * RX.transpose();

  // spatial inertias
  Mat3<T> abadRotationalInertia;
  abadRotationalInertia << 381, 58, 0.45, 58, 560, 0.95, 0.45, 0.95, 444;
  abadRotationalInertia = abadRotationalInertia * 1e-6;
  Vec3<T> abadCOM(0, 0.036, 0);  // LEFT
  SpatialInertia<T> abadInertia(0.646, abadCOM, abadRotationalInertia);
  Mat3<T> hipRotationalInertia;
  hipRotationalInertia << 1983, 245, 13, 245, 2103, 1.5, 13, 1.5, 408;
  hipRotationalInertia = hipRotationalInertia * 1e-6;
  Vec3<T> hipCOM(0, 0.016, -0.02);
  SpatialInertia<T> hipInertia(1.300, hipCOM, hipRotationalInertia);

  Mat3<T> kneeRotationalInertia, kneeRotationalInertiaRotated;
  kneeRotationalInertiaRotated << 6, 0, 0, 0, 248, 0, 0, 0, 245;
  kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
  kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
  Vec3<T> kneeCOM(0, 0, -0.061);
  SpatialInertia<T> kneeInertia(0.150, kneeCOM, kneeRotationalInertia);

  Vec3<T> rotorCOM(0, 0, 0);
  SpatialInertia<T> rotorInertiaX(0.055, rotorCOM, rotorRotationalInertiaX);
  SpatialInertia<T> rotorInertiaY(0.055, rotorCOM, rotorRotationalInertiaY);

  Mat3<T> bodyRotationalInertia;
  bodyRotationalInertia << 11253, 0, 0, 0, 36203, 0, 0, 0, 42673;
  bodyRotationalInertia = bodyRotationalInertia * 1e-6;
  Vec3<T> bodyCOM(0, 0, 0);
  SpatialInertia<T> bodyInertia(cheetah._bodyMass, bodyCOM,
                                bodyRotationalInertia);

  cheetah._abadInertia = abadInertia;
  cheetah._hipInertia = hipInertia;
  cheetah._kneeInertia = kneeInertia;
  cheetah._abadRotorInertia = rotorInertiaX;
  cheetah._hipRotorInertia = rotorInertiaY;
  cheetah._kneeRotorInertia = rotorInertiaY;
  cheetah._bodyInertia = bodyInertia;

  // locations
  cheetah._abadRotorLocation = Vec3<T>(0.12, 0.05, 0) ;
  cheetah._abadLocation =
      Vec3<T>(cheetah._bodyLength, cheetah._bodyWidth, 0) * 0.5;
  cheetah._hipLocation = Vec3<T>(0, cheetah._abadLinkLength, 0);
  cheetah._hipRotorLocation = Vec3<T>(0, 0.0, 0);//old:(0, 0.04, 0)
  cheetah._kneeLocation = Vec3<T>(0, 0, -cheetah._hipLinkLength);
  cheetah._kneeRotorLocation = Vec3<T>(0, 0, 0);

  return cheetah;
}

#endif  // PROJECT_MINICHEETAH_H
