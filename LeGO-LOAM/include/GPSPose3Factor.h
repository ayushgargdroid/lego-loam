#ifndef _GPSPOSE3FACTOR_LIDAR_ODOMETRY_H_
#define _GPSPOSE3FACTOR_LIDAR_ODOMETRY_H_

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>

namespace gtsamfactor {


class GPSPose3Factor: public gtsam::NoiseModelFactor1<gtsam::Pose3> {

private:
  // measurement information
  double mx_, my_;

public:

  /**
   * Constructor
   * @param poseKey    associated pose varible key
   * @param model      noise model for GPS snesor, in X-Y
   * @param m          Point2 measurement
   */
  GPSPose3Factor(gtsam::Key poseKey, const gtsam::Point2 m, gtsam::SharedNoiseModel model) :
      gtsam::NoiseModelFactor1<gtsam::Pose3>(model, poseKey), mx_(m.x()), my_(m.y()) {}

  // error function
  // @param p    the pose in Pose3
  // @param H    the optional Jacobian matrix, which use boost optional and has default null pointer
  gtsam::Vector evaluateError(const gtsam::Pose3& p, boost::optional<gtsam::Matrix&> H = boost::none) const {
  
    // note that use boost optional like a pointer
    // only calculate jacobian matrix when non-null pointer exists
    if (H) *H = (gtsam::Matrix66() << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                      0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished();
    
    // return error vector
    gtsam::Vector6 err;
    err << p.x() - mx_, p.y() - my_, 0.0, 0.0, 0.0, 0.0;
    return err;
  }

};

} // namespace gtsamfactor

#endif