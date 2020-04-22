#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <boost/optional/optional_io.hpp>

#include <iostream>

class GPSPose3Factor: public gtsam::NoiseModelFactor1<gtsam::Pose3> {

private:
  // measurement information
  gtsam::Point3 nT_;

public:

  /**
   * Constructor
   * @param poseKey    associated pose varible key
   * @param model      noise model for GPS snesor, in X-Y-Z
   * @param m          Point3 measurement
   */
  GPSPose3Factor(gtsam::Key poseKey, const gtsam::Point3 m, gtsam::SharedNoiseModel model) :
      gtsam::NoiseModelFactor1<gtsam::Pose3>(model, poseKey), nT_(m) {}

  // error function
  // @param p    the pose in Pose3
  // @param H    the optional Jacobian matrix, which use boost optional and has default null pointer
  gtsam::Vector evaluateError(const gtsam::Pose3& p, boost::optional<gtsam::Matrix&> H = boost::none) const {
    
    // return error vector
    gtsam::Vector vector3(3);
    vector3 = p.translation(H) - nT_;
    // Nulling error in Z, as we don't have altitude data
    vector3[2] = 0.0;
    return vector3;
  }

};
