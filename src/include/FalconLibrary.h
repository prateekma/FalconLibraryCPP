#pragma once

#include "fl/Utilities.h"

#include "fl/mathematics/geometry/Pose2d.h"
#include "fl/mathematics/geometry/Pose2dWithCurvature.h"
#include "fl/mathematics/geometry/Rotation2d.h"
#include "fl/mathematics/geometry/Translation2d.h"
#include "fl/mathematics/geometry/Twist2d.h"

#include "fl/mathematics/control/RamseteTracker.h"
#include "fl/mathematics/control/TrajectoryTracker.h"
#include "fl/mathematics/control/PurePursuitTracker.h"

#include "fl/mathematics/spline/ParametricQuinticHermiteSpline.h"
#include "fl/mathematics/spline/ParametricSpline.h"
#include "fl/mathematics/spline/SplineGenerator.h"

#include "fl/mathematics/trajectory/constraints/AngularAccelerationConstraint.h"
#include "fl/mathematics/trajectory/constraints/CentripetalAccelerationConstraint.h"
#include "fl/mathematics/trajectory/constraints/TimingConstraint.h"
#include "fl/mathematics/trajectory/constraints/VelocityLimitRadiusConstraint.h"

#include "fl/mathematics/trajectory/DistanceTrajectory.h"
#include "fl/mathematics/trajectory/IndexedTrajectory.h"
#include "fl/mathematics/trajectory/TimedTrajectory.h"
#include "fl/mathematics/trajectory/Trajectory.h"
#include "fl/mathematics/trajectory/TrajectoryGenerator.h"
#include "fl/mathematics/trajectory/TrajectoryIterator.h"

#include "fl/types/Interpolatable.h"
#include "fl/types/VaryInterpolatable.h"

namespace fl {}