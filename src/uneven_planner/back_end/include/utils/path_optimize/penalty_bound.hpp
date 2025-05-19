//
// Created by evanlin on 5/28/20.
//

#ifndef MOTIONPLANNER_PENALTYBOUND_HPP
#define MOTIONPLANNER_PENALTYBOUND_HPP

namespace ninebot_algo {

    static constexpr float PENALTY_BOUND_EPSILON = 0.1f;

    /**
     * @brief Linear penalty function for bounding \c var to the interval \f$ -a < var < a \f$
     * @param var The scalar that should be bounded
     * @param a lower and upper absolute bound
     * @param epsilon safety margin (move bound to the interior of the interval)
     * @see penaltyBoundToIntervalDerivative
     * @return Penalty / cost value that is nonzero if the constraint is not satisfied
     */
    inline double penaltyBoundToInterval(const double& var, const double& a,
                                         const double& epsilon) {
        if (var < -a + epsilon) {
            return (-var - (a - epsilon));
        }
        if (var <= a - epsilon) {
            return 0.;
        } else {
            return (var - (a - epsilon));
        }
    }

    /**
     * @brief Linear penalty function for bounding \c var to the interval \f$ a < var < b \f$
     * @param var The scalar that should be bounded
     * @param a lower bound
     * @param b upper bound
     * @param epsilon safety margin (move bound to the interior of the interval)
     * @see penaltyBoundToIntervalDerivative
     * @return Penalty / cost value that is nonzero if the constraint is not satisfied
     */
    inline double penaltyBoundToInterval(const double& var, const double& a, const double& b,
                                         const double& epsilon) {
        if (var < a + epsilon) {
            return (-var + (a + epsilon));
        }
        if (var <= b - epsilon) {
            return 0.;
        } else {
            return (var - (b - epsilon));
        }
    }

    /**
     * @brief Linear penalty function for bounding \c var from below: \f$ a < var \f$
     * @param var The scalar that should be bounded
     * @param a lower bound
     * @param epsilon safety margin (move bound to the interior of the interval)
     * @see penaltyBoundFromBelowDerivative
     * @return Penalty / cost value that is nonzero if the constraint is not satisfied
     */
    inline double penaltyBoundFromBelow(const double& var, const double& a, const double& epsilon) {
        if (var >= a + epsilon) {
            return 0.;
        } else {
            return (-var + (a + epsilon));
        }
    }

    /**
     * @brief Linear penalty function for bounding \c var from below: \f$ a < var \f$
     * @param var The scalar that should be bounded
     * @param a lower bound
     * @param epsilon safety margin (move bound to the interior of the interval)
     * @see penaltyBoundFromBelowDerivative
     * @return Penalty / cost value that is nonzero if the constraint is not satisfied
     */
    inline double penaltyBoundFromAbove(const double& var, const double& a, const double& epsilon) {
        if (var <= a + epsilon) {
            return 0.;
        } else {
            return (var - (a + epsilon));
        }
    }

    /**
     * @brief Derivative of the linear penalty function for bounding \c var to the interval \f$ -a < var < a \f$
     * @param var The scalar that should be bounded
     * @param a lower and upper absolute bound
     * @param epsilon safety margin (move bound to the interior of the interval)
     * @see penaltyBoundToInterval
     * @return Derivative of the penalty function w.r.t. \c var
     */
    inline double penaltyBoundToIntervalDerivative(const double& var, const double& a,
                                                   const double& epsilon) {
        if (var < -a + epsilon) {
            return -1;
        }
        if (var <= a - epsilon) {
            return 0.;
        } else {
            return 1;
        }
    }

    /**
     * @brief Derivative of the linear penalty function for bounding \c var to the interval \f$ a < var < b \f$
     * @param var The scalar that should be bounded
     * @param a lower bound
     * @param b upper bound
     * @param epsilon safety margin (move bound to the interior of the interval)
     * @see penaltyBoundToInterval
     * @return Derivative of the penalty function w.r.t. \c var
     */
    inline double penaltyBoundToIntervalDerivative(const double& var, const double& a,
                                                   const double& b, const double& epsilon) {
        if (var < a + epsilon) {
            return -1;
        }
        if (var <= b - epsilon) {
            return 0.;
        } else {
            return 1;
        }
    }

    /**
     * @brief Derivative of the linear penalty function for bounding \c var from below: \f$ a < var \f$
     * @param var The scalar that should be bounded
     * @param a lower bound
     * @param epsilon safety margin (move bound to the interior of the interval)
     * @see penaltyBoundFromBelow
     * @return Derivative of the penalty function w.r.t. \c var
     */
    inline double penaltyBoundFromBelowDerivative(const double& var, const double& a,
                                                  const double& epsilon) {
        if (var >= a + epsilon) {
            return 0.;
        } else {
            return -1;
        }
    }

    inline double fast_sigmoid(double x) {
        return x / (1 + std::fabs(x));
    }

}  // namespace ninebot_algo

#endif  //MOTIONPLANNER_PENALTYBOUND_HPP
