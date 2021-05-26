#ifndef TEB_POSE_SE2_H
#define TEB_POSE_SE2_H

#include <g2o/stuff/misc.h>
#include <Eigen/Core>
#include "misc.h"
#include "teb_types.h"

namespace teb_local_planner
{

/**
  * @class PoseSE2
  * @brief This class implements a pose in the domain SE2: \f$ \mathbb{R}^2 \times S^1 \f$
  * The pose consist of the position x and y and an orientation given as angle theta [-pi, pi].
  */
    class PoseSE2
    {
    public:

        /** @name Construct PoseSE2 instances */
        ///@{

        /**
          * @brief Default constructor
          */
        PoseSE2()
        {
            setZero();
        }

        /**
          * @brief Construct pose given a position vector and an angle theta
          * @param position 2D position vector
          * @param theta angle given in rad
          */
        PoseSE2(const Eigen::Ref<const Eigen::Vector2d>& position, double theta)
        {
            _position = position;
            _theta = theta;
        }

        /**
          * @brief Construct pose using single components x, y, and the yaw angle
          * @param x x-coordinate
          * @param y y-coordinate
          * @param theta yaw angle in rad
          */
        PoseSE2(double x, double y, double theta)
        {
            _position.coeffRef(0) = x;
            _position.coeffRef(1) = y;
            _theta = theta;
        }
        /**
         * @brief Construct pose using a geometry_msgs::Pose
         * @param pose geometry_msgs::Pose object
         */
        PoseSE2(const Pose& pose)
        {
            _position.coeffRef(0) = pose.position.x;
            _position.coeffRef(1) = pose.position.y;
            Eigen::Quaterniond q{pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z};
            Eigen::Vector3d eulerAngle = q.matrix().eulerAngles(2,1,0);
            _theta = eulerAngle[0];
        }
        /**
          * @brief Copy constructor
          * @param pose PoseSE2 instance
          */
        PoseSE2(const PoseSE2& pose)
        {
            _position = pose._position;
            _theta = pose._theta;
        }

        ///@}


        /**
          * @brief Destructs the PoseSE2
          */
        ~PoseSE2() {}


        /** @name Access and modify values */
        ///@{

        /**
          * @brief Access the 2D position part
          * @see estimate
          * @return reference to the 2D position part
          */
        Eigen::Vector2d& position() {return _position;}

        /**
          * @brief Access the 2D position part (read-only)
          * @see estimate
          * @return const reference to the 2D position part
          */
        const Eigen::Vector2d& position() const {return _position;}

        /**
          * @brief Access the x-coordinate the pose
          * @return reference to the x-coordinate
          */
        double& x() {return _position.coeffRef(0);}

        /**
          * @brief Access the x-coordinate the pose (read-only)
          * @return const reference to the x-coordinate
          */
        const double& x() const {return _position.coeffRef(0);}

        /**
          * @brief Access the y-coordinate the pose
          * @return reference to the y-coordinate
          */
        double& y() {return _position.coeffRef(1);}

        /**
          * @brief Access the y-coordinate the pose (read-only)
          * @return const reference to the y-coordinate
          */
        const double& y() const {return _position.coeffRef(1);}

        /**
          * @brief Access the orientation part (yaw angle) of the pose
          * @return reference to the yaw angle
          */
        double& theta() {return _theta;}

        /**
          * @brief Access the orientation part (yaw angle) of the pose (read-only)
          * @return const reference to the yaw angle
          */
        const double& theta() const {return _theta;}

        /**
          * @brief Set pose to [0,0,0]
          */
        void setZero()
        {
            _position.setZero();
            _theta = 0;
        }


        /**
         * @brief Return the unit vector of the current orientation
         * @returns [cos(theta), sin(theta))]^T
         */
        Eigen::Vector2d orientationUnitVec() const {return Eigen::Vector2d(std::cos(_theta), std::sin(_theta));}

        ///@}


        /** @name Arithmetic operations for which operators are not always reasonable */
        ///@{

        /**
          * @brief Scale all SE2 components (x,y,theta) and normalize theta afterwards to [-pi, pi]
          * @param factor scale factor
          */
        void scale(double factor)
        {
            _position *= factor;
            _theta = g2o::normalize_theta( _theta*factor );
        }

        /**
          * @brief Increment the pose by adding a double[3] array
          * The angle is normalized afterwards
          * @param pose_as_array 3D double array [x, y, theta]
          */
        void plus(const double* pose_as_array)
        {
            _position.coeffRef(0) += pose_as_array[0];
            _position.coeffRef(1) += pose_as_array[1];
            _theta = g2o::normalize_theta( _theta + pose_as_array[2] );
        }

        /**
          * @brief Get the mean / average of two poses and store it in the caller class
          * For the position part: 0.5*(x1+x2)
          * For the angle: take the angle of the mean direction vector
          * @param pose1 first pose to consider
          * @param pose2 second pose to consider
          */
        void averageInPlace(const PoseSE2& pose1, const PoseSE2& pose2)
        {
            _position = (pose1._position + pose2._position)/2;
            _theta = g2o::average_angle(pose1._theta, pose2._theta);
        }

        /**
          * @brief Get the mean / average of two poses and return the result (static)
          * For the position part: 0.5*(x1+x2)
          * For the angle: take the angle of the mean direction vector
          * @param pose1 first pose to consider
          * @param pose2 second pose to consider
          * @return mean / average of \c pose1 and \c pose2
          */
        static PoseSE2 average(const PoseSE2& pose1, const PoseSE2& pose2)
        {
            return PoseSE2( (pose1._position + pose2._position)/2 , g2o::average_angle(pose1._theta, pose2._theta) );
        }

        /**
          * @brief Rotate pose globally
          *
          * Compute [pose_x, pose_y] = Rot(\c angle) * [pose_x, pose_y].
          * if \c adjust_theta, pose_theta is also rotated by \c angle
          * @param angle the angle defining the 2d rotation
          * @param adjust_theta if \c true, the orientation theta is also rotated
          */
        void rotateGlobal(double angle, bool adjust_theta=true)
        {
            double new_x = std::cos(angle)*_position.x() - std::sin(angle)*_position.y();
            double new_y = std::sin(angle)*_position.x() + std::cos(angle)*_position.y();
            _position.x() = new_x;
            _position.y() = new_y;
            if (adjust_theta)
                _theta = g2o::normalize_theta(_theta+angle);
        }

        ///@}


        /** @name Operator overloads / Allow some arithmetic operations */
        ///@{

        /**
          * @brief Asignment operator
          * @param rhs PoseSE2 instance
          * @todo exception safe version of the assignment operator
          */
        PoseSE2& operator=( const PoseSE2& rhs )
        {
            if (&rhs != this)
            {
                _position = rhs._position;
                _theta = rhs._theta;
            }
            return *this;
        }

        /**
          * @brief Compound assignment operator (addition)
          * @param rhs addend
          */
        PoseSE2& operator+=(const PoseSE2& rhs)
        {
            _position += rhs._position;
            _theta = g2o::normalize_theta(_theta + rhs._theta);
            return *this;
        }

        /**
        * @brief Arithmetic operator overload for additions
        * @param lhs First addend
        * @param rhs Second addend
        */
        friend PoseSE2 operator+(PoseSE2 lhs, const PoseSE2& rhs)
        {
            return lhs += rhs;
        }

        /**
          * @brief Compound assignment operator (subtraction)
          * @param rhs value to subtract
          */
        PoseSE2& operator-=(const PoseSE2& rhs)
        {
            _position -= rhs._position;
            _theta = g2o::normalize_theta(_theta - rhs._theta);
            return *this;
        }

        /**
        * @brief Arithmetic operator overload for subtractions
        * @param lhs First term
        * @param rhs Second term
        */
        friend PoseSE2 operator-(PoseSE2 lhs, const PoseSE2& rhs)
        {
            return lhs -= rhs;
        }

        /**
          * @brief Multiply pose with scalar and return copy without normalizing theta
          * This operator is useful for calculating velocities ...
          * @param pose pose to scale
          * @param scalar factor to multiply with
          * @warning theta is not normalized after multiplying
          */
        friend PoseSE2 operator*(PoseSE2 pose, double scalar)
        {
            pose._position *= scalar;
            pose._theta *= scalar;
            return pose;
        }

        /**
          * @brief Multiply pose with scalar and return copy without normalizing theta
          * This operator is useful for calculating velocities ...
          * @param scalar factor to multiply with
          * @param pose pose to scale
          * @warning theta is not normalized after multiplying
          */
        friend PoseSE2 operator*(double scalar, PoseSE2 pose)
        {
            pose._position *= scalar;
            pose._theta *= scalar;
            return pose;
        }

        /**
           * @brief Output stream operator
           * @param stream output stream
           * @param pose to be used
           */
        friend std::ostream& operator<< (std::ostream& stream, const PoseSE2& pose)
        {
            stream << "x: " << pose._position[0] << " y: " << pose._position[1] << " theta: " << pose._theta;
            return stream;
        }

        ///@}


    private:

        Eigen::Vector2d _position;
        double _theta;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };


} // namespace teb_local_planner

#endif //TEB_POSE_SE2_H
