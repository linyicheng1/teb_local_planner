#ifndef TEB_TEB_TYPES_H
#define TEB_TEB_TYPES_H
#include <Eigen/Core>
#include <utility>
#include <boost/array.hpp>

struct Header
{
    Header()
            : seq(0)
            , stamp()
            , frame_id()  {
    }
    Header(const Header& h)
            : seq(h.seq)
            , stamp(h.stamp)
            , frame_id(h.frame_id)  {}
    uint32_t seq;
    int stamp;
    int frame_id;

    typedef boost::shared_ptr<Header> Ptr;
    typedef boost::shared_ptr<Header const> ConstPtr;

}; // struct Header_

struct Quaternion
{
    Quaternion()
            : x(0.0)
            , y(0.0)
            , z(0.0)
            , w(0.0)  {}
    Quaternion(double _x,double _y,double _z,double _w)
            : x(_x)
            , y(_y)
            , z(_z)
            , w(_w)  {}
    Quaternion(const Quaternion& q) = default;
    double x;
    double y;
    double z;
    double w;

    typedef boost::shared_ptr< Quaternion> Ptr;
    typedef boost::shared_ptr< Quaternion const> ConstPtr;
}; // struct Quaternion

struct Twist
{
    Twist()
            : linear()
            , angular()  {}
    Twist(Eigen::Vector3f _linear, Eigen::Vector3f _angular)
            : linear(std::move(_linear))
            , angular(std::move(_angular))  {}
    Twist(const Twist &t) = default;
    Eigen::Vector3f linear;
    Eigen::Vector3f angular;

    typedef boost::shared_ptr<Twist > Ptr;
    typedef boost::shared_ptr<Twist const> ConstPtr;

}; // struct Twist_

struct TwistWithCovariance
{
    TwistWithCovariance()
            : twist()
            , covariance()  {
        covariance.assign(0.0);
    }
    TwistWithCovariance(Twist _twist,  boost::array<double, 36> _covariance)
            : twist(_twist)
            , covariance(_covariance)  {
    }
    TwistWithCovariance(const TwistWithCovariance& t) = default;

    Twist twist;
    boost::array<double, 36> covariance;

    typedef boost::shared_ptr< TwistWithCovariance > Ptr;
    typedef boost::shared_ptr< TwistWithCovariance const> ConstPtr;

}; // struct TwistWithCovariance_

struct QuaternionStamped
{
    QuaternionStamped()
            : header()
            , quaternion()  {
    }
    QuaternionStamped(const QuaternionStamped& q)
            : header(q.header)
            , quaternion(q.quaternion)  {
    }
    Header header;
    Quaternion quaternion;

    typedef boost::shared_ptr<QuaternionStamped> Ptr;
    typedef boost::shared_ptr<QuaternionStamped const> ConstPtr;

}; // struct QuaternionStamped_


struct Point32
{

    Point32()
            : x(0.0)
            , y(0.0)
            , z(0.0)  {
    }
    Point32(const Point32& p)
            : x(p.x)
            , y(p.y)
            , z(p.z)  {
    }
    float x;
    float y;
    float z;

    typedef boost::shared_ptr<Point32> Ptr;
    typedef boost::shared_ptr<Point32 const> ConstPtr;

}; // struct Point32_

struct Polygon
{
    Polygon()
            : points()  {
    }
    Polygon(const Polygon& p)
            : points(p.points)  {
    }
    std::vector<Point32> points;

    typedef boost::shared_ptr<Polygon> Ptr;
    typedef boost::shared_ptr<Polygon const> ConstPtr;
}; // struct Polygon_

struct Point
{
    Point()
            : x(0.0)
            , y(0.0)
            , z(0.0)  {
    }
    Point(const Point& p)
            : x(0.0)
            , y(0.0)
            , z(0.0)  {
    }

    double x;
    double y;
    double z;

    typedef boost::shared_ptr<Point> Ptr;
    typedef boost::shared_ptr<Point const> ConstPtr;
}; // struct Point

struct Pose
{
    Pose()
            : position()
            , orientation()  {
    }
    Pose(const Pose& p)
            : position(p.position)
            , orientation(p.orientation)  {
    }

    Point position;
    Quaternion orientation;

    typedef boost::shared_ptr<Pose> Ptr;
    typedef boost::shared_ptr<Pose const> ConstPtr;

}; // struct Pose_

struct PoseStamped
{
    PoseStamped()
            : header()
            , pose()  {
    }
    PoseStamped(const PoseStamped& p)
            : header(p.header)
            , pose(p.pose)  {
    }
    Header header;
    Pose pose;

    typedef boost::shared_ptr<PoseStamped> Ptr;
    typedef boost::shared_ptr<PoseStamped const> ConstPtr;

}; // struct PoseStamped_

#endif //TEB_TEB_TYPES_H
