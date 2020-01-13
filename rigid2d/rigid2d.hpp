#ifndef RIGID2D_INCLUDE_GUARD_HPP
#define RIGID2D_INCLUDE_GUARD_HPP
#include <math.h>
#include <iostream>
/// \file
/// \brief Library for two-dimensional rigid body transformations.

//#include<iosfwd> // contains forward definitions for iostream objects

namespace rigid2d
{
 /// \brief PI.  Not in C++ standard until C++20.
    constexpr double PI=3.14159265358979323846;

    /// \brief approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 - a number to compare
    /// \param d2 - a second number to compare
    /// \param epsilon - absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    /// Note: the fabs function in <cmath> (c++ equivalent of math.h) will
    /// be useful here
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
    {
    if(abs(d1-d2) < epsilon)
    {
        return(true);
    }
    else
    {
        return(false);
    }
    
    }

    constexpr double deg2rad(double deg)
    {
        return(deg*PI/180);
    }

     constexpr double rad2deg(double rad)
    {
        return(rad*180/PI);
    }


    static_assert(almost_equal(0, 0), "is_zero failed");
    static_assert(almost_equal(0.001, 0.005, 1.0e-2), "is_zero failed");
    static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");

    static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg) failed");

    static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");

    /// \brief A 2-Dimensional Vector
    struct Vector2D
    {
        double x = 0.0;
        double y = 0.0;
    };

    class Twist2D
    {
        public: 

        double w = 0.0;
        double v_x = 0.0;
        double v_y = 0.0;
        float z1,z2,z5,z6;
        float z3,z4,z7,z8=0;
        float z9,z10,z11,z12,z13,z14,z15,z16;
        void Twist2D::Adjoint(Transform2D & adtf);

    };

    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// os - stream to output to
    /// v - the vector to print

    std::ostream & operator<<(std::ostream & os, const Vector2D & v);


    std::istream & operator>>(std::istream & is, Vector2D & v);



 class Transform2D
    {
    public:
           
    float m1,m2,m3,m4,m5,m6;
    float m7= 0;
    float m8 =0;
    float m9 =1;


    Transform2D();
    explicit Transform2D(const Vector2D & trans);    
    explicit Transform2D(double radians);
    Transform2D(const Vector2D & trans, double radians);


    Vector2D operator()(Vector2D v) const;
    Transform2D inv() const;
    Transform2D & operator*=(const Transform2D & rhs);
    friend std::ostream & operator<<(std::ostream & os, const Transform2D & tf);
     
     };
     
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf);
    std::istream & operator>>(std::istream & is, Transform2D & tf);
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs);



}


#endif