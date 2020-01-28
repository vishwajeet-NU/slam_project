#ifndef RIGID2D_INCLUDE_GUARD_HPP
#define RIGID2D_INCLUDE_GUARD_HPP
#include <math.h>
#include <iostream>
#include<iosfwd> // contains forward definitions for iostream objects

/// \file
/// \brief Library for two-dimensional rigid body transformations.


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

    /// \brief convert degrees to radians
    /// \param deg - angle in degrees
    /// \returns radians
    /// NOTE: implement this in the header file
    /// constexpr means that the function can be computed at compile time
    /// if given a compile-time constant as input
   
    
    }

    constexpr double deg2rad(double deg)
    {
        return(deg*PI/180);
    }

    /// \brief convert radians to degrees
    /// \param rad - angle in radians
    /// \returns the angle in degrees
   
     constexpr double rad2deg(double rad)
    {
        return(rad*180/PI);
    }
   constexpr double normalize_angle(double rad)
{
    rad = atan2(sin(rad),cos(rad));
    return rad;
}

    /// static_assertions test compile time assumptions.
    /// You should write at least one more test for each function
    /// You should also purposely (and temporarily) make one of these tests fail
    /// just to see what happens

    static_assert(almost_equal(0, 0), "is_zero failed");
    static_assert(almost_equal(0.001, 0.005, 1.0e-2), "is_zero failed");
    ///new statement 1   
    static_assert(almost_equal(0.0002, 0.0000001, 1.0e-3), "is_zero failed");

//   static_assert(almost_equal(normalize_angle(0),(0),1.0e-2),"normalize failed"); 

    static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");
    
    ///new statement 2
    static_assert(almost_equal(deg2rad(180.0), PI), "deg2rad failed");   
    
    static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg) failed");

    ///new statement 3
    static_assert(almost_equal(rad2deg(PI/2), 90.0), "rad2deg) failed");

    static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");

    /// \brief A 2-Dimensional Vector
    struct Vector2D
    {
        double x = 0.0;
        double y = 0.0;
        double norm_x =0.0;
        double norm_y = 0.0;
        int c;

        Vector2D(); 
        Vector2D(double a, double b);

        ///\brief this function normalizes the vector when called
        /// and changes it self parameters
        /// input = none 
        /// output = self after normal done 
        Vector2D Normalize();
        Vector2D& operator+=(const Vector2D& rhs);
        Vector2D& operator-=(const Vector2D& rhs);
        double operator*=(const Vector2D& rhs);
        double length();
        double distance(Vector2D vec, Vector2D vec1);
        double angle();


    };
   Vector2D operator+(Vector2D lhs, const Vector2D& rhs);
   Vector2D operator-(Vector2D lhs, const Vector2D& rhs);
   double operator*(Vector2D lhs, const Vector2D& rhs);

    /// \brief A 2-Dimensional Twist
    struct Twist2D
    {
        double w = 0.0;
        double v_x = 0.0;
        double v_y = 0.0;
    
    };

    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// os - stream to output to
    /// v - the vector to print

    std::ostream & operator<<(std::ostream & os, const Vector2D & v);

    ///\brief outputs a 2 dimensional twist( w_z, V_x, V_y) to the output stream
    /// os - stream to output to
    /// Twist - twist to print  
    std::ostream & operator<<(std::ostream & os, const Twist2D & Twist);

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as two numbers
    ///   separated by a newline or a space, or entered as [xcomponent ycomponent]
    /// is - stream from which to read
    /// v [out] - output vector
    /// Hint: The following may be useful:
    /// https://en.cppreference.com/w/cpp/io/basic_istream/peek
    /// https://en.cppreference.com/w/cpp/io/basic_istream/get

    std::istream & operator>>(std::istream & is, Vector2D & v);


    /// \brief takes in a twist from the user
    /// is - stream to read twist from 
    /// tv_in - output twist
    std::istream & operator>>(std::istream & is, Twist2D & tv_in);



 class Transform2D
    {
    public:
    /// \brief m variables represent matrix number of a SE(2) transform matrix
    //// z variables represnet members of a 6X6 Adjoint       
    float m1,m2,m3,m4,m5,m6;
    float container[3];
    float m7= 0;
    float m8 =0;
    float m9 =1;
    float z1 = 1;
    float z2= 0;
    float z3 = 0;
    float z4 = 0;
    float z7 =0;
    float z5,z6,z8,z9;
    int cad;

    int dummy(int aa, int bb);
    /// \brief Create an identity transformation

    Transform2D();
    
    /// \brief create a transformation that is a pure translation
    /// \param trans - the vector by which to translate
    
    explicit Transform2D(const Vector2D & trans);    
    
    /// \brief create a pure rotation
    /// \param radians - angle of the rotation, in radians
    explicit Transform2D(double radians);
    
    /// \brief Create a transformation with a translational and rotational
    /// component
    /// \param trans - the translation
    /// \param rot - the rotation, in radians
 
    Transform2D(const Vector2D & trans, double radians);

    /// \brief Creates Adjoint representation 
    void Adjoint();

    /// \brief apply a transformation to a Vector2D
    /// \param v - the vector to transform
    /// \return a vector in the new coordinate system
    Vector2D operator()(Vector2D v) const;
    
    /// \brief applies a tranformation to a Twist2D
    /// \param tv - the twist to tranform
    /// \return a twist in the new coordinate system
    Twist2D operator()(Twist2D tv) const;

    /// \brief invert the transformation
    /// \return the inverse transformation. 
    Transform2D inv() const;
    
    /// \brief compose this transform with another and store the result 
    /// in this object
    /// \param rhs - the first transform to apply
    /// \returns a reference to the newly transformed operator

    Transform2D & operator*=(const Transform2D & rhs);
    
    
    /// \brief \see operator<<(...) (declared outside this class)
    /// for a description
    friend std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

    float* displacement();
    
    Transform2D integrateTwist(Twist2D &twist);

     
     };

    
    /// \brief should print a human readable version of the transform:
    /// An example output:
    /// dtheta (degrees): 90 dx: 3 dy: 5
    /// \param os - an output stream
    /// \param tf - the transform to print 
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf);


    /// \brief Read a transformation from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
    std::istream & operator>>(std::istream & is, Transform2D & tf);

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs);


}


#endif