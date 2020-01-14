/// \file
/// \brief This file defines the methods that have been 
/// initialized in the header file.


#include "rigid2d.hpp"
#include<iostream>
#include<math.h>

namespace rigid2d
{

/// \brief Member functions definitions
/// for more information on each function, refer the comments
/// in header file


 std::ostream & operator<<(std::ostream & os, const Vector2D & v)

 {

os<<"["<<v.x <<"\t"<< v.y<<"] \n";

return os ;     
 }

std::ostream & operator<<(std::ostream & os, const Twist2D & tv)

 {

os<<"["<<tv.w <<"\t"<< tv.v_x<<"\t"<<tv.v_y<<"] \n";

return os ;     
 }


std::ostream & operator<<(std::ostream & os, const Transform2D & tf)

{
Transform2D out_1;
out_1 = tf;
double ang;
ang = acos(out_1.m1);
ang = rad2deg(ang);
os<<"degrees = "  << ang << " dx = "  << out_1.m3 <<" dy = "<< out_1.m6 << "\n"; 

return os;
}



std::istream & operator>>(std::istream & is, Transform2D & tf)
{
Vector2D vect;    
double angle_1;



std::cout<< "input an angle (degrees):";
is >> angle_1;
angle_1 = deg2rad(angle_1);
std::cout<< "input dx:";
is >> vect.x;

std::cout<< "input dy:";
is >> vect.y;

Transform2D buffer_TT(vect,angle_1);

tf = buffer_TT;
tf.Adjoint();
return is;

}




std::istream & operator>>(std::istream & is, Vector2D &v)
{
//Vector2D vector_2;    
std::cout<< "input x component:";  
is >> v.x;
std::cout<< "input y component:";
is >> v.y;

return is;
}

//add this to friend so that it can access the variables in transform
std::istream & operator>>(std::istream & is, Twist2D & tv)
{
std::cout<< "input w component of twist:";  
is >> tv.w;

std::cout<< "input v_x component of twist:";
is >> tv.v_x;

std::cout<< "input v_y component of twist:";
is >> tv.v_y;

return is;

}

Transform2D::Transform2D()
{

m1 = 1;
m2 = 0;
m3 = 0;
m4 = 0;
m5 = 1;
m6 = 0;
m7 =0;
m8 =0;
m9 =1;
}

Transform2D::Transform2D(const Vector2D & trans)
{

m1 = 1;
m2 = 0;
m3 = trans.x;
m4 = 0;
m5 = 1;
m6 = trans.y;
m7 =0;
m8 =0;
m9 =1;


}

Transform2D::Transform2D(double radians)
{
m1 = cos(radians);
m2 = -sin(radians);
m3 = 0;
m4 = sin(radians);
m5 = cos(radians);
m6 = 0;
m7 =0;
m8 =0;
m9 =1;


}
 
Transform2D::Transform2D(const Vector2D & trans, double radians) 
{
m1 = cos(radians);
m2 = -sin(radians);
m3 = trans.x;
m4 = sin(radians);
m5 = cos(radians);
m6 = trans.y;
m7 =0;
m8 =0;
m9 =1;

}
/// the new function for normalizing the vector
Vector2D Vector2D::Normalize(){

    double magnitude;
    magnitude = sqrt(x*x + y*y);
    norm_x = x/magnitude;
    norm_y = y/magnitude;
    return *this;
}
Vector2D Transform2D::operator()(Vector2D v) const {
Vector2D VEC;
VEC.x = m1*v.x + m2*v.y + m3;
VEC.y = m4*v.x + m5*v.y + m6;
return VEC;
}

Twist2D Transform2D::operator()(Twist2D tv) const {

Twist2D Twist;
Twist.w  = z15* tv.w;
Twist.v_x = z21*tv.w+ z22*tv.v_x + z23* tv.v_y;
Twist.v_y = z27*tv.w+ z28 *tv.v_x + z29 * tv.v_y;

return Twist;

}

void Transform2D::Adjoint()
{

z1 = m1;
z2 = m2;
z7 = m4;
z8 = m5;
z21 =m6;
z22 = m1;
z23 = m2;
z25 = -m6*m1-m3*m4;
z26 = -m6*m2-m3*m5;
z27 =-m3;
z28 = m4;
z29 = m5;

}

Transform2D Transform2D::inv() const
{

Transform2D T2;

T2.m1 = m1;
T2.m2 = m4;
T2.m3 = -1*(m1*m3 + m4 * m6);
T2.m4 = m2;
T2.m5 = m5;
T2.m6 = -1* (m2 * m3 + m5 * m6);
T2.m7 = m7;
T2.m8 = m8;
T2.m9 = m9;
T2.Adjoint();

return T2;

}

Transform2D &Transform2D::operator*=(const Transform2D & rhs)
{

Transform2D Some_vec;
Some_vec = rhs;
float m1_buffer = m1;
float m2_buffer = m2;
float m3_buffer = m3;
float m4_buffer = m4;
float m5_buffer = m5;
float m6_buffer = m6;



m1 = m1_buffer*Some_vec.m1 + m2_buffer*Some_vec.m4+m3_buffer*Some_vec.m7;
m2 = m1_buffer*Some_vec.m2 + m2_buffer*Some_vec.m5+m2_buffer*Some_vec.m8;
m3 = m1_buffer*Some_vec.m3 + m2_buffer*Some_vec.m6+m3_buffer*Some_vec.m9;

m4 = m4_buffer*Some_vec.m1 + m5_buffer*Some_vec.m4+  m6_buffer*Some_vec.m7;
m5 = m4_buffer*Some_vec.m2 + m5_buffer*Some_vec.m5 + m6_buffer*Some_vec.m8;
m6 = m4_buffer*Some_vec.m3 + m5_buffer*Some_vec.m6 + m6_buffer*Some_vec.m9;

m7 = Some_vec.m7;
m8 = Some_vec.m8;
m9 = Some_vec.m9;

return *this;
}
 
Transform2D operator*(Transform2D lhs, const Transform2D & rhs)

{


Transform2D buffer_result;
buffer_result= lhs.operator*=(rhs);
//std::cout<<buffer_result.m1;
return buffer_result;
}

}