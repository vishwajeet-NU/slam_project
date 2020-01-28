/// \file
/// \brief This file comprises the main chunk of code 
/// which calls all the methods and structures defined 
/// and makes use of them to create a program capable 
/// of tranforming Twists, Vectors and Transformation Matrices

#include"rigid2d/rigid2d.hpp"

#include<iostream>
using namespace rigid2d;


using namespace std;
int main(void)
{

/// \brief create the classes for all necessary transforms
Transform2D T_ab, T_bc, T_ba,T_cb,T_ac,T_ca;

/// \brief create the structures for all necessary vectors

Vector2D vec, vec_a, vec_b, vec_c;

/// \brief create the structures for all necessary twists

Twist2D twi, twi_a, twi_b, twi_c;

int x = T_ab.dummy(2,3);
cout<<x<<"\n";

cout<<"Start entering T_ab \n";

operator>>(cin, T_ab);

cout<<"Start entering T_bc \n";

operator>>(cin, T_bc);

/// \brief apply the methods defined earlier to find all 
/// tranforms in other reference frames
T_ba = T_ab.inv();
T_ac = operator*(T_ab,T_bc ); 
T_cb = T_bc.inv();
T_ca = T_ac.inv();

operator<<(cout,T_ab);
operator<<(cout,T_ba);
operator<<(cout,T_bc);
operator<<(cout,T_cb);
operator<<(cout,T_ac);
operator<<(cout,T_ca);


operator>>(cin, vec);
operator>>(cin,twi); 
char letter ;
cout<< "enter the reference frame for vector:";
cin >> letter;
/// create a switch case for various reference frame definitions 
switch (letter )
{
case 'a':
        vec_a = vec;
        vec_b = T_ba.operator()(vec);
        vec_c = T_ca.operator()(vec);
        twi_a = twi;
        twi_b = T_ba.operator()(twi);
        twi_c = T_ca.operator()(twi);
        
    break;
case 'b':
        vec_b = vec;
        vec_a = T_ab.operator()(vec);
        vec_c = T_cb.operator()(vec);
        twi_b = twi;
        twi_a = T_ab.operator()(twi);
        twi_c = T_cb.operator()(twi);


    break;
case 'c':
        vec_c = vec;
        vec_a = T_ac.operator()(vec);
        vec_b = T_bc.operator()(vec);
        twi_c = twi;
        twi_a = T_ac.operator()(twi);
        twi_b = T_bc.operator()(twi);

    break;

default:
    break;
}
float *add;
add = T_ab.displacement();

cout<<"vector in a \n";
operator<<(cout,vec_a);
cout<<"vector in b \n";
operator<<(cout,vec_b);
cout<<"vector in c \n";
operator<<(cout,vec_c);

cout<<"twist in a \n";
operator<<(cout,twi_a);
cout<<"twist in b \n";
operator<<(cout,twi_b);
cout<<"twist in c \n";
operator<<(cout,twi_c);

cout<<"x = " << *(add)<<" y = " << *(add+1)<<" theta = " << *(add+2);
/// \brief uncomment this if you  want normalized form
//vec_a = vec_a.Normalize();
///vec_b = vec_b.Normalize();
///vec_c = vec_c.Normalize();

///cout<<"normal Va = [" <<vec_a.norm_x<<" "<<vec_a.norm_y<<"]\n";
///cout<<"normal Vb = [" <<vec_b.norm_x<<" "<<vec_b.norm_y<<"]\n";
///cout<<"normal Vc = [" <<vec_c.norm_x<<" "<<vec_c.norm_y<<"]\n";

//cout<<"value of x is = "<< *add <<"    value of y is = "<<*(add+5)<< "    value of theta is = "<<*(add+11);
}
