#include "rigid2d.hpp"

#include<iostream>
using namespace rigid2d;


using namespace std;
int main(void)
{

Transform2D T_ab, T_bc, T_ba,T_cb,T_ac,T_ca;

Vector2D vec, vec_a, vec_b, vec_c;
Twist2D twi, twi_a, twi_b, twi_c;

cout<<"Start entering T_ab";

operator>>(cin, T_ab);

cout<<"Start entering T_bc";

operator>>(cin, T_bc);

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
//operator>>(cin,twi); add this to the input
char letter ;
cout<< " enter the reference frame for vector ";
cin >> letter;
switch (letter )
{
case 'a':
        vec_a = vec;
        vec_b = T_ba.operator()(vec);
        vec_c = T_ca.operator()(vec);
        //adding the twist here :: twist in a remains the same
        // twist in b and c change 
        // twist in b = adjoint(Tba) * twist a 
        // twist in c = adjoint(tca) * twist a
        // so twist_c = T_ca.operator(twist)

        cout<<"did a ";
    break;
case 'b':
        vec_b = vec;
        vec_a = T_ab.operator()(vec);
        vec_c = T_cb.operator()(vec);
        cout<<"did b "; 
    break;
case 'c':
        vec_c = vec;
        vec_a = T_ac.operator()(vec);
        vec_b = T_bc.operator()(vec);
        cout<<"did c ";
    break;

default:
    break;
}

operator<<(cout,vec_a);
operator<<(cout,vec_b);
operator<<(cout,vec_c);


}