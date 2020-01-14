Q.1 What is the difference between a class and a struct in C++

Ans : 
The member variables and methods inside a struct are public, while they are private 
by default in classes. By declaring the public keyword they can be made private

Q.2 Why is Vector2D a struct and Transform2DClass?

Guideline 1:
Use class if the class has an invariant; use struct if the data members can vary independently:

The vector2D struct is for defining the components of a vector. These are not interdependent and hence can be defined
inside of a class.

The Transform2DClass on the other hand, has interdependencies. For example, terms inside the 'rotation matrix' of a 
transformation, are interdependt( sin(theta), cos(theta)). Thus using a class is valid.

Guideline 2: Ease of comprehension : 
Transforms and vectors are different physical entities, and thus grouping the in their indvidual structres makes
the program better. The critical difference between structs and classes are their default scope definitions. Thus if by default the conents can be public, grouping vectors with a struct makes more sense. 
For the structure surrounding transforms, declaring a class is easier, plus it also fits in typcial format which states that if member functions are needed a class might be more familiar over a struct.


Q.3 Why are some of the constructors in Transform2D explicit?

A constructor with a single parameter, implicitly assumes declaration of a variable to the class type, if it us 
undefined. This can cause confusing results. For eg the pure rotation transform uses explicit, to prevent implicit 
type declaration, which which will lead to wrong result. Thus it is a good bug prevention enforcement standard. 

Q.4:
maths: sqrt(x*x + y*y) = mag
normalized out = x/mag , y/mag
design 1: change the Vector2D to a class. Declare a constructor which will normalize the vector by default, and store
the normalized vector into another class of the same type
This method seems unnecessary because we might not have to normalize a vector, and so it will take up memory and processing everyime a class is created, while also creating a class containing the normal vector

design 2: Create a member function inside the struct Vector2D which when called will return the normalized values
of the vector. :: I prefer this method and have implelemted the same in point 5

design 3:Create a member function inside the Transfrom2D class such that whenever a vector is input to the istream it
gets called, and thus creates a normalized vector class. 

Q.6 Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?

Both of the above functions are member functioins of the class.It is good practise to mark a member function as const
unless it modifies the object. Thus the operator*=() is marked const. The inv() function, however, modifies the members of the class, and returns a new class(which is the inverted transform). Thus it needs to have this ability, which declaring const will impair. 