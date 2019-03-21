#include "mymath.h"

MyVector MyPoint::operator -( const MyPoint& p1 ) const{
    int vec_x = _x - p1.x();
    int vec_y = _y - p1.y();
    return MyVector( vec_x, vec_y );
}
