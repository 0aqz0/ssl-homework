#include "mymath.h"
#include <iostream>

MyVector MyPoint::operator -( const MyPoint& p ) const{
    float vec_x = _x - p.x();
    float vec_y = _y - p.y();
    return MyVector( vec_x, vec_y );
}

MyVector MyVector::operator *( const float a ) const{
    float vec_x = _x * a;
    float vec_y = _y * a;
    return MyVector( vec_x, vec_y );
}

MyVector MyVector::operator -( const MyVector& v ) const{
    float vec_x = _x - v.x();
    float vec_y = _y - v.y();
    return MyVector( vec_x, vec_y );
}

MyVector MyVector::operator +( const MyVector& v ) const{
    float vec_x = _x + v.x();
    float vec_y = _y + v.y();
    return MyVector( vec_x, vec_y );
}

MyVector MyVector::operator /( const float a ) const{
    float vec_x = _x / a;
    float vec_y = _y / a;
    return MyVector( vec_x, vec_y );
}


float MyVector::operator *( const MyVector& v ) const{
    float result = v.x() * _x + v.y() * _y;
    return result;
}

void MyVector::operator =( const MyVector& v ){
    _x = v.x();
    _y = v.y();
}

void MyPoint::operator =( const MyPoint& p ){
    _x = p.x();
    _y = p.y();
}

MyVector MyVector::Unitization(){
    float vec_x = _x / this->mod();
    float vec_y = _y / this->mod();
    return MyVector( vec_x, vec_y );
}
