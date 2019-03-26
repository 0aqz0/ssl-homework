#ifndef MYMATH_H
#define MYMATH_H

#include <cmath>

class MyVector{
public:
    MyVector(): _x(0), _y(0){}
    MyVector( float x, float y ): _x(x), _y(y){}
    float x ( void ) const { return _x; }
    float y ( void ) const { return _y; }
    float mod() { return sqrt( _x * _x + _y * _y ); }
    MyVector operator*( const float a ) const;
    MyVector operator-( const MyVector& v ) const;
    MyVector operator+( const MyVector& v ) const;
    MyVector operator/( const float a ) const;
    float operator*( const MyVector& v ) const;
    void operator=( const MyVector& v );
    MyVector Unitization();
private:
    float _x;
    float _y;
};

class MyPoint{
public:
    MyPoint(): _x(0), _y(0) {}
    MyPoint( float x, float y ): _x(x), _y(y){}
    float x( void ) const { return _x; }
    float y( void ) const { return _y; }
    void Setx ( float x ) { _x = x; }
    void Sety ( float y ) { _y = y; }
    MyVector operator-(const MyPoint& p) const;
    void operator=(const MyPoint& p);
private:
    float _x;
    float _y;
};

class Node {
public:
    Node(int x, int y,int parent=-1, double cost=0) {
        this->x = x;
        this->y = y;
        this->parent = parent;
        cost = 0.0;
    }
    int x, y;
    int parent;
    double cost;
};

#endif // MYMATH_H
