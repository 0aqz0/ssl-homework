#ifndef MYMATH_H
#define MYMATH_H

class MyVector{
public:
    MyVector(): _x(0), _y(0){}
    MyVector( float x, float y ): _x(x), _y(y){}
    float x ( void ) const { return _x; }
    float y ( void ) const { return _y; }
    MyVector operator*(const float a) const;
    double operator*(const MyVector& v) const;
    void operator=(const MyVector& v);
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
