#ifndef MYMATH_H
#define MYMATH_H

class MyVector{
public:
    MyVector(): _x(0), _y(0){}
    MyVector( int x, int y ): _x(x), _y(y){}
    int x ( void ) const { return _x; }
    int y ( void ) const { return _y; }
private:
    int _x;
    int _y;
};

class MyPoint{
public:
    MyPoint(): _x(0), _y(0) {}
    MyPoint( int x, int y ): _x(x), _y(y){}
    int x( void ) const { return _x; }
    int y( void ) const { return _y; }
    void Setx ( int x ) { _x = x; }
    void Sety ( int y ) { _y = y; }
    MyVector operator-(const MyPoint& p1) const;
private:
    int _x;
    int _y;
};

#endif // MYMATH_H
