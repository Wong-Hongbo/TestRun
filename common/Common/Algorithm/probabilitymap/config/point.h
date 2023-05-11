#ifndef POINT_H
#define POINT_H

/*结构体点*/
template <class T>
struct point{
    inline point():x(0),y(0) {}
    inline point(T _x, T _y):x(_x),y(_y){}
    T x, y;
};
typedef point<int> IntPoint;
typedef struct {
  int     num_points;
  IntPoint*  points;
} GridLineTraversalLine;


#endif // POINT_H
