#ifndef LINE_H
#define LINE_H
#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>
#if 1
static inline void LineBresenham(cv::Point2i start, cv::Point2i end, std::vector<cv::Point2i> &line)
{
    int x1 = start.x;
    int y1 = start.y;
    int x2 = end.x;
    int y2 = end.y;
    int dx = x2 - x1;
    int dy = y2 - y1;
    int ux = (dx > 0) ? 1: - 1;//x的增量方向，取或-1
    int uy = (dy > 0) ? 1: - 1;//y的增量方向，取或-1
    int eps = 0;
    dx = abs(dx);
    dy = abs(dy);
    cv::Point2i pt;
    int x = x1;
    int y = y1;
    if (dx > dy)
    {
        for (x = x1; x != x2; x += ux)
        {
            pt.x = x;
            pt.y = y;
            line.push_back(pt);
            eps += dy;
            if ((eps << 1) >= dx)
            {
                y += uy; eps -= dx;
            }
        }
    }
    else
    {
        for (y = y1; y != y2; y += uy)
        {
            pt.x = x;
            pt.y = y;
            line.push_back(pt);
            eps += dx;
            if ((eps << 1) >= dy)
            {
                x += ux; eps -= dy;
            }
        }
    }
}
#endif

#endif // LINE_H
