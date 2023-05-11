#ifndef  _Project_Include
#define  _Project_Include

#include <iostream>
#include <iomanip>
#include <math.h>
#include <stdio.h>

#define			BASE_X						(19695588)
#define			BASE_Y						(3125798)

//为了避免同一个站点范围内出现跨越带号而导致的高斯坐标跳变问题，
//在高斯经纬度转换函数中加入一个带号参数zone。默认情况下，如果不出现跨带问题，就不用传这个参数。
//如果在同一个站点范围内可能出现跨带现象，则可以将该站点大部分区域所属的带号传进去进行带号固定，
//以避免可能出现的高斯坐标数据跳变现象。
//经纬度转行深高斯
void GaussProjCal(double longitude, double latitude, double *X, double *Y, int zone=-1);
//行深高斯转经纬度
void GaussProjInvCal( double X, double Y, double *longitude, double *latitude, int zone=-1);

//以下两个函数用于公司与第三方单位进行沟通时进行坐标转换使用
//如果第三方使用通用高斯坐标，可以通过这两个函数进行数据交流和检验
//行深高斯转通用高斯
void GaussXS2GaussGeneral(double xsX, double xsY, double& X, double& Y);
//通用高斯转行深高斯
void GaussGeneral2GaussXS(double X, double Y, double& xsX, double& xsY);

#endif
