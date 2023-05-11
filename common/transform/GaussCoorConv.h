#ifndef  _Project_Include
#define  _Project_Include

#include <iostream>
#include <iomanip>
#include <math.h>
#include <stdio.h>
#ifndef BASE_X
#define			BASE_X						(19695588)
#endif

#ifndef BASE_Y
#define			BASE_Y						(3125798)
#endif

void GaussProjCal(double longitude, double latitude, double *X, double *Y, int zone=-1);
void GaussProjInvCal( double X, double Y, double *longitude, double *latitude, int zone=-1);
void GaussProjInvCal_yhl( double X, double Y, double& longitude, double& latitude );

void GaussXS2GaussGeneral(double xsX, double xsY, double& X, double& Y);

void GaussGeneral2GaussXS(double X, double Y, double& xsX, double& xsY);

#endif
