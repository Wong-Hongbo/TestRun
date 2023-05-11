//如果使用行深高斯坐标，请不要更改相应的参数
#include <assert.h>
#include "Covproject.h"

//ѡ��54����ϵͳ��84����ϵͳ
//��ʼ����Ϊ54,SunSnow2008.3
int  CoordSysParam = 54;

#define  AngleToRangle     3.14159265358979323846
#define  PSEC   206264.80624709635515647

static double   Co=AngleToRangle/180.0; 

static double  	C0=6367558.49687;
static double  	C1=32005.7801;
static double  	C2=133.9213;
static double  	C3= 0.7032;

static double  	K0=1.57046064172E-7;
static double  	K1=5.051773759E-3;
static double  	K2=2.9837302E-5;
static double  	K3=2.38189E-7;

static double  	a54=6378245.00000000;    
static double  	b54=6356863.01877305;  
static double   f54=1/298.3;                
static double  	e254=6.693421622966E-3;    
static double  	e_254=6.738525414683E-3;   

static double  	a84=6378137.00000000;      
static double  	b84=6356752.31424518;       
static double   f84=1/298.257223563;       
static double  	e284=6.69437999E-3;        
static double  	e_284=6.739496742E-3;       

void MeridianABCDE(double e2,double &A,double &B,double &C,
  double &D,double &E,double &F,double &G)
{
	A = 1+0.75*e2+45.0/64.0*e2*e2+175.0/256.0*e2*e2*e2+11025.0/16384.0*e2*e2*e2*e2;
	A += 43659.0/65536.0*pow(e2,5)+693693.0/1048576.0*pow(e2,6);
	B = 0.375*e2+15.0/32.0*e2*e2+525.0/1024.0*e2*e2*e2+2205.0/4096.0*e2*e2*e2*e2;
	B += 72765.0/131072.0*pow(e2,5)+297297.0/524288.0*pow(e2,6);
	C = 15.0/256.0*e2*e2+105.0/1024.0*e2*e2*e2+2205.0/16384.0*e2*e2*e2*e2;
	C += 10395.0/65536.0*pow(e2,5)+1486485.0/8388608.0*pow(e2,6);
	D = 35.0/3072.0*e2*e2*e2+105.0/4096.0*e2*e2*e2*e2;
	D += 10395.0/262144.0*pow(e2,5)+55055.0/1048576.0*pow(e2,6);
	E = 315.0/131072.0*pow(e2,4)+3465.0/524288.0*pow(e2,5)+99099.0/8388608.0*pow(e2,6);
	F = 639.0/1310720.0*pow(e2,5)+9009.0/5242880.0*pow(e2,6);
	G = 1001.0/8388608.0*pow(e2,6);
}

double dMeridian_X(double a,double e2,double dLat)
{
	double A,B,C,D,E,F,G,X;

	MeridianABCDE(e2,A,B,C,D,E,F,G);
	X = a*(1-e2)*(A*dLat-B*sin(2*dLat)+C*sin(4*dLat)-D*sin(6*dLat)
		+E*sin(8*dLat)-F*sin(10*dLat)+G*sin(12*dLat));

	return X;
}

double dMeridian_Bf(double a,double e2,double X)
{
	double A, B, C, D, E, F, G, B0, Bf;

	MeridianABCDE(e2,A,B,C,D,E,F,G);
	Bf=X/(a*(1-e2))/A;
	do{
		B0=Bf;
		Bf=X/a/(1-e2)+B*sin(2*Bf)-C*sin(4*Bf)+D*sin(6*Bf)
			-E*sin(8*Bf)+F*sin(10*Bf)-G*sin(12*Bf);
		Bf=Bf/A;
	}while(fabs(Bf-B0)>1e-13);

	return Bf;
}

void Gaussxy(double a,double e2,double B,double L,double L0,
		 double &x,double &y)
{
	double N, X, l, tB, nB;

	L=(L<0.0)? L+2.0*AngleToRangle:L;
	l=L-L0*AngleToRangle/180;   
	X=dMeridian_X(a,e2,B); 
	N=a/sqrt(1-e2*sin(B)*sin(B));
	tB=tan(B);
	nB=sqrt(e2/(1.0-e2))*cos(B);

	x = X+N*sin(B)*cos(B)*l*l/2
		+N*sin(B)*pow(cos(B),3)*(5.0-tB*tB+9.0*nB*nB+4.0*pow(nB,4))*pow(l,4)/24.0
		+N*sin(B)*pow(cos(B),5)*(61.0-58.0*tB*tB+pow(tB,4))*pow(l,6)/720.0;
	y = N*cos(B)*l+N*pow(cos(B),3)*(1.0-tB*tB+nB*nB)*pow(l,3)/6.0
		+N*pow(cos(B),5)*(5.0-18.0*tB*tB+pow(tB,4)+14.0*nB*nB-58.0*tB*tB*nB*nB)
		*pow(l,5)/120.0;

}

void GaussBL(double a,double e2,double x,double y,double L0,
             double &B,double &L)
{
	double t,n,b,l,M,N,Bf;

	Bf=dMeridian_Bf(a,e2,x); 
	L0=L0*AngleToRangle/180;
	M=a*(1-e2)/pow(sqrt(1-e2*sin(Bf)*sin(Bf)),3);
	N=a/sqrt(1-e2*sin(Bf)*sin(Bf));
	t=tan(Bf);
	n=sqrt(e2/(1.0-e2))*cos(Bf);

	b=-t*y*y/2/M/N
		+t*pow(y,4)/24/M/pow(N,3)*(5+3*t*t+n*n-9*t*t*n*n)
		-t*pow(y,6)/720/M/pow(N,5)*(61+90*t*t+45*pow(t,4));
	l=y/N/cos(Bf)
		-pow(y,3)/6/pow(N,3)/cos(Bf)*(1+2*t*t+n*n)
		+pow(y,5)/120/pow(N,5)/cos(Bf)*(5+28*t*t+24*pow(t,4)+6*n*n+8*t*t*n*n);

	B=Bf+b;
	L=L0+l;
	if(fabs(B*PSEC) < 5e-5)  B=fabs(B);
	if(fabs(L*PSEC) < 5e-5)  L=fabs(L);
}

int BLtoXY(double B, double L, double L0, double& X, double& Y)
{
    B = B * Co;
    L = L * Co;

    double  a = a54;
    double  e2 = e254;

    if( CoordSysParam == 84 )
    {
        a = a84;  e2 = e284;
    }

    Gaussxy(a, e2, B, L, L0, X, Y);

    Y += 500000.0;

    return 1;
}

int XYtoBL(double X, double Y, double L0, double& B, double& L)
{
    Y -= 500000.0;

    double  a = a54;
    double  e2 = e254;

    if( CoordSysParam == 84 )
    {
        a = a84;  e2 = e284;
    }

    GaussBL(a, e2, X, Y, L0, B, L);

    B = B / Co;
    L = L / Co;

    return 1;
}

double NtoL(int N)
{
    int  l = (N <= 30) ? N*6-3 : (N-60)*6-3;
    return (double)l;
}

int LtoN(double L)
{
    if(L < 0)  L += 360;
    return ((int)(L/6) + 1);
}

double LOCM(double L)
{
    return NtoL(LtoN(L));
}

void GaussProjCal(double longitude, double latitude, double *X, double *Y, int zone)
{
    int ProjNo;
    double X0, Y0, xval,yval;

	if (-1 == zone)
		ProjNo = LtoN(longitude);
    else
        ProjNo = zone;
	
	BLtoXY(latitude, longitude , NtoL(ProjNo), yval,  xval);
	xval = xval-500000L;

	X0 = 1000000L*(ProjNo)+500000L; 
	Y0 = 0;
    xval = xval+X0; 
	yval = yval+Y0;
	
	xval -= BASE_X;
	yval -= BASE_Y;

    *X = xval;    *Y = yval;

	//ԭ���Ĵ���ת���㷨���뱣���Զ��գ�
/*
    int ProjNo=0;
    int ZoneWide;
    double longitude1,latitude1, longitude0,latitude0, X0,Y0, xval,yval;
    double a,f, e2,ee, NN, T,C,A, M, iPI;
	
    iPI = 0.0174532925199433; ////3.1415926535898/180.0;
    ZoneWide = 6;
    a=6378245.0; f=1.0/298.3;  //54�걱������ϵ����
	//	a=6378140.0; f=1/298.257; //80����������ϵ����
	//	a=6378137;f=1/298.257223563;  //WGS-84����ϵ
	
	if (-1 == zone)
		ProjNo = (int)(longitude/ZoneWide)+1;//ProjNo=(int)(longitude/ZoneWide);
    else
        ProjNo = zone;
	
    //longitude0 = ProjNo*ZoneWide+ZoneWide/2;
	longitude0 = ProjNo*ZoneWide-ZoneWide/2; //���뾭��
    longitude0 = longitude0*iPI ;
    latitude0=0;
    longitude1 = longitude*iPI ; 
    latitude1 = latitude * iPI ; 
	
    e2=2*f-f*f;
    ee=e2*(1.0-e2);
    NN=a/sqrt(1.0-e2*sin(latitude1)*sin(latitude1)); T=tan(latitude1)*tan(latitude1); C=ee*cos(latitude1)*cos(latitude1);
    A=(longitude1-longitude0)*cos(latitude1);
    M=a*((1-e2/4-3*e2*e2/64-5*e2*e2*e2/256)*latitude1-(3*e2/8+3*e2*e2/32+45*e2*e2*e2/1024)*sin(2*latitude1)	
		+(15*e2*e2/256+45*e2*e2*e2/1024)*sin(4*latitude1)-(35*e2*e2*e2/3072)*sin(6*latitude1));
    xval = NN*(A+(1-T+C)*A*A*A/6+(5-18*T+T*T+72*C-58*ee)*A*A*A*A*A/120); yval = M+NN*tan(latitude1)*(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24 +(61-58*T+T*T+600*C-330*ee)*A*A*A*A*A*A/720);
	//X0 = 1000000L*(ProjNo+1)+500000L; Y0 = 0;
	X0 = 1000000L*(ProjNo)+500000L; Y0 = 0;
    xval = xval+X0; yval = yval+Y0;
	
	xval -= BASE_X;
	yval -= BASE_Y;
	
    *X = xval;    *Y = yval;
*/
}

void GaussProjInvCal( double X, double Y, double *longitude, double *latitude, int zone )
{
    int ProjNo;
	double X0, Y0, xval, yval;
	
	if (-1 ==  zone)
        ProjNo = (int)(X/1000000L) ; 
    else
        ProjNo = zone;
	
    X0 = ProjNo*1000000L+500000L;
	Y0 = 0;
    xval = X-X0;
    yval = Y-Y0; 

	XYtoBL(yval, xval+500000L,  NtoL(ProjNo), *latitude, *longitude);
/*
	//ԭ���Ĵ���ת���㷨���뱣���Զ��գ�
	int ProjNo;
    int ZoneWide; ////����
    double longitude1,latitude1, longitude0, X0,Y0, xval,yval; 
	double e1,e2,f,a, ee, NN, T,C, M, D,R,u,fai, iPI;
	
    iPI = 0.0174532925199433; //3.1415926535898/180.0;
    a=6378245.0; f=1.0/298.3; //54�걱������ϵ����
	// a=6378140.0; f=1/298.257; //80����������ϵ����
	// a=6378137;f=1/298.257223563;  //WGS-84����ϵ
	
    ZoneWide = 6; //6�ȴ���
	if (-1 ==  zone)
        ProjNo = (int)(X/1000000L) ; //���Ҵ���
    else
        ProjNo = zone;
	
    //longitude0 = (ProjNo-1) * ZoneWide + ZoneWide / 2; 
	longitude0 = ProjNo*ZoneWide - ZoneWide/2;//���뾭��
	longitude0 = longitude0 * iPI ;
    X0 = ProjNo*1000000L+500000L;
	Y0 = 0;
	
    xval = X-X0;
    yval = Y-Y0; //���ڴ�������
    e2 = 2*f-f*f;
    e1 = (1.0-sqrt(1-e2))/(1.0+sqrt(1-e2)); ee = e2/(1-e2); M = yval;
    u = M/(a*(1-e2/4-3*e2*e2/64-5*e2*e2*e2/256));
    fai = u+(3*e1/2-27*e1*e1*e1/32)*sin(2*u)+(21*e1*e1/16-55*e1*e1*e1*e1/32)*sin(4*u)
        +(151*e1*e1*e1/96)*sin(6*u)+(1097*e1*e1*e1*e1/512)*sin(8*u); C = ee*cos(fai)*cos(fai); T = tan(fai)*tan(fai);
    NN = a/sqrt(1.0-e2*sin(fai)*sin(fai));
    R = a*(1-e2)/sqrt((1-e2*sin(fai)*sin(fai))*(1-e2*sin(fai)*sin(fai))*(1-e2*sin(fai)*sin(fai))); D = xval/NN;
	
    //���㾭��(Longitude) γ��(Latitude)
    longitude1 = longitude0+(D-(1+2*T+C)*D*D*D/6+(5-2*C+28*T-3*C*C+8*ee+24*T*T)*D*D*D*D*D/120)/cos(fai);
    latitude1 = fai -(NN*tan(fai)/R)*(D*D/2-(5+3*T+10*C-4*C*C-9*ee)*D*D*D*D/24	
        +(61+90*T+298*C+45*T*T-256*ee-3*C*C)*D*D*D*D*D*D/720); //ת��Ϊ�� DD
    *longitude = longitude1 / iPI; *latitude = latitude1 / iPI;
	*/
}

void GaussXS2GaussGeneral(double xsX, double xsY, double& X, double& Y)
{
	int ProjNo;
	double xval, yval;
	double X0, Y0;

	//�ӻػ�׼��
	xval = xsX + BASE_X; 
	yval = xsY + BASE_Y;

	//�ӻ���Ϊ����������ƫ����
	double dd = (xval-500000L)/1000000L;
	ProjNo = (int)(dd+0.5);
	X0 = 1000000L*(ProjNo)+500000L;
	Y0 = 0;
    xval = xval-X0; 
	yval = yval-Y0;

	//X��Y����������Y����ƫ��500KM
	X = yval; 
	Y = xval+500000L;
}

void GaussGeneral2GaussXS(double X, double Y, double& xsX, double& xsY)
{
	int ProjNo;
	double xval, yval;
	double X0, Y0;

	//X��Y����������Y����ƫ��500KM
	xval = Y-500000L;
	yval = X;

	//��ȥ��Ϊ����������ƫ����
	double dd = (xval + BASE_X-500000L)/1000000L;
	ProjNo = (int)(dd+0.5);
	X0 = 1000000L*(ProjNo+1)+500000L;
	Y0 = 0;
	xval = xval+X0;
	yval = yval+Y0;

	xsX = xval - BASE_X;
	xsY = yval - BASE_Y;
}

