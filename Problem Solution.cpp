#include <iostream>
#include <cmath>
using namespace std;
#define PI 3.14159265358979

const double lcx = 1.5;
const double lcy = 1;

struct relativePosition
{
	double x, y;
};

struct relativeVelocity
{
	double x, y;
};

double convertToRadians(double degrees);
relativePosition calcEndEffectorPos(double * l, double * theta, int linknum);
double calcOmegaC(double * theta, double * omega,double thetaC,double * length, relativePosition pos,int linkNum);
double angToLinVelocity(double rad, double length);


int main()
{
	int linkNum;
	double * l;
	double * theta;
	double * omega;
	cout << "Enter number of links: " << endl;
	cin >> linkNum;
	l = new (nothrow) double[linkNum];
	theta = new (nothrow) double[linkNum];
	omega = new (nothrow) double[linkNum];

	cout << "Enter link lengths starting from base: " << endl;
	for (int i = 0; i < linkNum; i++)
	{
		cout << "Enter length " << i << " : ";
		cin >> l[i];
		cout << endl;
	}
	
	cout << "Enter angles starting from base: " << endl;
	for (int j = 0; j < linkNum; j++)
	{
		cout << "Enter angle " << j << " : ";
		cin >> theta[j];
		theta[j] = convertToRadians(theta[j]);
		cout << endl;
	}
	
	cout << "Enter angular velocities starting from base: " << linkNum << endl;
	for (int k = 0; k < linkNum; k++)
	{
		cout << "Enter omega " << k << " : ";
		cin >> omega[k];
		cout << endl;
	}

	relativePosition EEPos = calcEndEffectorPos(l,theta,linkNum);
	
	//cout << EEPos.x << " , " << EEPos.y << endl;
	
	double thetaC = atan2(EEPos.y, EEPos.x); //calculates the angle of the EE wrt the camera
	double omegaC = calcOmegaC(theta,omega,thetaC,l,EEPos,linkNum);
	cout << "Angle from camera perspective, ThetaC: " << thetaC * 180 / PI << " degrees" << endl;
	cout << "Angular speed from camera perspective, OmegaC: " << omegaC <<  " rad/sec" << endl;
	return 0;
}


double convertToRadians(double degrees)
{
	double c = degrees * PI / 180;
	return c;
}

relativePosition calcEndEffectorPos(double * l, double * theta,int linknum)
{
	relativePosition tpos;
	tpos.x = 0;
	tpos.y = 0;
	double angle = theta[0];

	for (int i = 0; i < linknum; i++) 
	{	
		tpos.x += l[i] * sin(angle - PI / 2); //EE position wrt base axis
		tpos.y += l[i] * cos(angle - PI / 2);

		angle += theta[i+1];

	}
	cout << "Angle of End Effector wrt base: " << atan2(tpos.y,tpos.x) * 180 / PI << " degrees" << endl;
	tpos.x = abs(lcx - tpos.x); //EE position wrt camera position
	tpos.y = abs(lcy - tpos.y);
	return tpos;
}

double calcOmegaC(double * theta, double * omega,double thetaC,double * length, relativePosition pos, int linkNum)
{
	relativeVelocity vt;
	vt.x = 0;
	vt.y = 0;	
	double angle = theta[0];
	double velocity = angToLinVelocity(omega[0], length[0]);

	for (int i = 0; i < linkNum; i++)
	{

		vt.x = vt.x + velocity * cos(angle - PI / 2); //EE position wrt base axis
		vt.y += velocity * sin(angle - PI / 2);
		angle += theta[i + 1];
		velocity = angToLinVelocity(omega[i + 1],length[i+1]);

	}

	double lVelEE = vt.x*sin(thetaC);
	// computes the angular velocity from camera's perspective using distance from end effector to camera
	double Wc = lVelEE / (sqrt(pow(pos.x, 2.0) + pow(pos.y, 2.0)));

	return Wc;

}

double angToLinVelocity(double rad,double length)
{
	
	return rad*length;
	
}