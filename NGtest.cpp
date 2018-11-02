#include <Eigen\Dense>
// f(x) = a * exp(b*t)
float funtion(float a, float b, float t)
{
	return a * exp(b*t);
}

float derivativeA()
{
	float step = 0.001;

}