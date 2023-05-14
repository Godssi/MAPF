#include "DynamicObstacle.h"

int DynamicObstacle::hash() const
{
	return x_pos * 1000 + y_pos;
}

bool DynamicObstacle::operator == (const DynamicObstacle& other) const
{
	if (x_pos == other.x_pos && y_pos == other.y_pos)
		return true;
	else
		return false;
}

//bool dynamicobstacle::operator< (const dynamicobstacle& other) const
//{
//	return (this->hash() < other.hash());
//}


string DynamicObstacle::str()
{
	string result = dynamicObstacle_Name;
	return result;
}

string DynamicObstacle::repr()
{
	return str();
}

std::ostream& operator<< (std::ostream& os, const DynamicObstacle& agent)
{
	os << '[' << agent.x_pos << "," << agent.y_pos << ']';
	return os;
}

void DynamicObstacle::Random_Theta() {
	srand(static_cast<unsigned>(time(0)));

	// Random한 이동 방향을 -120 ~ 120도로 설정
	 this->velo_theta = (((double)rand() / RAND_MAX) * 240 - 120) * (3.141592 / 180);	 
}

void DynamicObstacle::Move() {
	x_pos = x_pos + velocity * cos(velo_theta);
	y_pos = y_pos + velocity * sin(velo_theta);
}