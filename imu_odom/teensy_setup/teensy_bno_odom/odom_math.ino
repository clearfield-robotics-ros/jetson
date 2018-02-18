float vx = 0;
float vz = 0;
float base_width = 1.10515; //wheel base width in meters, CHANGE THIS

void get_odom(float left_v, float right_v)
{
  vx = (left_v + right_v) / 2;
  vz = (right_v - left_v) / base_width;
}

