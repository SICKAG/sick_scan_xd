//
// Overview for using quaternion and relationship to roll, pitch, yaw
//

#include <stdio.h>
#if __has_include(<tf2/LinearMath/Quaternion.hpp>)
#include <tf2/LinearMath/Quaternion.hpp>
#else
#include <tf2/LinearMath/Quaternion.h>
#endif
#include <tf2/LinearMath/Matrix3x3.h>

#define _USE_MATH_DEFINES

#include <cmath>
#include <iostream>

// see
// [2] https://quaternions.online/


int main(int argc, char *argv[])
{
  double roll, pitch, yaw;

  roll = 0.1; //
  pitch = 0.2;
  yaw = 0.3;

  printf("Quaternion example program\n");


  tf2::Quaternion myQuaternion;

  myQuaternion.setRPY(roll, pitch, yaw);

  char quatOrder[] = "xyzw";
  for (int i = 0; i < 4; i++)
  {
    printf("%c-Quat[%d]: %8.6lf\n", quatOrder[i], i, myQuaternion[i]);
  }

  tf2::Matrix3x3(myQuaternion).getRPY(roll, pitch, yaw);
  for (int i = 0; i < 4; i++)
  {
    printf("%c-Quat[%d]: %8.6lf\n", quatOrder[i], i, myQuaternion[i]);
  }


  const char *axisDescr[] = {"roll", "pitch", "yaw"};

  tf2::Matrix3x3 m(myQuaternion);

  m.getRPY(roll, pitch, yaw);


  printf("Rotationsmatrix\n");

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {

      printf("%10.3lf", m[i][j]);
      // printf(m)
    }
    printf("\n");
  }

  // manual construction

  tf2::Matrix3x3 rotz;
  tf2::Matrix3x3 roty;
  tf2::Matrix3x3 rotx;

  tf2::Matrix3x3 rot;

  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
    {
      rotz[i][j] = 0.0;
      roty[i][j] = 0.0;
      rotx[i][j] = 0.0;
    }

  rotx[0][0] = 1.0;
  roty[1][1] = 1.0;
  rotz[2][2] = 1.0;

  rotz[0][0] = cos(yaw);
  rotz[1][1] = cos(yaw);
  rotz[0][1] = -sin(yaw);
  rotz[1][0] = sin(yaw);

  roty[0][0] = cos(pitch);
  roty[2][2] = cos(pitch);
  roty[0][2] = sin(pitch);
  roty[2][0] = -sin(pitch);

  rotx[1][1] = cos(roll);
  rotx[2][2] = cos(roll);
  rotx[1][2] = -sin(roll);
  rotx[2][1] = sin(roll);

  rot = rotz * roty * rotx;

  const char* reflArr[] = {"rotz","roty","rotx","rot_result"};
  for (int loop = 0; loop < 4; loop++)
  {
    tf2::Matrix3x3 *rotptr = NULL;
    switch (loop)
    {
      case 0:
        rotptr = &rotz;
        break;
      case 1:
        rotptr = &roty;
        break;
      case 2:
        rotptr = &rotx;
        break;
      case 3:
        rotptr = &rot;

    }
    printf("\nName: %s\n==========================\n", reflArr[loop]);
    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 3; j++)
      {

        printf("%10.3lf", (*rotptr)[i][j]);
        // printf(m)
      }
      printf("\n");
    }
  }

  exit(0);
}