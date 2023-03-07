
#ifndef KUKA_KINEMATICS_H
#define KUKA_KINEMATICS_H
#include "matrix.h"

/*Eigen header*/
#include <eigen3/Eigen/Dense>
// #include <eigen3/Eigen/Geometry>
/*Eigen header*/
using namespace Eigen;

//关节DH参数，详见机械臂的DH参数表
#define KUKA_OFFSET_D1 0.34
#define KUKA_OFFSET_D2 0
#define KUKA_OFFSET_D3 0.4
#define KUKA_OFFSET_D4 0
#define KUKA_OFFSET_D5 0.4
#define KUKA_OFFSET_D6 0
// #define KUKA_OFFSET_D7 0.152
#define KUKA_OFFSET_D7  (0.152+FigTip_offset)
const float FigTip_offset=0.19;//以hd上的螺栓为起点 默认夹持时 hd的Y=135

extern const double jointLimit[7];
extern double homeJoints[7];
// const double posQLimit[7]={};
// const double posELimit[6]={};

extern MATRIX_D cameraXYZ;
extern MATRIX_D cameraQuaternion;
extern MATRIX_D cameraRotMat;
extern const char* camera_base_link0;

//初始位置关节角的余弦正弦值
// typedef struct
// {
//   double c[6];
//   double s[6];
// }JOINTLINK;
// typedef struct
// {
// double Jcb[6][6];
// double TrnsJcb[6][6];
// double invJcb[6][6];
// } JACOBIAN;

extern MATRIX_D KUKA_Kinematics(double* joints);
extern MATRIX_D KUKA_Kinematics2(double* joints);
extern int KUKA_Jacobian(double*,MATRIX_D&,MATRIX_D&);
extern MATRIX_D KUKA_InKinematics(double*,double*,double*,double*,bool);//waiting to finish

extern MATRIX_D RotMat2AxisAngle(MATRIX_D rotMat);//旋转矩阵to轴-角
extern MATRIX_D RPY2RotMat(double alpha, double beta, double gamma);//rpy欧拉角  to  旋转矩阵
extern MATRIX_D RotMat2EulerAngle(MATRIX_D rotMat);//旋转矩阵  to  欧拉角
extern MATRIX_D RotMat2Quaternion(MATRIX_D rotMat);
extern MATRIX_D Quaternion2RotMat(double x, double y, double z, double w);
extern MATRIX_D EulerZYX2Quaternion(double alpha, double beta, double gamma);
extern MATRIX_D Quaternion2EulerZYX(double x, double y, double z, double w);

extern void Camera2Link0Trans(const char*);

extern MATRIX_D CalcB0(double alpha, double beta, double gamma);//计算B0
extern int CalcInverse(MATRIX_D &jcb, MATRIX_D &ijcb);//求逆（雅可比矩阵，逆雅可比矩阵）//没啥用
#endif
