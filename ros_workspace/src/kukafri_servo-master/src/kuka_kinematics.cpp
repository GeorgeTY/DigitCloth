
#include "kuka_kinematics.h"

//init some global variable
const double jointLimit[7]={169,119,169,119,169,119,174};
double homeJoints[7]={0,0,0,0,0,0,0};
MATRIX_D cameraXYZ=MatD31(0,0,0);
MATRIX_D cameraQuaternion=MatD41(0,0,0,1);
MATRIX_D cameraRotMat=Eye(3);
const char* camera_base_link0="right";


MATRIX_D RPY2RotMat(double alpha, double beta, double gamma)//欧拉角alpha、beta、gamma  转换为  旋转矩阵
{
	MATRIX_D rotMat=Zeros(3,3);//生成3*3 的0矩阵

	rotMat=Rot(2,cos(gamma),sin(gamma))
			* Rot(1,cos(beta),sin(beta))
			* Rot(0,cos(alpha),sin(alpha));//z-y-x顺序旋转，得到旋转矩阵

	return rotMat;

	}

MATRIX_D RotMat2EulerAngle(MATRIX_D rotMat)//旋转矩阵  转换为  欧拉角alpha、beta、gamma
{
	MATRIX_D EulerAngle=Zeros(3,1);
	double alpha, beta,gamma, r11,r13,r12, r21, r22, r23, r31, r32, r33;
	r31=rotMat(3,1); r11=rotMat(1,1);
	r21=rotMat(2,1); r12=rotMat(1,2);
	r22=rotMat(2,2); r13=rotMat(1,3);
	r23=rotMat(2,3); 
	r32=rotMat(3,2); r33=rotMat(3,3);

	//XYZ固定角坐标系欧拉角 or ZYX动坐标系欧拉角
	// if (r31>0.9999)
	// {
	// 	beta=-M_PI/2;
	// 	alpha=0;
	// 	gamma=atan2(-r23,r22);
	// }
	// else if (r31<-0.999)
	// {
	// 	beta=M_PI/2;
	// 	alpha=0;
	// 	gamma=atan2(-r23,r22);
	// }
	// else
	// { 
	// 	beta=asin(-r31);
	// 	alpha=atan2(r21/cos(beta),r11/cos(beta));
	//     gamma=atan2(r32/cos(beta),r33/cos(beta));
	// }

	// Matrix3d rotation_matrix= Matrix3d::Identity();
	// Vector3d eular_angles = rotation_matrix.eulerAngles();
	// Quaterniond q=Quaterniond();

	gamma=atan2(r21,r11);
	beta=atan2(-r31,r11*cos(gamma)+r21*sin(gamma));
	alpha=atan2(r13*sin(gamma)-r23*cos(gamma),-r12*sin(gamma)+r22*cos(gamma));

	EulerAngle=MatD31(alpha,beta,gamma);
	return EulerAngle;
}

MATRIX_D RotMat2Quaternion(MATRIX_D rotMat)
{
	double x, y, z, w;
    double trace = rotMat(1,1) + rotMat(2,2) + rotMat(3,3);
	double epsilon=1E-12;
	if( trace > epsilon ){
            double s = 0.5 / sqrt(trace + 1.0);
            w = 0.25 / s;
            x = ( rotMat(3,2) - rotMat(2,3) ) * s;
            y = ( rotMat(1,3) - rotMat(3,1) ) * s;
            z = ( rotMat(2,1) - rotMat(1,2) ) * s;
	}else{
		if ( rotMat(1,1) > rotMat(2,2) && rotMat(1,1) > rotMat(3,3) ){
			double s = 2.0 * sqrt( 1.0 + rotMat(1,1) - rotMat(2,2) - rotMat(3,3));
			w = (rotMat(3,2) - rotMat(2,3) ) / s;
			x = 0.25 * s;
			y = (rotMat(1,2) + rotMat(2,1) ) / s;
			z = (rotMat(1,3) + rotMat(3,1) ) / s;
		} else if (rotMat(2,2) > rotMat(3,3)) {
			double s = 2.0 * sqrt( 1.0 + rotMat(2,2) - rotMat(1,1) - rotMat(3,3));
			w = (rotMat(1,3) - rotMat(3,1) ) / s;
			x = (rotMat(1,2) + rotMat(2,1) ) / s;
			y = 0.25 * s;
			z = (rotMat(2,3) + rotMat(3,2) ) / s;
		}else {
			double s = 2.0 * sqrt( 1.0 + rotMat(3,3) - rotMat(1,1) - rotMat(2,2) );
			w = (rotMat(2,1) - rotMat(1,2) ) / s;
			x = (rotMat(1,3) + rotMat(3,1) ) / s;
			y = (rotMat(2,3) + rotMat(3,2) ) / s;
			z = 0.25 * s;
		}
	}
	MATRIX_D quaternion=MatD41(x,y,z,w);  

	return quaternion;
}
MATRIX_D Quaternion2RotMat(double x, double y, double z, double w)
{
	double x2, y2, z2, w2;
    x2 = x*x;  y2 = y*y; z2 = z*z;  w2 = w*w;

    return  MatD33( w2+x2-y2-z2, 2*x*y-2*w*z, 2*x*z+2*w*y,
                	2*x*y+2*w*z, w2-x2+y2-z2, 2*y*z-2*w*x,
                    2*x*z-2*w*y, 2*y*z+2*w*x, w2-x2-y2+z2 );
}
MATRIX_D EulerZYX2Quaternion(double alpha, double beta, double gamma)
{
	double x,y,z,w;
	w=cos(gamma/2)*cos(beta/2)*cos(alpha/2)-sin(gamma/2)*sin(beta/2)*sin(alpha/2);
	x=cos(gamma/2)*cos(beta/2)*sin(alpha/2)+sin(gamma/2)*sin(beta/2)*cos(alpha/2);
	y=cos(gamma/2)*sin(beta/2)*cos(alpha/2)-sin(gamma/2)*cos(beta/2)*sin(alpha/2);
	z=sin(gamma/2)*cos(beta/2)*cos(alpha/2)+cos(gamma/2)*sin(beta/2)*sin(alpha/2);

	MATRIX_D quaternion=MatD41(x,y,z,w);
	return quaternion;
}
MATRIX_D Quaternion2EulerZYX(double x, double y, double z, double w)
{
	double alpha,beta,gamma;
	alpha=atan2(2*(w*x+y*z),1-2*(x*x+y*y));
	beta=asin(2*(w*y-x*z));
	gamma=atan2(2*(w*z+x*y),1-2*(y*y+z*z));

	MATRIX_D euler=MatD31(alpha,beta,gamma);
	return euler;
}

MATRIX_D CalcB0(double alpha, double beta, double gamma) //  欧拉角速度　到　末端角速度　的转换矩阵
{

	MATRIX_D B0 = MatD33 (cos(beta)*cos(gamma),  -sin(gamma),   0.0,
			              cos(beta)*sin(gamma),   cos(gamma),   0.0,
						       -sin(beta),           0.0,       1.0);
	return B0;

}

MATRIX_D KUKA_Kinematics(double* joints)
{  
	MATRIX_D T01(4,4),T12(4,4),T23(4,4),T34(4,4),T45(4,4),T56(4,4),T67(4,4);
	MATRIX_D T07(4,4);
	MATRIX_D RotM=MatD33(0,0,0,0,0,0,0,0,0);
	MATRIX_D handori=MatD31(0,0,0);
	MATRIX_D handpos=MatD31(0,0,0);
	// double handori[3];
	// double handpos[3];

	T01=(Rot4(2,cos(joints[0]),sin(joints[0]))
			*D4(KUKA_OFFSET_D1)
			*Rot4(0,cos(-M_PI/2),sin(-M_PI/2)));
	T12=(Rot4(2,cos(joints[1]),sin(joints[1]))
			*D4(KUKA_OFFSET_D2)
			*Rot4(0,cos(M_PI/2),sin(M_PI/2)));
	T23=(Rot4(2,cos(joints[2]),sin(joints[2]))
			*D4(KUKA_OFFSET_D3)
			*Rot4(0,cos(-M_PI/2),sin(-M_PI/2)));
	T34=(Rot4(2,cos(joints[3]),sin(joints[3]))
			*D4(KUKA_OFFSET_D4)
			*Rot4(0,cos(M_PI/2),sin(M_PI/2)));
	T45=(Rot4(2,cos(joints[4]),sin(joints[4]))
			*D4(KUKA_OFFSET_D5)
			*Rot4(0,cos(-M_PI/2),sin(-M_PI/2)));
	T56=(Rot4(2,cos(joints[5]),sin(joints[5]))
			*D4(KUKA_OFFSET_D6)
			*Rot4(0,cos(M_PI/2),sin(M_PI/2)));
	T67=(Rot4(2,cos(joints[6]),sin(joints[6]))
			*D4(KUKA_OFFSET_D7));
	T07=T01*T12*T23*T34*T45*T56*T67; 

	// printf("one new kine loop:-----");
	// T01.print();T12.print();T23.print();T34.print();T45.print();T56.print();T67.print();
	// T07.print();

	for (int i = 1; i <= 3; i++)
	{
		for (int j = 1; j <= 3; j++)
		{
			// printf("T07(%d,%d)=%12.4f\n",i,j,T07(i,j));
			RotM(i,j)=T07(i,j);
		}
	}
	handori=RotMat2EulerAngle(RotM);
	for (int i = 1; i <= 3; i++)
	{
		// printf("T07(%d,4)=%12.4f\n",i,T07(i,4));
		handpos(i,1)=T07(i,4);
	}
	// handpos.print();handori.print();
    return ((handpos || handori));
}
MATRIX_D KUKA_Kinematics2(double* joints)
{  
	MATRIX_D jnt1_p=MatD31(0,0,0),
    jnt2_p=MatD31(0,0,0),  jnt3_p=MatD31(0,0,0), jnt4_p=MatD31(0,0,0),
    jnt5_p=MatD31(0,0,0),  jnt6_p=MatD31(0,0,0), hnd_p=MatD31(0,0,0);
    
	// MATRIX_D AxisAngle=MatD31(0.0,0.0,0.0);
	// MATRIX_D EulerAngle=MatD31(0.0,0.0,0.0);
	MATRIX_D Quaternion=MatD41(0.0,0.0,0.0,0.0);

	
	MATRIX_D rx_1=cameraRotMat;
	jnt1_p=cameraXYZ + rx_1 * MatD31(0,0,KUKA_OFFSET_D1);
	jnt2_p=jnt1_p;

	//rx_s is rx_2
	MATRIX_D rx_s  = rx_1 * (Rot(2, cos(joints[0]), sin(joints[0]))
						* Rot(0, cos(-M_PI/2), sin(-M_PI/2))
						* Rot(2, cos(joints[1]), sin(joints[1])));
		jnt3_p = jnt2_p + rx_s * Rot(0,cos(M_PI/2),sin(M_PI/2)) * MatD31(0,0,KUKA_OFFSET_D3);

	MATRIX_D rx_3 = rx_s * Rot(0,cos(M_PI/2),sin(M_PI/2)) * Rot(2, cos(joints[2]), sin(joints[2]));
		jnt4_p = jnt3_p + rx_3 * MatD31(0,0,KUKA_OFFSET_D4);

	MATRIX_D rx_4 = rx_3 * Rot(0,cos(M_PI/2),sin(M_PI/2)) *  Rot(2, cos(joints[3]), sin(joints[3]));
		jnt5_p = jnt4_p + rx_4 * Rot(0,cos(-M_PI/2),sin(-M_PI/2)) * MatD31(0,0,KUKA_OFFSET_D5);

	MATRIX_D rx_5 = rx_4 * Rot(0,cos(-M_PI/2),sin(-M_PI/2)) * Rot(2, cos(joints[4]), sin(joints[4]));
		jnt6_p = jnt5_p + rx_5 * MatD31(0,0,KUKA_OFFSET_D6);

	MATRIX_D rx_6 = rx_5 * Rot(0,cos(-M_PI/2),sin(-M_PI/2)) * Rot(2,cos(joints[5]), sin(joints[5]));
		hnd_p = jnt6_p + rx_6 * Rot(0,cos(M_PI/2),sin(M_PI/2)) * MatD31(0,0,KUKA_OFFSET_D7);

	MATRIX_D rx_h = rx_6 * Rot(0,cos(M_PI/2),sin(M_PI/2)) * Rot(2, cos(joints[6]), sin(joints[6]));

	// printf("one new kine loop:-----");
	// rx_s.print();rx_3.print();rx_4.print();rx_5.print();rx_6.print();rx_h.print();
	// jnt1_p.print();jnt2_p.print();jnt3_p.print();jnt4_p.print();jnt5_p.print();jnt6_p.print();hnd_p.print();

    Quaternion=RotMat2Quaternion(rx_h);
	// printf("rotmatix to queternion\n");
	// Quaternion.print();
	// printf("quaternion to rotmatrix\n");
	// Quaternion2RotMat(0.331448,0.331448,0.624613,0.624613).print();
	// printf("rotmatrix to quaternion\n");
	// RotMat2Quaternion(Quaternion2RotMat(0.331448,0.331448,0.624613,0.624613)).print();

    return ((hnd_p || Quaternion));
}

int KUKA_Jacobian(double* joints, MATRIX_D& jcb, MATRIX_D& rotM)
{
	// int flag;
	MATRIX_D rx_1=cameraRotMat;
    MATRIX_D jnt1_p=cameraXYZ + rx_1 * MatD31(0,0,KUKA_OFFSET_D1);
	MATRIX_D jnt1_axis=MatD31(rx_1(1,3), rx_1(2,3), rx_1(3,3));

	MATRIX_D rx_s  = rx_1 * (Rot(2, cos(joints[0]), sin(joints[0]))
						* Rot(0, cos(-M_PI/2), sin(-M_PI/2))
						* Rot(2, cos(joints[1]), sin(joints[1])));
	MATRIX_D jnt2_p = jnt1_p;
	MATRIX_D jnt2_axis= MatD31(rx_s(1,3),rx_s(2,3),rx_s(3,3));
	MATRIX_D jnt3_p = jnt2_p + rx_s * Rot(0,cos(M_PI/2),sin(M_PI/2)) * MatD31(0,0,KUKA_OFFSET_D3);
	MATRIX_D rx_3 = rx_s * Rot(0,cos(M_PI/2),sin(M_PI/2)) * Rot(2, cos(joints[2]), sin(joints[2]));
	MATRIX_D jnt3_axis = MatD31(rx_3(1,3),rx_3(2,3),rx_3(3,3));
	MATRIX_D jnt4_p = jnt3_p + rx_3 * MatD31(0,0,KUKA_OFFSET_D4);
	MATRIX_D rx_4 = rx_3 * Rot(0,cos(M_PI/2),sin(M_PI/2)) *  Rot(2, cos(joints[3]), sin(joints[3]));
	MATRIX_D jnt4_axis=MatD31(rx_4(1,3),rx_4(2,3),rx_4(3,3));
    MATRIX_D jnt5_p = jnt4_p + rx_4 * Rot(0,cos(-M_PI/2),sin(-M_PI/2)) * MatD31(0,0,KUKA_OFFSET_D5);
    MATRIX_D rx_5 = rx_4 * Rot(0,cos(-M_PI/2),sin(-M_PI/2)) * Rot(2, cos(joints[4]), sin(joints[4]));
	MATRIX_D jnt5_axis = MatD31(rx_5(1,3),rx_5(2,3),rx_5(3,3));
    MATRIX_D jnt6_p = jnt5_p + rx_5 * MatD31(0,0,KUKA_OFFSET_D6);
    MATRIX_D rx_6 = rx_5 * Rot(0,cos(-M_PI/2),sin(-M_PI/2)) * Rot(2,cos(joints[5]), sin(joints[5]));
	MATRIX_D jnt6_axis = MatD31(rx_6(1,3),rx_6(2,3),rx_6(3,3));	
	MATRIX_D pos = jnt6_p + rx_6 * Rot(0,cos(M_PI/2),sin(M_PI/2)) * MatD31(0,0,KUKA_OFFSET_D7);
	rotM = rx_6 * Rot(0,cos(M_PI/2),sin(M_PI/2)) * Rot(2, cos(joints[6]), sin(joints[6]));
	MATRIX_D jnt7_p = pos;
	MATRIX_D jnt7_axis = MatD31(rotM(1,3),rotM(2,3),rotM(3,3));

	// double Jcb[6][7];
	// (double *)Jcb<<=
	jcb=
	(((jnt1_axis && (pos-jnt1_p)) | (jnt2_axis&&(pos-jnt2_p)) |
       (jnt3_axis && (pos-jnt3_p)) | (jnt4_axis&&(pos-jnt4_p)) |
	   (jnt5_axis && (pos-jnt5_p)) | (jnt6_axis&&(pos-jnt6_p)) | (jnt7_axis&&(pos-jnt7_p))) ||
	   (jnt1_axis | jnt2_axis | jnt3_axis |
	    jnt4_axis | jnt5_axis | jnt6_axis | jnt7_axis));

	// MATRIX_D jcb(6,6,(double *)jcbn->Jcb);
	// MATRIX_D ijcb(6,6),tjcb(6,6);
	// tjcb=jcb.t();
	// // (double*)jcbn->TrnsJcb<<=tjcb;
	// flag=CalcInverse(jcb,ijcb);
	// // if(flag==0)
	// 	// (double*)jcbn->invJcb<<=ijcb;
	// printf("jacobian is:\n"); jcb.print();//for test/

	// MATRIX_D* tmp={rotM,jcb};
	return 1;
}
MATRIX_D KUKA_InKinematics(double* RefPos, double* currentPos, double* currentJoints, double* holdposJoints,bool posCmdFinishFlag)
{
	/*error tolerance control*/
	bool moveFlag=false;
	for (int i = 0; i < 3; i++)
	{
		if(fabs(RefPos[i]-currentPos[i])>0.001) moveFlag=true;//xyz accuracy 0.001m
	}
	for (int i = 3; i < 7; i++)
	{
		if(fabs(RefPos[i]-currentPos[i])>0.01) moveFlag=true; //quaternion accuracy 0.01 
	}
	if (!moveFlag&&posCmdFinishFlag)
	{
		MATRIX_D reftheta(7,1,holdposJoints);
		printf("\nholdposJoints once\n");
		return reftheta;
	}

	// MATRIX_D ref_pos(6,1,RefPos);
	// MATRIX_D pos(6,1,currentPos);
	// MATRIX_D B0=Zeros(3,3);
    // B0=CalcB0(pos(4,1),pos(5,1),pos(6,1));
  	// MATRIX_D PosErrorTrn=Zeros(6,6);
	// PosErrorTrn=((Eye(3)|Zeros(3,3))||(Zeros(3,3)|B0));

	MATRIX_D jcb=Zeros(6,7); //jacobian matrix is 6*7
	MATRIX_D curRotM=Zeros(3,3); //Rot matrix　is 3*3

	KUKA_Jacobian(currentJoints,jcb,curRotM);
	// printf("\ncurrent jcb is");	jcb.print(); 
	// printf("current rotmatrix is");	curRotM.print();

	MATRIX_D nc = MatD31(curRotM(1,1),curRotM(2,1),curRotM(3,1));
	MATRIX_D oc = MatD31(curRotM(1,2),curRotM(2,2),curRotM(3,2));
	MATRIX_D ac = MatD31(curRotM(1,3),curRotM(2,3),curRotM(3,3));

	MATRIX_D desRotM=Quaternion2RotMat(RefPos[3],RefPos[4],RefPos[5],RefPos[6]);
	// printf("destiny rotmatrix is"); desRotM.print();
	MATRIX_D nd = MatD31(desRotM(1,1),desRotM(2,1),desRotM(3,1));
	MATRIX_D od = MatD31(desRotM(1,2),desRotM(2,2),desRotM(3,2));
	MATRIX_D ad = MatD31(desRotM(1,3),desRotM(2,3),desRotM(3,3));
	// printf("nc oc ac nd od ad"); nc.print();oc.print();ac.print();nd.print();od.print();ad.print();
	// printf("result of x is"); (nc&&nd).print();(oc&&od).print();(ac&&ad).print();

	MATRIX_D deltaw=0.5*((nc&&nd)+(oc&&od)+(ac&&ad));//*****!!!MATRIX_D类型的运算：叉乘优先级低于加法!!!****//
	MATRIX_D deltap=MatD31(RefPos[0],RefPos[1],RefPos[2])-MatD31(currentPos[0],currentPos[1],currentPos[2]);

	MatrixXf jacobiE(6,7);
	for(int i=0;i<6;i++){
		for(int j=0;j<7;j++){
			jacobiE(i,j)=jcb(i+1,j+1);
		}
    }

	/* calculate the pseudo_inverse Jacobain  */	
		printf("jacobian:\n");jcb.print();//for test
		JacobiSVD<MatrixXf> svd(jacobiE, ComputeThinU | ComputeThinV );
		// printf("svd\n");//for test
		float  pinvtoler = 1.e-6; // choose your tolerance wisely
		// float  pinvtoler = 1.e-2; // choose your tolerance wisely
		MatrixXf singularValues_inv = svd.singularValues();
		printf("singularValues: ");//for test
		for ( long i=0; i<jacobiE.rows(); ++i) {
			printf(" %lf,",singularValues_inv(i));//for test
			if ( singularValues_inv(i) > pinvtoler )
				singularValues_inv(i)=1.0/singularValues_inv(i);
			else singularValues_inv(i)=0;

		}
		printf("\nsingularValues_inv: ");//for test
		for ( long i=0; i<jacobiE.rows(); ++i)
			printf(" %lf,",singularValues_inv(i));//for test
		printf("\n");
		// printf("sigularValues_inv_limit\n");//for test
		MatrixXf pinvmat =  svd.matrixV()*singularValues_inv.asDiagonal()*svd.matrixU().transpose();
		// printf("pinvmat\n");//for test
	
	MATRIX_D invjcb=Zeros(7,6); //jacobian matrix's inverse is 7*6
	for(int i=0;i<7;i++){
		for(int j=0;j<6;j++){
			invjcb(i+1,j+1)=pinvmat(i,j);
		}
	}
	printf("invjcb:\n");invjcb.print();//for test
	MATRIX_D err_pos=MatD61(0,0,0,0,0,0);
	err_pos = deltap||deltaw;
	// printf("err_pos is"); err_pos.print();
	MATRIX_D DJnt(7,1);
    DJnt=invjcb*err_pos;//算出角度差
	// printf("Delta joints are :"); DJnt.print();//for test

	for(int i=1;i<=7;i++){//限位，每8ms关节运动量不超过1.4度
		if((DJnt(i,1)*180/M_PI)>1.4)
			DJnt(i,1)=0.024434609;
		else if((DJnt(i,1)*180/M_PI)<-1.4)
			DJnt(i,1)=-0.024434609;
	}
	// printf("Delta joints after limited are :"); DJnt.print();// for test
	MATRIX_D reftheta(7,1);
	for (int i = 0; i < 7; i++)
	{
		reftheta(i+1,1)=currentJoints[i]+DJnt(i+1,1);
		//joint limit
		if(reftheta(i+1,1)>jointLimit[i]*M_PI/180) reftheta(i+1,1)=jointLimit[i]*M_PI/180;
		if(reftheta(i+1,1)<-jointLimit[i]*M_PI/180) reftheta(i+1,1)=-jointLimit[i]*M_PI/180;
	}
	// printf("ref theta is :"); reftheta.print();//for test
	// MATRIX_D testM=Zeros(3,3);
	// MATRIX_D testJ=Zeros(6,7);
	// KUKA_Jacobian(reftheta.matrix2array(),testJ,testM);
	// printf("rotmatrix change to"); testM.print();
	
	/* beforehand Reachable Boundary Limit check */
		KUKA_Jacobian(reftheta.matrix2array(),jcb,curRotM);
		for(int i=0;i<6;i++){
			for(int j=0;j<7;j++){
				jacobiE(i,j)=jcb(i+1,j+1);
			}
		}
		JacobiSVD<MatrixXf> svd_f(jacobiE, ComputeThinU | ComputeThinV );
		pinvtoler = 1.e-2; // choose your tolerance wisely
		singularValues_inv = svd_f.singularValues();
		printf("next T singularValues: ");//for test
		for ( long i=0; i<jacobiE.rows(); ++i) {
			printf(" %lf,",singularValues_inv(i));//for test
			if ( singularValues_inv(i) > pinvtoler ){
				singularValues_inv(i)=1.0/singularValues_inv(i);
			}else{
				printf("\n*********Reached the reachable boundary Limit**********\n");
				MATRIX_D HoldPJoints(7,1,holdposJoints);
				return HoldPJoints;
			}
		}

	return reftheta;
}


void Camera2Link0Trans(const char* robotname)
{
    cameraXYZ.print();
	// printf("%s===%s\n",robotname,camera_base_link0);
	// printf("%d",strcmp(robotname,camera_base_link0));

	// double tempy=sqrt(cameraXYZ(2,1)*cameraXYZ(2,1)+cameraXYZ(3,1)*cameraXYZ(3,1))*sin(25*M_PI/180+atan(cameraXYZ(3,1)/cameraXYZ(2,1)));
	// double tempx=sqrt(cameraXYZ(2,1)*cameraXYZ(2,1)+cameraXYZ(3,1)*cameraXYZ(3,1))*cos(25*M_PI/180+atan(cameraXYZ(3,1)/cameraXYZ(2,1)));
	// printf("x===%lf\ny===%lf\n",tempx,tempy);

	//the transform param from camera to camera_base_link0
	if (strcmp(robotname,camera_base_link0)==0) //when robotname equal to camera_base_link0
	{
		//camera to link0 rotation matrix
		cameraRotMat=Quaternion2RotMat(cameraQuaternion(1,1),cameraQuaternion(2,1),cameraQuaternion(3,1),cameraQuaternion(4,1));
		//camera to link0 xyz vector
		cameraXYZ=-cameraRotMat*cameraXYZ;
		// printf("%s cameraXYZ R:",robotname);
		// cameraRotMat.print();
		// RotMat2EulerAngle(cameraRotMat).print();
		// cameraXYZ.print();
	}
	else //the transform param from camera to another link0
	{
		double right2left=-50*M_PI/180, left2right=-right2left;
		int flag = (strcmp(camera_base_link0,"right")==0)?1:0;
		// printf("%d",flag);
		//camera to camera_base_link0 rotation matrix
		cameraRotMat=Quaternion2RotMat(cameraQuaternion(1,1),cameraQuaternion(2,1),cameraQuaternion(3,1),cameraQuaternion(4,1));
		//camera to another_link0 xyz vector
		cameraXYZ=-cameraRotMat*cameraXYZ+cameraRotMat*MatD31( 0, 0.317208*cos(25*M_PI/180)*(flag?1:-1), -0.317208*sin(25*M_PI/180) );
		//camera to another_link0 rotation matrix
		cameraRotMat=cameraRotMat * (flag?Rot(0,cos(right2left),sin(right2left)):Rot(0,cos(left2right),sin(left2right)));

		// printf("%s cameraXYZ L:",robotname);
		// cameraRotMat.print();
		// RotMat2EulerAngle(cameraRotMat).print();
		// cameraXYZ.print();
		// MatD31( 0, 0.317208*cos(25*M_PI/180)*(flag?1:-1), -0.317208*sin(25*M_PI/180) ).print();
	}

}


extern int CalcInverse(MATRIX_D &jcb, MATRIX_D &ijcb)
{
  MATRIX_D mat(6,6);
  mat=jcb;
  if(mat.snglr())
	 return (-1);
  else
  {
	  ijcb=mat.inverse();
  }
	return (0);
}

MATRIX_D AxisAnglePi(MATRIX_D BMat)
{
 double b11,b22,b33,b12,b13,b23;
 b11=BMat(1,1);b22=BMat(2,2);b33=BMat(3,3);
 b12=BMat(1,2);b13=BMat(1,3);b23=BMat(2,3);

 if((b12>0)&(b13>0)&(b23>0))
 {
   return(MatD31(sqrt(b11),sqrt(b22),sqrt(b33)));
 }else if(abs(b12*b13*b23)>0.0001)
   { // no one is zero
       if(b12>0)
       {
    	return(MatD31(sqrt(b11),sqrt(b22),-sqrt(b33)));
       }
       else if(b13>0)
       {
    	return(MatD31(sqrt(b11),-sqrt(b22),sqrt(b33)));
       }
       else
       {
    	return(MatD31(-sqrt(b11),sqrt(b22),sqrt(b33)));
       }
   }else if((abs(b12)>0.0001)||(abs(b13)>0.0001)||(abs(b23)>0.0001))
        {// one ni is zero
	           if(abs(b12)>0.0001)
	              { if(b12>0)
	    	        return(MatD31(sqrt(b11),sqrt(b22),0.0));
	                else
	                return(MatD31(sqrt(b11),-sqrt(b22),0.0));
	              }
	          if(abs(b13)>0.0001)
	     	       { if(b13>0)
	     	    	 return(MatD31(sqrt(b11),0.0,sqrt(b33)));
	     	         else
	     	         return(MatD31(sqrt(b11),0.0,-sqrt(b33)));
	     	       }
	          if(abs(b23)>0.0001)
	     	       { if(b23>0)
	     	    	 return(MatD31(0.0,sqrt(b22),sqrt(b33)));
	     	         else
	     	         return(MatD31(0.0,sqrt(b22),-sqrt(b33)));
	     	       }
   }else
   {   // two n_i are zero
	   if(abs(b11)>0.0001)
		   {return (MatD31(sqrt(b11),0.0,0.0));}
	   else if(abs(b22)>0.0001)
	      {return (MatD31(0.0,sqrt(b22),0.0));}
	   else
	      {return (MatD31(0.0,0.0,sqrt(b33)));}
   }
}

MATRIX_D RotMat2AxisAngle(MATRIX_D rotMat)
{
  double angle, cos_angle;

  MATRIX_D temp=Zeros(3,3);
  MATRIX_D vN = MatD31(0,0,0);

  angle=acos(1/2.0*(rotMat.tr()-1));
  cos_angle = cos(angle);

  if(cos_angle<-0.999)
  {// condition that angle = pi
   MATRIX_D BMat = 0.5*(rotMat+Eye(3));

   return (AxisAnglePi(BMat));
          }
  else if (cos_angle > 0.999 )
  { // condition that angle = 0, axis is undetermined.
	         return (Zeros(3,1));
  }
  else {
	temp=1/(2*sin(angle))*(rotMat-rotMat.t());
	vN=MatD31(temp(3,2),temp(1,3),temp(2,1));
	return(angle * vN);
  }
}





