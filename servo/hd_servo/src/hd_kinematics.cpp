#include"hd_kinematics.h"

MATRIX_D Forward_kinematics(arrayD_three joint_theta,int flag){

    float link_3;
    float DigitSurf_link3;

    MATRIX_D T(4,4);//总的齐次变换矩阵（从总的手坐标系到DIGIT传感器坐标系）；
    MATRIX_D T0=Eye(4);

    if(flag==0)/*左*/{
        link_3=link_3_L;
        DigitSurf_link3=DigitSurf_link3_L;
        T0=Sysmm(1)*Trans(0,axis0_center);
    }
    else/*右*/{
        link_3=link_3_R;
        DigitSurf_link3=DigitSurf_link3_R;
        T0=Trans(0,axis0_center);
    }

    MATRIX_D T1=T_3R(link_1,0,joint_theta[0]);
    MATRIX_D T2=T_3R(link_2,0,joint_theta[1]);
    MATRIX_D T3=T_3R(link_3,0,joint_theta[2]);

    /*need test*/
    MATRIX_D T4=Eye(4);   //Digit传感器表面相对于link3的齐次变换矩阵
    T4(2,4)=abs(DigitSurf_link3);
    
    T=T0*T1*T2*T3*T4;
    MATRIX_D  End_pos_=MatD31(0,0,0);
    End_pos_(1,1)=T(1,4);
    End_pos_(2,1)=T(2,4);
    End_pos_(3,1)=acos(T(2,2));
    End_pos_(3,1)=End_pos_(3,1)*RadtoDeg;//得到每个手指坐标系下的fai角

    End_pos_(3,1)=End_pos_(3,1)-90.0;//把每个手指坐标系下的fai角转换成总的基座标系下，转换成竖直状态下为0度

    return End_pos_;//返回列向量

}

MATRIX_D Inver_kinematics(arrayD_three end_pos_,int flag){
    /*将目标末端坐标转换为每根手指的坐标系下*/
    double Xe,Ye,FAIe;
    end_pos_[2]=end_pos_[2]+90.0;//命令度数基于竖直状态下为0度，需转换成fai角
    FAIe=end_pos_[2]*DegtoRad;
    float DigitSurf_link3;
    float link_3;

    /*手坐标系向手指坐标系的转换*/
    if(!flag)/*左*/{
        end_pos_[0]=-(axis0_center+end_pos_[0]);
        DigitSurf_link3=DigitSurf_link3_L;
        link_3=link_3_L;
    }
    else/*右*/{
        end_pos_[0]=end_pos_[0]-axis0_center;
        DigitSurf_link3=DigitSurf_link3_R;
        link_3=link_3_R;
    }

    /*Digit指尖坐标向link3末端的转换*/
    Xe=end_pos_[0]-DigitSurf_link3*sin(FAIe);
    Ye=end_pos_[1]+DigitSurf_link3*cos(FAIe);

    double Xc=Xe-link_3*cos(FAIe);
    double Yc=Ye-link_3*sin(FAIe);
    double r2=Xc*Xc+Yc*Yc;
    double D=(link_1*link_1+link_2*link_2-r2)/(2*link_1*link_2);

    MATRIX_D  Angle_joint_=MatD31(0,0,0);
    Angle_joint_(2,1)=M_PI-atan2(sqrt(1-D*D),D);//θ2

    double alpha=atan2(Yc,Xc);
    double gamma=atan2(link_2*sin(Angle_joint_(2,1)),link_1+link_2*cos(Angle_joint_(2,1)));

    Angle_joint_(1,1)=alpha-gamma;//θ1
    Angle_joint_(3,1)=FAIe-Angle_joint_(1,1)-Angle_joint_(2,1);//θ3

    for(int i=1;i<=3;++i){
        Angle_joint_(i,1)=Angle_joint_(i,1)*RadtoDeg;
    }

    return Angle_joint_;//返回列向量  

}