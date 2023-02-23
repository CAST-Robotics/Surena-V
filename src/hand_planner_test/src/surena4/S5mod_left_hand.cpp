#include "S5mod_left_hand.h"
#include <iostream>


left_hand::left_hand(){


}

left_hand::left_hand(VectorXd q_le,VectorXd r_target,MatrixXd R_target)
{

    HO_FK_left_palm(q_le);

    dist=distance(r_target,r_left_palm);
    sai_target=sai_calc(R_target);
    theta_target=theta_calc(R_target);
    phi_target=phi_calc(R_target);
    Eul_Angles_target <<phi_target,theta_target,sai_target;
    dist_or=distance(Eul_Angles_target,Eul_Angles);


}


left_hand::left_hand(VectorXd q_le,VectorXd v,VectorXd r_target,MatrixXd R_target)
{

    HO_FK_left_palm(q_le);
    dist=distance(r_target,r_left_palm);
    dist_or=distance(R_target,R_left_palm);
    sai_target=sai_calc(R_target);
    theta_target=theta_calc(R_target);
    phi_target=phi_calc(R_target);
    V.resize(3,1); //??
    V=v; //???
    sai_dot=(sai_target-sai); //?? coef kuuuuuuu??
    phi_dot=(phi_target-phi);
    theta_dot=(theta_target-theta);

    euler2w();
    jacob(q_le);
}


left_hand::left_hand(VectorXd q_le,VectorXd r_target,MatrixXd R_target,int i,double d0)
{

HO_FK_left_palm(q_le);

dist=distance(r_target,r_left_palm);
sai_target=sai_calc(R_target);
theta_target=theta_calc(R_target);
phi_target=phi_calc(R_target);

V.resize(3,1);
double v_coef=v_des*min(float(i)/100.0,1.0)*pow(atan(dist/d0*20.0)/M_PI*2,2)/dist;
V=v_coef*(r_target-r_left_palm);

double oriet_coef=1;//pow(tanh(3*(d_orient-dist)/d_orient),2);
sai_dot=oriet_coef*(sai_target-sai);
phi_dot=oriet_coef*(phi_target-phi);
theta_dot=oriet_coef*(theta_target-theta);

euler2w();
jacob(q_le);
}


left_hand::left_hand(VectorXd q_le,VectorXd r_target,MatrixXd R_target,double d0,double v_0, double v__target)
{
HO_FK_left_palm(q_le);

dist=distance(r_target,r_left_palm);
sai_target=sai_calc(R_target);
theta_target=theta_calc(R_target);
phi_target=phi_calc(R_target);

V.resize(3,1);
//double v_coef=v_des*min(float(i)/100.0,1.0)*pow(atan(dist/d0*20.0)/M_PI*2,2)/dist;
v0=v_0;
v_target=v__target;
double v_coef=velocity(dist,d0);
V=v_coef*(r_target-r_left_palm)/dist;

double oriet_coef=1;//pow(tanh(3*(d_orient-dist)/d_orient),2);
sai_dot=oriet_coef*(sai_target-sai);
phi_dot=oriet_coef*(phi_target-phi);
theta_dot=oriet_coef*(theta_target-theta);
euler2w();
jacob(q_le);
}




void left_hand::update_left_hand(VectorXd q_le,VectorXd r_target,MatrixXd R_target)
{

  
HO_FK_left_palm(q_le);

dist=distance(r_target,r_left_palm);
sai_target=sai_calc(R_target);
theta_target=theta_calc(R_target);
phi_target=phi_calc(R_target);
Eul_Angles_target <<phi_target,theta_target,sai_target;
dist_or=distance(Eul_Angles_target,Eul_Angles);


double oriet_coef=1;//pow(tanh(3*(d_orient-dist)/d_orient),2);
sai_dot=oriet_coef*(sai_target-sai);
phi_dot=oriet_coef*(phi_target-phi);
theta_dot=oriet_coef*(theta_target-theta);

euler2w();
jacob(q_le);

}


void left_hand::update_left_hand(VectorXd q_le,VectorXd v,VectorXd r_target,MatrixXd R_target)
{

    HO_FK_left_palm(q_le);
    dist=distance(r_target,r_left_palm);
    sai_target=sai_calc(R_target);
    theta_target=theta_calc(R_target);
    phi_target=phi_calc(R_target);
    Eul_Angles_target <<phi_target,theta_target,sai_target;
    dist_or=distance(Eul_Angles_target,Eul_Angles);

    V.resize(3,1);
    V=v;
    double oriet_coef=1;//pow(tanh(3*(d_orient-dist)/d_orient),2);
 sai_dot=oriet_coef*(sai_target-sai);
 phi_dot=oriet_coef*(phi_target-phi);
 theta_dot=oriet_coef*(theta_target-theta);

    euler2w();
    jacob(q_le);
}


void left_hand::update_left_hand(VectorXd q_le,VectorXd r_target,MatrixXd R_target,int i,double d0)
{

HO_FK_left_palm(q_le);

dist=distance(r_target,r_left_palm);
sai_target=sai_calc(R_target);
theta_target=theta_calc(R_target);
phi_target=phi_calc(R_target);
Eul_Angles_target <<phi_target,theta_target,sai_target;
dist_or=distance(Eul_Angles_target,Eul_Angles);


V.resize(3,1);
double v_coef=v_des*min(float(i)/100.0,1.0)*pow(atan(dist/d0*20.0)/M_PI*2,2)/dist;
V=v_coef*(r_target-r_left_palm);

double oriet_coef=1;//pow(tanh(3*(d_orient-dist)/d_orient),2);
sai_dot=oriet_coef*(sai_target-sai);
phi_dot=oriet_coef*(phi_target-phi);
theta_dot=oriet_coef*(theta_target-theta);

euler2w();
jacob(q_le);
}


void left_hand::update_left_hand(VectorXd q_le,VectorXd r_target,MatrixXd R_target,double d0,double v_0, double v__target)
{
HO_FK_left_palm(q_le);

dist=distance(r_target,r_left_palm);
sai_target=sai_calc(R_target);
theta_target=theta_calc(R_target);
phi_target=phi_calc(R_target);

V.resize(3,1);
//double v_coef=v_des*min(float(i)/100.0,1.0)*pow(atan(dist/d0*20.0)/M_PI*2,2)/dist;
v0=v_0;
v_target=v__target;
double v_coef=velocity(dist,d0);
V=v_coef*(r_target-r_left_palm)/dist;

double oriet_coef=1;//pow(tanh(3*(d_orient-dist)/d_orient),2);
sai_dot=oriet_coef*(sai_target-sai);
phi_dot=oriet_coef*(phi_target-phi);
theta_dot=oriet_coef*(theta_target-theta);
euler2w();
jacob(q_le);
}


/*void right_hand::matrix_view(MatrixXd M){ //??????????
    for (int i = 0; i <M.rows() ; ++i) {
        QString str;
        for (int j = 0; j <M.cols() ; ++j) {str+=QString::number(M(i,j));str+="   ";}
        cout<<str;}cout<<"";}

void right_hand::matrix_view(VectorXd M){
    QString str;
    for (int i = 0; i <M.rows() ; ++i) {str+=QString::number(M(i));str+="   ";}
    cout<<str;cout<<"";} */   //UNCOMMENT LATER**************************************




double left_hand::toRad(double d){
    double r;
    r=d*M_PI/180;
    return r;
}
/*
double  left_hand::phi_calc(MatrixXd R){
    if(R(1,0)<1 && R(1,0)>-1){return atan2(-R(2,0),R(0,0));}
    else if (R(1,0)==-1){return -atan2(R(2,1),R(2,2));}
    else if(R(1,0)==-1){return atan2(R(2,1),R(2,2));}
}
double  left_hand::theta_calc(MatrixXd R){
    if(R(1,0)<1 && R(1,0)>-1){return asin(R(1,0));}
    else if (R(1,0)==-1){return -M_PI/2;}
    else if(R(1,0)==-1){return M_PI/2;};
}
double  left_hand::sai_calc(MatrixXd R){

    if(R(1,0)<1 && R(1,0)>-1){return atan2(-R(1,2),R(1,1));}
    else if (R(1,0)==-1){return 0;}
    else if(R(1,0)==-1){return 0;}
}
*/
double  left_hand::phi_calc(MatrixXd R){
    if(R(1,0)==1 || R(1,0)==-1){return atan2(R(0,2),R(2,2));}
    else{return atan2(-R(2,0),R(0,0));}
}
double  left_hand::theta_calc(MatrixXd R){
    return asin(R(1,0));
}
double  left_hand::sai_calc(MatrixXd R){

    if(R(1,0)==1 || R(1,0)==-1){return 0;}
    else{return atan2(-R(1,2),R(1,1));}
}



MatrixXd left_hand::rot(int axis , double q ,int dim){
    if (dim==3){
        MatrixXd R(3,3);
    if (axis==1){
        R<<1,0,0,
                0,cos(q),-sin(q),
                0,sin(q),cos(q);
    }

    if (axis==2){
        R<<cos(q),0,sin(q),
                0,1,0 ,
                -sin(q),0,cos(q);
    }

    if (axis==3){
                R<<cos(q),-sin(q),0,
                sin(q),cos(q),0,
                0,0,1;
    }
    return R;
    }

    if(dim==4){
                    MatrixXd R(4,4);
        if (axis==1){
            R<<1,0,0,0,
                    0,cos(q),-sin(q),0,
                    0,sin(q),cos(q),0,
                    0,0,0,1;
        }

        if (axis==2){
            R<<cos(q),0,sin(q),0,
                    0,1,0,0,
                    -sin(q),0,cos(q),0,
                    0,0,0,1;
        }

        if (axis==3){
            R<<cos(q),-sin(q),0,0,
                    sin(q),cos(q),0,0,
                    0,0,1,0,
                    0,0,0,1;
        }
        return R;
    }

}

MatrixXd left_hand::trans(int axis, double d){
    MatrixXd H(4,4);
    H=MatrixXd::Identity(4,4);
    H(axis-1,3)=d;
    return H;
}

MatrixXd left_hand::trans(Vector3d d){
    MatrixXd H(4,4);
    H=MatrixXd::Identity(4,4);
    H.block(0,3,3,1)=d;
    return H; 
}

double left_hand::distance(VectorXd V1,VectorXd V2){
    double d;
    d=sqrt(pow(V1(0)-V2(0),2)+pow(V1(1)-V2(1),2)+pow(V1(2)-V2(2),2));
    return d;
}

double left_hand::velocity(double d,double d0){ //????
    double a2,a3;
    a3=2*(v_target-v0)/d0/d0/d0;
    a2=-3/2*a3*d0;
    return v_target+a2*d*d+a3*d*d*d;
}

void left_hand::HO_FK_left_palm(VectorXd q_le){

    R1_fix_shd.resize(4,4);
    R2_fix_shd.resize(4,4);
    R1_le .resize(4,4);
    R2_le.resize(4,4);
    R3_le.resize(4,4);
    R4_le.resize(4,4);
    R5_le.resize(4,4);
    R6_le.resize(4,4);
    R7_le.resize(4,4);


R1_fix_shd=rot(1,angle_fix_shd,4);
R2_fix_shd=rot(1,-angle_fix_shd,4);
R1_le = rot(2,q_le(0),4);
R2_le = rot(1,q_le(1),4);
R3_le = rot(3,q_le(2),4);
R4_le = rot(2,q_le(3),4);
R5_le = rot(3,q_le(4),4);
R6_le = rot(1,q_le(5),4);
R7_le = rot(2,q_le(6),4);

P_arm_le.resize(4,4);
P_forearm_le.resize(4,4);
P_palm_le.resize(4,4);
P_arm_le = trans(3,-L_arm);
P_forearm_le = trans(3,-L_forearm);
P_palm_le = trans(3,-L_palm);

T_left_palm.resize(4,4);
T_left_palm=R1_fix_shd*R1_le*R2_fix_shd*R2_le*R3_le*P_arm_le*R4_le*R5_le*P_forearm_le*R6_le*R7_le*P_palm_le;
r_left_palm.resize(3,1);
r_left_palm<<T_left_palm.block(0,3,3,1);
R_left_palm.resize(3,3);
R_left_palm<<T_left_palm.block(0,0,3,3);
theta=theta_calc(R_left_palm);
sai=sai_calc(R_left_palm);
phi=phi_calc(R_left_palm);
//Eul_Angles << sai, theta, phi;
//if(sai>M_PI/2){
//    sai=sai-M_PI;
//    if(phi>M_PI/2 ){phi=phi-M_PI;}
//    if(phi<-M_PI/2 ){phi=phi+M_PI;}
//    theta=-theta;
//}

//if(sai<-M_PI/2){
//    sai=sai+M_PI;
//    if(phi>M_PI/2 ){phi=phi-M_PI;}
//    if(phi<-M_PI/2 ){phi=phi+M_PI;}
//    theta=-theta;
//}

}

void left_hand::euler2w(){
//w_right_palm<<sin(sai)*sin(theta)*phi_dot+cos(sai)*theta_dot,
//        cos(sai)*sin(theta)*phi_dot-sin(sai)*theta_dot,
//        cos(theta)*phi_dot+sai_dot;

//w_right_palm<<cos(phi)*theta_dot + sin(phi)*sin(theta)*sai_dot,
//        sin(phi)*theta_dot - cos(phi)*sin(theta)*sai_dot,
//        cos(theta)*sai_dot + phi_dot;
//w_right_palm=-w_right_palm;

    w_left_palm<< sin(phi)*theta_dot + cos(phi)*cos(theta)*sai_dot,
            sin(theta)*sai_dot + phi_dot,
            cos(phi)*theta_dot - cos(theta)*sin(phi)*sai_dot;

}

void left_hand::jacob(VectorXd q_le){

    J_left_palm.resize(3,7);
    J_w_left_palm.resize(3,7);

    J_left_palm<<                                                                                                                                                                                                                                                                                                                                                         L_forearm*(sin(q_le(3))*(cos(q_le(2))*sin(q_le(0)) - sin(q_le(2))*(cos(angle_fix_shd)*cos(q_le(0))*sin(q_le(1)) - cos(q_le(0))*cos(q_le(1))*sin(angle_fix_shd))) - cos(q_le(3))*(cos(angle_fix_shd)*cos(q_le(0))*cos(q_le(1)) + cos(q_le(0))*sin(angle_fix_shd)*sin(q_le(1)))) - L_palm*(cos(q_le(6))*(sin(q_le(5))*(sin(q_le(4))*(sin(q_le(0))*sin(q_le(2)) + cos(q_le(2))*(cos(angle_fix_shd)*cos(q_le(0))*sin(q_le(1)) - cos(q_le(0))*cos(q_le(1))*sin(angle_fix_shd))) - cos(q_le(4))*(cos(q_le(3))*(cos(q_le(2))*sin(q_le(0)) - sin(q_le(2))*(cos(angle_fix_shd)*cos(q_le(0))*sin(q_le(1)) - cos(q_le(0))*cos(q_le(1))*sin(angle_fix_shd))) + sin(q_le(3))*(cos(angle_fix_shd)*cos(q_le(0))*cos(q_le(1)) + cos(q_le(0))*sin(angle_fix_shd)*sin(q_le(1))))) - cos(q_le(5))*(sin(q_le(3))*(cos(q_le(2))*sin(q_le(0)) - sin(q_le(2))*(cos(angle_fix_shd)*cos(q_le(0))*sin(q_le(1)) - cos(q_le(0))*cos(q_le(1))*sin(angle_fix_shd))) - cos(q_le(3))*(cos(angle_fix_shd)*cos(q_le(0))*cos(q_le(1)) + cos(q_le(0))*sin(angle_fix_shd)*sin(q_le(1))))) - sin(q_le(6))*(cos(q_le(4))*(sin(q_le(0))*sin(q_le(2)) + cos(q_le(2))*(cos(angle_fix_shd)*cos(q_le(0))*sin(q_le(1)) - cos(q_le(0))*cos(q_le(1))*sin(angle_fix_shd))) + sin(q_le(4))*(cos(q_le(3))*(cos(q_le(2))*sin(q_le(0)) - sin(q_le(2))*(cos(angle_fix_shd)*cos(q_le(0))*sin(q_le(1)) - cos(q_le(0))*cos(q_le(1))*sin(angle_fix_shd))) + sin(q_le(3))*(cos(angle_fix_shd)*cos(q_le(0))*cos(q_le(1)) + cos(q_le(0))*sin(angle_fix_shd)*sin(q_le(1)))))) - L_arm*(cos(angle_fix_shd)*cos(q_le(0))*cos(q_le(1)) + cos(q_le(0))*sin(angle_fix_shd)*sin(q_le(1))),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         L_forearm*(cos(q_le(3))*(cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)) - cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0))) - sin(q_le(2))*sin(q_le(3))*(cos(angle_fix_shd)*cos(q_le(1))*sin(q_le(0)) + sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)))) - L_palm*(cos(q_le(6))*(sin(q_le(5))*(cos(q_le(4))*(sin(q_le(3))*(cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)) - cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0))) + cos(q_le(3))*sin(q_le(2))*(cos(angle_fix_shd)*cos(q_le(1))*sin(q_le(0)) + sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)))) + cos(q_le(2))*sin(q_le(4))*(cos(angle_fix_shd)*cos(q_le(1))*sin(q_le(0)) + sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)))) - cos(q_le(5))*(cos(q_le(3))*(cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)) - cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0))) - sin(q_le(2))*sin(q_le(3))*(cos(angle_fix_shd)*cos(q_le(1))*sin(q_le(0)) + sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(1))))) + sin(q_le(6))*(sin(q_le(4))*(sin(q_le(3))*(cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)) - cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0))) + cos(q_le(3))*sin(q_le(2))*(cos(angle_fix_shd)*cos(q_le(1))*sin(q_le(0)) + sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)))) - cos(q_le(2))*cos(q_le(4))*(cos(angle_fix_shd)*cos(q_le(1))*sin(q_le(0)) + sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(1))))) + L_arm*(cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)) - cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0))),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     L_palm*(cos(q_le(6))*(sin(q_le(5))*(sin(q_le(4))*(cos(q_le(0))*cos(q_le(2)) + sin(q_le(2))*(cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)) - cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0)))) + cos(q_le(3))*cos(q_le(4))*(cos(q_le(0))*sin(q_le(2)) - cos(q_le(2))*(cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)) - cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0))))) + cos(q_le(5))*sin(q_le(3))*(cos(q_le(0))*sin(q_le(2)) - cos(q_le(2))*(cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)) - cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0))))) - sin(q_le(6))*(cos(q_le(4))*(cos(q_le(0))*cos(q_le(2)) + sin(q_le(2))*(cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)) - cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0)))) - cos(q_le(3))*sin(q_le(4))*(cos(q_le(0))*sin(q_le(2)) - cos(q_le(2))*(cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)) - cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0)))))) + L_forearm*sin(q_le(3))*(cos(q_le(0))*sin(q_le(2)) - cos(q_le(2))*(cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)) - cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0)))),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             - L_forearm*(cos(q_le(3))*(cos(q_le(0))*cos(q_le(2)) + sin(q_le(2))*(cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)) - cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0)))) - sin(q_le(3))*(cos(angle_fix_shd)*cos(q_le(1))*sin(q_le(0)) + sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)))) - L_palm*(cos(q_le(6))*(cos(q_le(5))*(cos(q_le(3))*(cos(q_le(0))*cos(q_le(2)) + sin(q_le(2))*(cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)) - cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0)))) - sin(q_le(3))*(cos(angle_fix_shd)*cos(q_le(1))*sin(q_le(0)) + sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)))) - cos(q_le(4))*sin(q_le(5))*(sin(q_le(3))*(cos(q_le(0))*cos(q_le(2)) + sin(q_le(2))*(cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)) - cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0)))) + cos(q_le(3))*(cos(angle_fix_shd)*cos(q_le(1))*sin(q_le(0)) + sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(1))))) - sin(q_le(4))*sin(q_le(6))*(sin(q_le(3))*(cos(q_le(0))*cos(q_le(2)) + sin(q_le(2))*(cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)) - cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0)))) + cos(q_le(3))*(cos(angle_fix_shd)*cos(q_le(1))*sin(q_le(0)) + sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(1))))),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              L_palm*(sin(q_le(6))*(sin(q_le(4))*(cos(q_le(0))*sin(q_le(2)) - cos(q_le(2))*(cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)) - cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0)))) - cos(q_le(4))*(cos(q_le(3))*(cos(q_le(0))*cos(q_le(2)) + sin(q_le(2))*(cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)) - cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0)))) - sin(q_le(3))*(cos(angle_fix_shd)*cos(q_le(1))*sin(q_le(0)) + sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(1))))) + cos(q_le(6))*sin(q_le(5))*(cos(q_le(4))*(cos(q_le(0))*sin(q_le(2)) - cos(q_le(2))*(cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)) - cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0)))) + sin(q_le(4))*(cos(q_le(3))*(cos(q_le(0))*cos(q_le(2)) + sin(q_le(2))*(cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)) - cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0)))) - sin(q_le(3))*(cos(angle_fix_shd)*cos(q_le(1))*sin(q_le(0)) + sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)))))),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   L_palm*cos(q_le(6))*(sin(q_le(5))*(sin(q_le(3))*(cos(q_le(0))*cos(q_le(2)) + sin(q_le(2))*(cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)) - cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0)))) + cos(q_le(3))*(cos(angle_fix_shd)*cos(q_le(1))*sin(q_le(0)) + sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)))) + cos(q_le(5))*(sin(q_le(4))*(cos(q_le(0))*sin(q_le(2)) - cos(q_le(2))*(cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)) - cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0)))) - cos(q_le(4))*(cos(q_le(3))*(cos(q_le(0))*cos(q_le(2)) + sin(q_le(2))*(cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)) - cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0)))) - sin(q_le(3))*(cos(angle_fix_shd)*cos(q_le(1))*sin(q_le(0)) + sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)))))),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 L_palm*(sin(q_le(6))*(cos(q_le(5))*(sin(q_le(3))*(cos(q_le(0))*cos(q_le(2)) + sin(q_le(2))*(cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)) - cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0)))) + cos(q_le(3))*(cos(angle_fix_shd)*cos(q_le(1))*sin(q_le(0)) + sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)))) - sin(q_le(5))*(sin(q_le(4))*(cos(q_le(0))*sin(q_le(2)) - cos(q_le(2))*(cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)) - cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0)))) - cos(q_le(4))*(cos(q_le(3))*(cos(q_le(0))*cos(q_le(2)) + sin(q_le(2))*(cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)) - cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0)))) - sin(q_le(3))*(cos(angle_fix_shd)*cos(q_le(1))*sin(q_le(0)) + sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)))))) - cos(q_le(6))*(cos(q_le(4))*(cos(q_le(0))*sin(q_le(2)) - cos(q_le(2))*(cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)) - cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0)))) + sin(q_le(4))*(cos(q_le(3))*(cos(q_le(0))*cos(q_le(2)) + sin(q_le(2))*(cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)) - cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0)))) - sin(q_le(3))*(cos(angle_fix_shd)*cos(q_le(1))*sin(q_le(0)) + sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)))))),
L_palm*(cos(q_le(6))*(sin(q_le(5))*(sin(q_le(4))*(cos(q_le(2))*(cos(q_le(1))*pow(sin(angle_fix_shd),2)*sin(q_le(0)) - cos(angle_fix_shd)*sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(1))) + cos(q_le(0))*sin(angle_fix_shd)*sin(q_le(2))) + cos(q_le(4))*(cos(q_le(3))*(sin(q_le(2))*(cos(q_le(1))*pow(sin(angle_fix_shd),2)*sin(q_le(0)) - cos(angle_fix_shd)*sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(1))) - cos(q_le(0))*cos(q_le(2))*sin(angle_fix_shd)) + sin(q_le(3))*(sin(q_le(0))*sin(q_le(1))*pow(sin(angle_fix_shd),2) + cos(angle_fix_shd)*cos(q_le(1))*sin(q_le(0))*sin(angle_fix_shd)))) + cos(q_le(5))*(sin(q_le(3))*(sin(q_le(2))*(cos(q_le(1))*pow(sin(angle_fix_shd),2)*sin(q_le(0)) - cos(angle_fix_shd)*sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(1))) - cos(q_le(0))*cos(q_le(2))*sin(angle_fix_shd)) - cos(q_le(3))*(sin(q_le(0))*sin(q_le(1))*pow(sin(angle_fix_shd),2) + cos(angle_fix_shd)*cos(q_le(1))*sin(q_le(0))*sin(angle_fix_shd)))) - sin(q_le(6))*(cos(q_le(4))*(cos(q_le(2))*(cos(q_le(1))*pow(sin(angle_fix_shd),2)*sin(q_le(0)) - cos(angle_fix_shd)*sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(1))) + cos(q_le(0))*sin(angle_fix_shd)*sin(q_le(2))) - sin(q_le(4))*(cos(q_le(3))*(sin(q_le(2))*(cos(q_le(1))*pow(sin(angle_fix_shd),2)*sin(q_le(0)) - cos(angle_fix_shd)*sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(1))) - cos(q_le(0))*cos(q_le(2))*sin(angle_fix_shd)) + sin(q_le(3))*(sin(q_le(0))*sin(q_le(1))*pow(sin(angle_fix_shd),2) + cos(angle_fix_shd)*cos(q_le(1))*sin(q_le(0))*sin(angle_fix_shd))))) - L_arm*(sin(q_le(0))*sin(q_le(1))*pow(sin(angle_fix_shd),2) + cos(angle_fix_shd)*cos(q_le(1))*sin(q_le(0))*sin(angle_fix_shd)) + L_forearm*(sin(q_le(3))*(sin(q_le(2))*(cos(q_le(1))*pow(sin(angle_fix_shd),2)*sin(q_le(0)) - cos(angle_fix_shd)*sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(1))) - cos(q_le(0))*cos(q_le(2))*sin(angle_fix_shd)) - cos(q_le(3))*(sin(q_le(0))*sin(q_le(1))*pow(sin(angle_fix_shd),2) + cos(angle_fix_shd)*cos(q_le(1))*sin(q_le(0))*sin(angle_fix_shd))), L_forearm*(cos(q_le(3))*(cos(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) + sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + sin(q_le(2))*sin(q_le(3))*(sin(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) - cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd)))) - L_palm*(sin(q_le(6))*(sin(q_le(4))*(sin(q_le(3))*(cos(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) + sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - cos(q_le(3))*sin(q_le(2))*(sin(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) - cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd)))) + cos(q_le(2))*cos(q_le(4))*(sin(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) - cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd)))) + cos(q_le(6))*(sin(q_le(5))*(cos(q_le(4))*(sin(q_le(3))*(cos(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) + sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - cos(q_le(3))*sin(q_le(2))*(sin(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) - cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd)))) - cos(q_le(2))*sin(q_le(4))*(sin(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) - cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd)))) - cos(q_le(5))*(cos(q_le(3))*(cos(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) + sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + sin(q_le(2))*sin(q_le(3))*(sin(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) - cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd)))))) + L_arm*(cos(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) + sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))),   L_palm*(cos(q_le(6))*(sin(q_le(5))*(sin(q_le(4))*(sin(q_le(2))*(cos(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) + sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(q_le(2))*sin(angle_fix_shd)*sin(q_le(0))) - cos(q_le(3))*cos(q_le(4))*(cos(q_le(2))*(cos(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) + sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(2)))) - cos(q_le(5))*sin(q_le(3))*(cos(q_le(2))*(cos(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) + sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(2)))) - sin(q_le(6))*(cos(q_le(4))*(sin(q_le(2))*(cos(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) + sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(q_le(2))*sin(angle_fix_shd)*sin(q_le(0))) + cos(q_le(3))*sin(q_le(4))*(cos(q_le(2))*(cos(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) + sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(2))))) - L_forearm*sin(q_le(3))*(cos(q_le(2))*(cos(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) + sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(2))), - L_palm*(cos(q_le(6))*(cos(q_le(5))*(sin(q_le(3))*(sin(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) - cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(q_le(3))*(sin(q_le(2))*(cos(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) + sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(q_le(2))*sin(angle_fix_shd)*sin(q_le(0)))) + cos(q_le(4))*sin(q_le(5))*(cos(q_le(3))*(sin(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) - cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - sin(q_le(3))*(sin(q_le(2))*(cos(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) + sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(q_le(2))*sin(angle_fix_shd)*sin(q_le(0))))) + sin(q_le(4))*sin(q_le(6))*(cos(q_le(3))*(sin(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) - cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - sin(q_le(3))*(sin(q_le(2))*(cos(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) + sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(q_le(2))*sin(angle_fix_shd)*sin(q_le(0))))) - L_forearm*(sin(q_le(3))*(sin(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) - cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(q_le(3))*(sin(q_le(2))*(cos(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) + sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(q_le(2))*sin(angle_fix_shd)*sin(q_le(0)))), -L_palm*(sin(q_le(6))*(cos(q_le(4))*(sin(q_le(3))*(sin(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) - cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(q_le(3))*(sin(q_le(2))*(cos(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) + sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(q_le(2))*sin(angle_fix_shd)*sin(q_le(0)))) + sin(q_le(4))*(cos(q_le(2))*(cos(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) + sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(2)))) - cos(q_le(6))*sin(q_le(5))*(sin(q_le(4))*(sin(q_le(3))*(sin(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) - cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(q_le(3))*(sin(q_le(2))*(cos(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) + sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(q_le(2))*sin(angle_fix_shd)*sin(q_le(0)))) - cos(q_le(4))*(cos(q_le(2))*(cos(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) + sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(2))))), -L_palm*cos(q_le(6))*(sin(q_le(5))*(cos(q_le(3))*(sin(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) - cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - sin(q_le(3))*(sin(q_le(2))*(cos(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) + sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(q_le(2))*sin(angle_fix_shd)*sin(q_le(0)))) + cos(q_le(5))*(cos(q_le(4))*(sin(q_le(3))*(sin(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) - cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(q_le(3))*(sin(q_le(2))*(cos(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) + sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(q_le(2))*sin(angle_fix_shd)*sin(q_le(0)))) + sin(q_le(4))*(cos(q_le(2))*(cos(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) + sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(2))))), -L_palm*(cos(q_le(6))*(sin(q_le(4))*(sin(q_le(3))*(sin(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) - cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(q_le(3))*(sin(q_le(2))*(cos(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) + sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(q_le(2))*sin(angle_fix_shd)*sin(q_le(0)))) - cos(q_le(4))*(cos(q_le(2))*(cos(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) + sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(2)))) + sin(q_le(6))*(cos(q_le(5))*(cos(q_le(3))*(sin(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) - cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - sin(q_le(3))*(sin(q_le(2))*(cos(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) + sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(q_le(2))*sin(angle_fix_shd)*sin(q_le(0)))) - sin(q_le(5))*(cos(q_le(4))*(sin(q_le(3))*(sin(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) - cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(q_le(3))*(sin(q_le(2))*(cos(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) + sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(q_le(2))*sin(angle_fix_shd)*sin(q_le(0)))) + sin(q_le(4))*(cos(q_le(2))*(cos(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) + sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(2)))))),
L_arm*(cos(q_le(1))*sin(q_le(0))*pow(cos(angle_fix_shd),2) + sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(1))*cos(angle_fix_shd)) + L_forearm*(sin(q_le(3))*(sin(q_le(2))*(pow(cos(angle_fix_shd),2)*sin(q_le(0))*sin(q_le(1)) - cos(angle_fix_shd)*cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0))) + cos(angle_fix_shd)*cos(q_le(0))*cos(q_le(2))) + cos(q_le(3))*(cos(q_le(1))*sin(q_le(0))*pow(cos(angle_fix_shd),2) + sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(1))*cos(angle_fix_shd))) + L_palm*(cos(q_le(6))*(cos(q_le(5))*(sin(q_le(3))*(sin(q_le(2))*(pow(cos(angle_fix_shd),2)*sin(q_le(0))*sin(q_le(1)) - cos(angle_fix_shd)*cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0))) + cos(angle_fix_shd)*cos(q_le(0))*cos(q_le(2))) + cos(q_le(3))*(cos(q_le(1))*sin(q_le(0))*pow(cos(angle_fix_shd),2) + sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(1))*cos(angle_fix_shd))) + sin(q_le(5))*(sin(q_le(4))*(cos(q_le(2))*(pow(cos(angle_fix_shd),2)*sin(q_le(0))*sin(q_le(1)) - cos(angle_fix_shd)*cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0))) - cos(angle_fix_shd)*cos(q_le(0))*sin(q_le(2))) + cos(q_le(4))*(cos(q_le(3))*(sin(q_le(2))*(pow(cos(angle_fix_shd),2)*sin(q_le(0))*sin(q_le(1)) - cos(angle_fix_shd)*cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0))) + cos(angle_fix_shd)*cos(q_le(0))*cos(q_le(2))) - sin(q_le(3))*(cos(q_le(1))*sin(q_le(0))*pow(cos(angle_fix_shd),2) + sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(1))*cos(angle_fix_shd))))) - sin(q_le(6))*(cos(q_le(4))*(cos(q_le(2))*(pow(cos(angle_fix_shd),2)*sin(q_le(0))*sin(q_le(1)) - cos(angle_fix_shd)*cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0))) - cos(angle_fix_shd)*cos(q_le(0))*sin(q_le(2))) - sin(q_le(4))*(cos(q_le(3))*(sin(q_le(2))*(pow(cos(angle_fix_shd),2)*sin(q_le(0))*sin(q_le(1)) - cos(angle_fix_shd)*cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0))) + cos(angle_fix_shd)*cos(q_le(0))*cos(q_le(2))) - sin(q_le(3))*(cos(q_le(1))*sin(q_le(0))*pow(cos(angle_fix_shd),2) + sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(1))*cos(angle_fix_shd))))), L_forearm*(cos(q_le(3))*(sin(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) + cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - sin(q_le(2))*sin(q_le(3))*(cos(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) - sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd)))) - L_palm*(sin(q_le(6))*(sin(q_le(4))*(sin(q_le(3))*(sin(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) + cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(q_le(3))*sin(q_le(2))*(cos(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) - sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd)))) - cos(q_le(2))*cos(q_le(4))*(cos(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) - sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd)))) + cos(q_le(6))*(sin(q_le(5))*(cos(q_le(4))*(sin(q_le(3))*(sin(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) + cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(q_le(3))*sin(q_le(2))*(cos(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) - sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd)))) + cos(q_le(2))*sin(q_le(4))*(cos(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) - sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd)))) - cos(q_le(5))*(cos(q_le(3))*(sin(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) + cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - sin(q_le(2))*sin(q_le(3))*(cos(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) - sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd)))))) + L_arm*(sin(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) + cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))), - L_palm*(sin(q_le(6))*(cos(q_le(4))*(sin(q_le(2))*(sin(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) + cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - cos(angle_fix_shd)*cos(q_le(2))*sin(q_le(0))) + cos(q_le(3))*sin(q_le(4))*(cos(q_le(2))*(sin(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) + cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(2)))) - cos(q_le(6))*(sin(q_le(5))*(sin(q_le(4))*(sin(q_le(2))*(sin(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) + cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - cos(angle_fix_shd)*cos(q_le(2))*sin(q_le(0))) - cos(q_le(3))*cos(q_le(4))*(cos(q_le(2))*(sin(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) + cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(2)))) - cos(q_le(5))*sin(q_le(3))*(cos(q_le(2))*(sin(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) + cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(2))))) - L_forearm*sin(q_le(3))*(cos(q_le(2))*(sin(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) + cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(2))),   L_palm*(cos(q_le(6))*(cos(q_le(5))*(sin(q_le(3))*(cos(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) - sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - cos(q_le(3))*(sin(q_le(2))*(sin(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) + cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - cos(angle_fix_shd)*cos(q_le(2))*sin(q_le(0)))) + cos(q_le(4))*sin(q_le(5))*(cos(q_le(3))*(cos(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) - sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + sin(q_le(3))*(sin(q_le(2))*(sin(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) + cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - cos(angle_fix_shd)*cos(q_le(2))*sin(q_le(0))))) + sin(q_le(4))*sin(q_le(6))*(cos(q_le(3))*(cos(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) - sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + sin(q_le(3))*(sin(q_le(2))*(sin(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) + cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - cos(angle_fix_shd)*cos(q_le(2))*sin(q_le(0))))) + L_forearm*(sin(q_le(3))*(cos(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) - sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - cos(q_le(3))*(sin(q_le(2))*(sin(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) + cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - cos(angle_fix_shd)*cos(q_le(2))*sin(q_le(0)))),  L_palm*(sin(q_le(6))*(cos(q_le(4))*(sin(q_le(3))*(cos(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) - sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - cos(q_le(3))*(sin(q_le(2))*(sin(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) + cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - cos(angle_fix_shd)*cos(q_le(2))*sin(q_le(0)))) - sin(q_le(4))*(cos(q_le(2))*(sin(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) + cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(2)))) - cos(q_le(6))*sin(q_le(5))*(sin(q_le(4))*(sin(q_le(3))*(cos(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) - sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - cos(q_le(3))*(sin(q_le(2))*(sin(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) + cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - cos(angle_fix_shd)*cos(q_le(2))*sin(q_le(0)))) + cos(q_le(4))*(cos(q_le(2))*(sin(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) + cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(2))))),  L_palm*cos(q_le(6))*(sin(q_le(5))*(cos(q_le(3))*(cos(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) - sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + sin(q_le(3))*(sin(q_le(2))*(sin(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) + cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - cos(angle_fix_shd)*cos(q_le(2))*sin(q_le(0)))) + cos(q_le(5))*(cos(q_le(4))*(sin(q_le(3))*(cos(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) - sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - cos(q_le(3))*(sin(q_le(2))*(sin(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) + cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - cos(angle_fix_shd)*cos(q_le(2))*sin(q_le(0)))) - sin(q_le(4))*(cos(q_le(2))*(sin(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) + cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(2))))),  L_palm*(cos(q_le(6))*(sin(q_le(4))*(sin(q_le(3))*(cos(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) - sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - cos(q_le(3))*(sin(q_le(2))*(sin(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) + cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - cos(angle_fix_shd)*cos(q_le(2))*sin(q_le(0)))) + cos(q_le(4))*(cos(q_le(2))*(sin(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) + cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(2)))) + sin(q_le(6))*(cos(q_le(5))*(cos(q_le(3))*(cos(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) - sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + sin(q_le(3))*(sin(q_le(2))*(sin(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) + cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - cos(angle_fix_shd)*cos(q_le(2))*sin(q_le(0)))) - sin(q_le(5))*(cos(q_le(4))*(sin(q_le(3))*(cos(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) - sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - cos(q_le(3))*(sin(q_le(2))*(sin(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) + cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - cos(angle_fix_shd)*cos(q_le(2))*sin(q_le(0)))) - sin(q_le(4))*(cos(q_le(2))*(sin(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) + cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(2))))));


    
J_w_left_palm<< 
                  0,                     cos(q_le(0)),                                                                                         cos(angle_fix_shd)*cos(q_le(1))*sin(q_le(0)) + sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)),                                                                                                            cos(q_le(2))*(cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)) - cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0))) - cos(q_le(0))*sin(q_le(2)),                                                                                                                                                                                                    sin(q_le(3))*(cos(q_le(0))*cos(q_le(2)) + sin(q_le(2))*(cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)) - cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0)))) + cos(q_le(3))*(cos(angle_fix_shd)*cos(q_le(1))*sin(q_le(0)) + sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(1))),                                                                                                                                                                                                                                                                                                             - cos(q_le(4))*(cos(q_le(0))*sin(q_le(2)) - cos(q_le(2))*(cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)) - cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0)))) - sin(q_le(4))*(cos(q_le(3))*(cos(q_le(0))*cos(q_le(2)) + sin(q_le(2))*(cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)) - cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0)))) - sin(q_le(3))*(cos(angle_fix_shd)*cos(q_le(1))*sin(q_le(0)) + sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)))),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  - sin(q_le(5))*(sin(q_le(3))*(cos(q_le(0))*cos(q_le(2)) + sin(q_le(2))*(cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)) - cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0)))) + cos(q_le(3))*(cos(angle_fix_shd)*cos(q_le(1))*sin(q_le(0)) + sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)))) - cos(q_le(5))*(sin(q_le(4))*(cos(q_le(0))*sin(q_le(2)) - cos(q_le(2))*(cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)) - cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0)))) - cos(q_le(4))*(cos(q_le(3))*(cos(q_le(0))*cos(q_le(2)) + sin(q_le(2))*(cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(1)) - cos(q_le(1))*sin(angle_fix_shd)*sin(q_le(0)))) - sin(q_le(3))*(cos(angle_fix_shd)*cos(q_le(1))*sin(q_le(0)) + sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(1))))),
 cos(angle_fix_shd),  sin(angle_fix_shd)*sin(q_le(0)), cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd)) - sin(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)), cos(q_le(2))*(cos(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) + sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(2)), sin(q_le(3))*(sin(q_le(2))*(cos(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) + sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(q_le(2))*sin(angle_fix_shd)*sin(q_le(0))) - cos(q_le(3))*(sin(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) - cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))), cos(q_le(4))*(cos(q_le(2))*(cos(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) + sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(2))) - sin(q_le(4))*(sin(q_le(3))*(sin(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) - cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(q_le(3))*(sin(q_le(2))*(cos(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) + sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(q_le(2))*sin(angle_fix_shd)*sin(q_le(0)))),   sin(q_le(5))*(cos(q_le(3))*(sin(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) - cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - sin(q_le(3))*(sin(q_le(2))*(cos(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) + sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(q_le(2))*sin(angle_fix_shd)*sin(q_le(0)))) + cos(q_le(5))*(cos(q_le(4))*(sin(q_le(3))*(sin(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) - cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(q_le(3))*(sin(q_le(2))*(cos(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) + sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(q_le(2))*sin(angle_fix_shd)*sin(q_le(0)))) + sin(q_le(4))*(cos(q_le(2))*(cos(q_le(1))*(pow(cos(angle_fix_shd),2) + cos(q_le(0))*pow(sin(angle_fix_shd),2)) + sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - sin(angle_fix_shd)*sin(q_le(0))*sin(q_le(2)))),
sin(angle_fix_shd), -cos(angle_fix_shd)*sin(q_le(0)), cos(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) - sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd)), cos(q_le(2))*(sin(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) + cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(2)), cos(q_le(3))*(cos(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) - sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + sin(q_le(3))*(sin(q_le(2))*(sin(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) + cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - cos(angle_fix_shd)*cos(q_le(2))*sin(q_le(0))), sin(q_le(4))*(sin(q_le(3))*(cos(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) - sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - cos(q_le(3))*(sin(q_le(2))*(sin(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) + cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - cos(angle_fix_shd)*cos(q_le(2))*sin(q_le(0)))) + cos(q_le(4))*(cos(q_le(2))*(sin(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) + cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(2))), - sin(q_le(5))*(cos(q_le(3))*(cos(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) - sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + sin(q_le(3))*(sin(q_le(2))*(sin(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) + cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - cos(angle_fix_shd)*cos(q_le(2))*sin(q_le(0)))) - cos(q_le(5))*(cos(q_le(4))*(sin(q_le(3))*(cos(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) - sin(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - cos(q_le(3))*(sin(q_le(2))*(sin(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) + cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) - cos(angle_fix_shd)*cos(q_le(2))*sin(q_le(0)))) - sin(q_le(4))*(cos(q_le(2))*(sin(q_le(1))*(cos(q_le(0))*pow(cos(angle_fix_shd),2) + pow(sin(angle_fix_shd),2)) + cos(q_le(1))*(cos(angle_fix_shd)*sin(angle_fix_shd) - cos(angle_fix_shd)*cos(q_le(0))*sin(angle_fix_shd))) + cos(angle_fix_shd)*sin(q_le(0))*sin(q_le(2))));

};

void left_hand::doQP(VectorXd q_le){
jacob(q_le);
//  min 0.5 * x G x + g0 x
//  s.t.
//      CE^T x + ce0 = 0
//      CI^T x + ci0 >= 0
//  optCosts = solve_quadprog(G, g, CE, ce, CI, ci, x);


//    upbound(1:7,1)=min(((1/T)*((pi/180)*[45;90;45;-5;60;30;30]- q(:,i))),qdot_max);
//    lowbound(1:7,1)=max(((1/T)*((pi/180)*[-90;-10;-45;-90;-60;-30;-30] - q(:,i))),-qdot_max);
G.resize(7,7);
g.resize(7,1);

G=power*MatrixXd::Identity(7,7);
g.fill(0.0);

G=G+2*left_palm_position_power*J_left_palm.transpose()*J_left_palm;
g=g-2*left_palm_position_power*J_left_palm.transpose()*V;

if (dist<d_orient){
G=G+pow(tanh(5*(d_orient-dist)/d_orient),2)*left_palm_orientation_power*2*J_w_left_palm.transpose()*J_w_left_palm;
g=g+pow(tanh(5*(d_orient-dist)/d_orient),2)*left_palm_orientation_power*(-2)*J_w_left_palm.transpose()*w_left_palm;

}

vector<double> minimum(7);
vector<double> maximum(7);
minimum={-110.0,5.0,-60.0,-90.0,-90.0,-20.0,-20.0};
maximum={80.0 ,90.0 ,60.0 ,-5.0 ,90.0 ,20.0 ,20.0 };
//minimum={-180,-180,-180,-180,-180,-180,-180};
//maximum={180 ,180 ,180 ,180,180 ,180 ,180 };

    CI.resize(7,14);
    CI<<MatrixXd::Identity(7,7)*(-1),MatrixXd::Identity(7,7); //???????
    ci0.resize(14,1);
    ci0<<   min((toRad(maximum[0])-q_le(0))/T,qdot_max),
            min((toRad(maximum[1])-q_le(1))/T,qdot_max),//
            min((toRad(maximum[2])-q_le(2))/T,qdot_max),//
            min((toRad(maximum[3])-q_le(3))/T,qdot_max),
            min((toRad(maximum[4])-q_le(4))/T,qdot_max),//
            min((toRad(maximum[5])-q_le(5))/T,qdot_max),
            min((toRad(maximum[6])-q_le(6))/T,qdot_max),
            -max((toRad(minimum[0])-q_le(0))/T,-qdot_max),
            -max((toRad(minimum[1])-q_le(1))/T,-qdot_max),//
            -max((toRad(minimum[2])-q_le(2))/T,-qdot_max),//
            -max((toRad(minimum[3])-q_le(3))/T,-qdot_max),
            -max((toRad(minimum[4])-q_le(4))/T,-qdot_max),//
            -max((toRad(minimum[5])-q_le(5))/T,-qdot_max),
            -max((toRad(minimum[6])-q_le(6))/T,-qdot_max);


//CI.resize(7,1);
//ci0.resize(1,1);
//CI.fill(0);
//ci0.fill(1);




CE.resize(0,0);
//ce0.resize(0,0);

//CE.resize(7,1);
//ce0.resize(1,1);
//CE.fill(0);
//CE(1,0)=1;
//ce0.fill(0);


 qdot.resize(7,1);
    double optCosts;
  optCosts = solve_quadprog(G, g, CE, ce0, CI, ci0, qdot);
VectorXd temp=CI.transpose()*qdot + ci0;
//matrix_view(temp);


//qDebug()<<(toRad(maximum[1])-q_le(1))/T/2<<"\t"<<qdot(1)<<"\t"<<(toRad(minimum[1])-q_le(1))/T/2<<"\n\n\n";

q_next.resize(7,1);
q_next=q_le+T*qdot;

//????
if(q_next(0)<toRad(minimum[0])-.005){ q_next(0)=toRad(minimum[0]); cout<<"qr"<<1<<" out of range!";}
if(q_next(0)>toRad(maximum[0])+.005){ q_next(0)=toRad(maximum[0]); cout<<"qr"<<1<<" out of range!";}
if(q_next(1)<toRad(minimum[1])-.005){ q_next(1)=toRad(minimum[1]); cout<<"qr"<<2<<" out of range!";}
if(q_next(1)>toRad(maximum[1])+.005){ q_next(1)=toRad(maximum[1]); cout<<"qr"<<2<<" out of range!";}
if(q_next(2)<toRad(minimum[2])-.005){ q_next(2)=toRad(minimum[2]); cout<<"qr"<<3<<" out of range!";}
if(q_next(2)>toRad(maximum[2])+.005){ q_next(2)=toRad(maximum[2]); cout<<"qr"<<3<<" out of range!";}
if(q_next(3)<toRad(minimum[3])-.005){ q_next(3)=toRad(minimum[3]); cout<<"qr"<<4<<" out of range!";}
if(q_next(3)>toRad(maximum[3])+.005){ q_next(3)=toRad(maximum[3]); cout<<"qr"<<4<<" out of range!";}
if(q_next(4)<toRad(minimum[4])-.005){ q_next(4)=toRad(minimum[4]); cout<<"qr"<<5<<" out of range!";}
if(q_next(4)>toRad(maximum[4])+.005){ q_next(4)=toRad(maximum[4]); cout<<"qr"<<5<<" out of range!";}
if(q_next(5)<toRad(minimum[5])-.005){ q_next(5)=toRad(minimum[5]); cout<<"qr"<<6<<" out of range!";}
if(q_next(5)>toRad(maximum[5])+.005){ q_next(5)=toRad(maximum[5]); cout<<"qr"<<6<<" out of range!";}
if(q_next(6)<toRad(minimum[6])-.005){ q_next(6)=toRad(minimum[6]); cout<<"qr"<<7<<" out of range!";}
if(q_next(6)>toRad(maximum[6])+.005){ q_next(6)=toRad(maximum[6]); cout<<"qr"<<7<<" out of range!";}



}
double left_hand::wrist_pos2mot(double pos){
    double mot;
    mot=60*M_PI/63*tan(pos);
    return mot;

}
vector<int> left_hand::data2qc(vector<double> cntrl) {
    vector<int> qref(7);
    qref[0]=int((cntrl[0])*(1/(2*M_PI))*(2304)*120);
    qref[1]=int((cntrl[1])*(1/(2*M_PI))*(2304)*120);
    qref[2]=int((cntrl[2])*(1/(2*M_PI))*(2304)*120);
    qref[3]=int((cntrl[3])*(1/(2*M_PI))*(2304)*100);
    qref[4]=int(wrist_pos2mot(cntrl[4]));
    qref[5]=int(wrist_pos2mot(cntrl[5]));
    qref[6]=int(wrist_pos2mot(cntrl[6]));


    vector<double> minimum(7);
    vector<double> maximum(7);

    vector<int> minimumQC(7);
    vector<int> maximumQC(7);
    minimum={-90.0,-10.0,-45.0,-90.0,-60.0,-30.0,-30.0};
    maximum={45.0,90.0,45.0,-5.0,60.0,30.0,30.0};

    minimumQC[0]=int(minimum[0]*(1/(2*M_PI))*(2304)*120);
    minimumQC[1]=int(minimum[1]*(1/(2*M_PI))*(2304)*120);
    minimumQC[2]=int(minimum[2]*(1/(2*M_PI))*(2304)*120);
    minimumQC[3]=int(minimum[3]*(1/(2*M_PI))*(2304)*100);
    minimumQC[4]=int(wrist_pos2mot(minimum[4]));
    minimumQC[5]=int(wrist_pos2mot(minimum[5]));
    minimumQC[6]=int(wrist_pos2mot(minimum[6]));

    maximumQC[0]=int(maximum[0]*(1/(2*M_PI))*(2304)*120);
    maximumQC[1]=int(maximum[1]*(1/(2*M_PI))*(2304)*120);
    maximumQC[2]=int(maximum[2]*(1/(2*M_PI))*(2304)*120);
    maximumQC[3]=int(maximum[3]*(1/(2*M_PI))*(2304)*100);
    maximumQC[4]=int(wrist_pos2mot(maximum[4]));
    maximumQC[5]=int(wrist_pos2mot(maximum[5]));
    maximumQC[6]=int(wrist_pos2mot(maximum[6]));

    for (int i = 0; i < 12; ++i) {
        if(qref[i]<minimumQC[i]){qref[i]=minimumQC[i];}
        if(qref[i]>maximumQC[i]){qref[i]=maximumQC[i];}
    }


    for (int i = 0; i < 12; ++i) {
        if(qref[i]<minimumQC[i]){qref[i]=minimumQC[i];}
        if(qref[i]>maximumQC[i]){qref[i]=maximumQC[i];}
    }
    //    qref[0] = -qref[0];
    //    qref[1] = -qref[1];
    //    qref[2] = -qref[2];
    //    qref[3] = -qref[3];
    //    qref[4] = -qref[4];
    //    qref[5] = -qref[5];
    //    qref[6] = -qref[6];



    return qref;
}

vector<int> left_hand::data2qc_without_wrist(vector<double> cntrl) {
    vector<int> qref(4);
    qref[0]=int((cntrl[0])*(1/(2*M_PI))*(2304)*120);
    qref[1]=int((cntrl[1])*(1/(2*M_PI))*(2304)*120);
    qref[2]=int((cntrl[2])*(1/(2*M_PI))*(2304)*120);
    qref[3]=int((cntrl[3])*(1/(2*M_PI))*(2304)*100);


    vector<double> minimum(7);
    vector<double> maximum(7);

    vector<int> minimumQC(7);
    vector<int> maximumQC(7);
    minimum={-90.0,0.0,-45.0,-90.0,-60.0,-30.0,-30.0};
    maximum={45.0,90.0,45.0,-5.0,60.0,30.0,30.0};

     //maximum={14.0,26.0,75.0,35.0,26.0,14.0,75.0,35.0,23.0,10.0,10.0,23.0};

    minimumQC[0]=int(minimum[0]*(1/(2*M_PI))*(2304)*120);
    minimumQC[1]=int(minimum[1]*(1/(2*M_PI))*(2304)*120);
    minimumQC[2]=int(minimum[2]*(1/(2*M_PI))*(2304)*120);
    minimumQC[3]=int(minimum[3]*(1/(2*M_PI))*(2304)*100);


    maximumQC[0]=int(maximum[0]*(1/(2*M_PI))*(2304)*120);
    maximumQC[1]=int(maximum[1]*(1/(2*M_PI))*(2304)*120);
    maximumQC[2]=int(maximum[2]*(1/(2*M_PI))*(2304)*120);
    maximumQC[3]=int(maximum[3]*(1/(2*M_PI))*(2304)*100);


    for (int i = 0; i < 12; ++i) {
        if(qref[i]<minimumQC[i]){qref[i]=minimumQC[i];}
        if(qref[i]>maximumQC[i]){qref[i]=maximumQC[i];}
    }


    for (int i = 0; i < 12; ++i) {
        if(qref[i]<minimumQC[i]){qref[i]=minimumQC[i];}
        if(qref[i]>maximumQC[i]){qref[i]=maximumQC[i];}
    }
//    qref[0] = -qref[0];
//    qref[1] = -qref[1];
//    qref[2] = -qref[2];
//    qref[3] = -qref[3];




    return qref;
}

