#include "S5_hand.h"

// --- CONSTRUCTOR ---
// This is the core of the hand class. It initializes all hand-specific parameters.
S5_hand::S5_hand(HandType type) : hand_type(type) {
    if (hand_type == RIGHT) {
        angle_fix_shd = toRad(10);
        minimum = {-110.0, -90.0, -60.0, -90.0, -90.0, -20.0, -20.0};
        maximum = {80.0, -5.0, 60.0, -5.0, 90.0, 20.0, 20.0};
        wrist_clip_value = 85.0;
    } else { // LEFT
        angle_fix_shd = toRad(-10);
        minimum = {-110.0, 5.0, -60.0, -90.0, -90.0, -20.0, -20.0};
        maximum = {80.0, 90.0, 60.0, -5.0, 90.0, 20.0, 20.0};
        wrist_clip_value = 64.0;
    }
}

// --- UPDATE METHODS ---
void S5_hand::update_hand(VectorXd q_in, VectorXd r_target, MatrixXd R_target) {
    HO_FK_palm(q_in);
    dist = distance(r_target, r_palm);
    sai_target = sai_calc(R_target);
    theta_target = theta_calc(R_target);
    phi_target = phi_calc(R_target);
}

void S5_hand::update_hand(VectorXd q_in, VectorXd v, VectorXd r_target, MatrixXd R_target) {
    HO_FK_palm(q_in);
    dist = distance(r_target, r_palm);
    sai_target = sai_calc(R_target);
    theta_target = theta_calc(R_target);
    phi_target = phi_calc(R_target);

    V.resize(3, 1);
    V = v;

    double oriet_coef = 1;
    sai_dot = oriet_coef * (sai_target - sai);
    phi_dot = oriet_coef * (phi_target - phi);
    theta_dot = oriet_coef * (theta_target - theta);

    euler2w();
    calculate_jacobian(q_in, this->angle_fix_shd, this->J_palm, this->J_w_palm);
}

void S5_hand::update_hand(VectorXd q_in, VectorXd r_target, MatrixXd R_target, int i, double d0) {
    HO_FK_palm(q_in);
    dist = distance(r_target, r_palm);
    sai_target = sai_calc(R_target);
    theta_target = theta_calc(R_target);
    phi_target = phi_calc(R_target);

    V.resize(3, 1);
    // create smooth varying coef based on atan function (from 1 to 0)
    double v_coef = v_des * min(float(i) / 100.0, 1.0) * pow(atan(dist / d0 * 20.0) / M_PI * 2, 2) / dist;
    V = v_coef * (r_target - r_palm);

    double oriet_coef = 1;
    sai_dot = oriet_coef * (sai_target - sai);
    phi_dot = oriet_coef * (phi_target - phi);
    theta_dot = oriet_coef * (theta_target - theta);

    euler2w();
    calculate_jacobian(q_in, this->angle_fix_shd, this->J_palm, this->J_w_palm);
}

void S5_hand::update_hand(VectorXd q_in, VectorXd r_target, MatrixXd R_target, double d0, double v_0, double v__target) {
    HO_FK_palm(q_in);
    dist = distance(r_target, r_palm);
    sai_target = sai_calc(R_target);
    theta_target = theta_calc(R_target);
    phi_target = phi_calc(R_target);

    V.resize(3, 1);
    v0 = v_0;
    v_target = v__target;
    double v_coef = velocity(dist, d0);
    V = v_coef * (r_target - r_palm) / dist;

    double oriet_coef = 1;
    sai_dot = oriet_coef * (sai_target - sai);
    phi_dot = oriet_coef * (phi_target - phi);
    theta_dot = oriet_coef * (theta_target - theta);

    euler2w();
    calculate_jacobian(q_in, this->angle_fix_shd, this->J_palm, this->J_w_palm);
}

// --- KINEMATICS & CONTROL ---
// Calculate the Homogeneous Transformation for the palm using Forward Kinematics
void S5_hand::HO_FK_palm(VectorXd q_in) {
    R1_fix_shd.resize(4, 4); R2_fix_shd.resize(4, 4);
    R1_ra.resize(4, 4); R2_ra.resize(4, 4); R3_ra.resize(4, 4);
    R1_fix_elbow.resize(4, 4); R2_fix_elbow.resize(4, 4);
    R4_ra.resize(4, 4); R5_ra.resize(4, 4); R6_ra.resize(4, 4); R7_ra.resize(4, 4);

    R1_fix_shd = rot(1, -angle_fix_shd, 4);
    R2_fix_shd = rot(1, angle_fix_shd, 4);
    R1_ra = rot(2, q_in(0), 4);
    R2_ra = rot(1, q_in(1), 4);
    R3_ra = rot(3, q_in(2), 4);
    R1_fix_elbow = rot(3, +angle_fix_elbow, 4);
    R2_fix_elbow = rot(3, -angle_fix_elbow, 4);
    R4_ra = rot(2, q_in(3), 4);
    R5_ra = rot(3, q_in(4), 4);
    R6_ra = rot(1, q_in(5), 4);
    R7_ra = rot(2, q_in(6), 4);

    P_arm_ra.resize(4, 4); P_forearm_ra.resize(4, 4); P_palm_ra.resize(4, 4);

    P_arm_ra = trans(3, -L_arm);
    P_forearm_ra = trans(3, -L_forearm);
    P_palm_ra = trans(3, -L_palm);

    T_palm.resize(4, 4);
    T_palm = R1_fix_shd * R1_ra * R2_fix_shd * R2_ra * R3_ra * P_arm_ra * R1_fix_elbow * R4_ra * R5_ra * P_forearm_ra * R2_fix_elbow * R6_ra * R7_ra * P_palm_ra;

    r_palm.resize(3, 1);
    r_palm << T_palm.block(0, 3, 3, 1);
    R_palm.resize(3, 3);
    R_palm << T_palm.block(0, 0, 3, 3);

    theta = theta_calc(R_palm);
    sai = sai_calc(R_palm);
    phi = phi_calc(R_palm);
}

void S5_hand::doQP(VectorXd q_in) {
    calculate_jacobian(q_in, this->angle_fix_shd, this->J_palm, this->J_w_palm);

    G.resize(7, 7);
    g.resize(7, 1);

    G = power * MatrixXd::Identity(7, 7);
    g.fill(0.0);

    G = G + 2 * palm_position_power * J_palm.transpose() * J_palm;
    g = g - 2 * palm_position_power * J_palm.transpose() * V;
    if (dist < d_orient) {
        G = G + pow(tanh(5 * (d_orient - dist) / d_orient), 2) * palm_orientation_power * 2 * J_w_palm.transpose() * J_w_palm;
        g = g + pow(tanh(5 * (d_orient - dist) / d_orient), 2) * palm_orientation_power * (-2) * J_w_palm.transpose() * w_palm;
    }

    CI.resize(7, 14);
    CI << MatrixXd::Identity(7, 7) * (-1), MatrixXd::Identity(7, 7);
    ci0.resize(14, 1);
    ci0 << min((toRad(maximum[0]) - q_in(0)) / T, qdot_max),
           min((toRad(maximum[1]) - q_in(1)) / T, qdot_max),
           min((toRad(maximum[2]) - q_in(2)) / T, qdot_max),
           min((toRad(maximum[3]) - q_in(3)) / T, qdot_max),
           min((toRad(maximum[4]) - q_in(4)) / T, qdot_max),
           min((toRad(maximum[5]) - q_in(5)) / T, qdot_max),
           min((toRad(maximum[6]) - q_in(6)) / T, qdot_max),
           -max((toRad(minimum[0]) - q_in(0)) / T, -qdot_max),
           -max((toRad(minimum[1]) - q_in(1)) / T, -qdot_max),
           -max((toRad(minimum[2]) - q_in(2)) / T, -qdot_max),
           -max((toRad(minimum[3]) - q_in(3)) / T, -qdot_max),
           -max((toRad(minimum[4]) - q_in(4)) / T, -qdot_max),
           -max((toRad(minimum[5]) - q_in(5)) / T, -qdot_max),
           -max((toRad(minimum[6]) - q_in(6)) / T, -qdot_max);

    CE.resize(0, 0);
    ce0.resize(0);

    qdot.resize(7, 1);
    solve_quadprog(G, g, CE, ce0, CI, ci0, qdot);
    
    q_next.resize(7, 1);
    q_next = q_in + T * qdot;

    if(q_next(0)<toRad(minimum[0])-.005){ q_next(0)=toRad(minimum[0]); cout<<"q1 out of range!"<<endl;}
    if(q_next(0)>toRad(maximum[0])+.005){ q_next(0)=toRad(maximum[0]); cout<<"q1 out of range!"<<endl;}
    if(q_next(1)<toRad(minimum[1])-.005){ q_next(1)=toRad(minimum[1]); cout<<"q2 out of range!"<<endl;}
    if(q_next(1)>toRad(maximum[1])+.005){ q_next(1)=toRad(maximum[1]); cout<<"q2 out of range!"<<endl;}
    if(q_next(2)<toRad(minimum[2])-.005){ q_next(2)=toRad(minimum[2]); cout<<"q3 out of range!"<<endl;}
    if(q_next(2)>toRad(maximum[2])+.005){ q_next(2)=toRad(maximum[2]); cout<<"q3 out of range!"<<endl;}
    if(q_next(3)<toRad(minimum[3])-.005){ q_next(3)=toRad(minimum[3]); cout<<"q4 out of range!"<<endl;}
    if(q_next(3)>toRad(maximum[3])+.005){ q_next(3)=toRad(maximum[3]); cout<<"q4 out of range!"<<endl;}
    if(q_next(4)<toRad(minimum[4])-.005){ q_next(4)=toRad(minimum[4]); cout<<"q5 out of range!"<<endl;}
    if(q_next(4)>toRad(maximum[4])+.005){ q_next(4)=toRad(maximum[4]); cout<<"q5 out of range!"<<endl;}
    if(q_next(5)<toRad(minimum[5])-.005){ q_next(5)=toRad(minimum[5]); cout<<"q6 out of range!"<<endl;}
    if(q_next(5)>toRad(maximum[5])+.005){ q_next(5)=toRad(maximum[5]); cout<<"q6 out of range!"<<endl;}
    if(q_next(6)<toRad(minimum[6])-.005){ q_next(6)=toRad(minimum[6]); cout<<"q7 out of range!"<<endl;}
    if(q_next(6)>toRad(maximum[6])+.005){ q_next(6)=toRad(maximum[6]); cout<<"q7 out of range!"<<endl;}
}

// --- UTILITY & TRANSFORMATION FUNCTIONS ---
double S5_hand::toRad(double d) { 
    return d * M_PI / 180.0; 
}

MatrixXd S5_hand::rot(int axis, double q, int dim) {
    MatrixXd R;
    if (dim == 3) {
        R.resize(3, 3);
        if (axis == 1) { R << 1, 0, 0, 0, cos(q), -sin(q), 0, sin(q), cos(q); }
        else if (axis == 2) { R << cos(q), 0, sin(q), 0, 1, 0, -sin(q), 0, cos(q); }
        else if (axis == 3) { R << cos(q), -sin(q), 0, sin(q), cos(q), 0, 0, 0, 1; }
    } else if (dim == 4) {
        R = MatrixXd::Identity(4, 4);
        if (axis == 1) { R.block(1, 1, 2, 2) << cos(q), -sin(q), sin(q), cos(q); }
        else if (axis == 2) { R(0,0)=cos(q); R(0,2)=sin(q); R(2,0)=-sin(q); R(2,2)=cos(q); }
        else if (axis == 3) { R.block(0, 0, 2, 2) << cos(q), -sin(q), sin(q), cos(q); }
    }
    return R;
}

MatrixXd S5_hand::trans(int axis, double d) {
    MatrixXd H = MatrixXd::Identity(4, 4);
    H(axis - 1, 3) = d;
    return H;
}

MatrixXd S5_hand::trans(Vector3d d) {
    MatrixXd H = MatrixXd::Identity(4, 4);
    H.block(0, 3, 3, 1) = d;
    return H;
}

double S5_hand::distance(VectorXd V1, VectorXd V2) {
    return (V1 - V2).norm();
}

double S5_hand::velocity(double d, double d0) {
    double a3 = 2 * (v_target - v0) / (d0 * d0 * d0);
    double a2 = -1.5 * a3 * d0;
    return v_target + a2 * d * d + a3 * d * d * d;
}

void S5_hand::euler2w() {
    w_palm.resize(3);
    w_palm << sin(phi) * theta_dot + cos(phi) * cos(theta) * sai_dot,
              sin(theta) * sai_dot + phi_dot,
              cos(phi) * theta_dot - cos(theta) * sin(phi) * sai_dot;
}

double S5_hand::phi_calc(MatrixXd R) {
    if (abs(R(1, 0)) > 0.9999) { return atan2(R(0, 2), R(2, 2)); }
    else { return atan2(-R(2, 0), R(0, 0)); }
}

double S5_hand::theta_calc(MatrixXd R) {
    return asin(R(1, 0));
}

double S5_hand::sai_calc(MatrixXd R) {
    if (abs(R(1, 0)) > 0.9999) { return 0; }
    else { return atan2(-R(1, 2), R(1, 1)); }
}

MatrixXd S5_hand::ObjToNeck(double h_pitch, double h_roll, double h_yaw) {
    T0.resize(4, 4);
    T0 << cos(M_PI / 9), 0, sin(M_PI / 9), 0, 0, 1, 0, 0, -sin(M_PI / 9), 0, cos(M_PI / 9), 0, 0, 0, 0, 1;
    T1 = trans(Vector3d(camera[0], camera[1], camera[2]));
    T2 = trans(Vector3d(0, 0, head_PtoR)) * rot(1, h_roll, 4);
    T3 = rot(2, h_pitch, 4);
    T4 = rot(3, h_yaw, 4) * trans(Vector3d(0, 0, head_YtoP));
    T5 = trans(Vector3d(Shoulder2Head[0], Shoulder2Head[1], Shoulder2Head[2]));
    
    T_EEtobase.resize(4, 4);
    T_EEtobase = T5 * T4 * T3 * T2 * T1 * T0;
    return T_EEtobase;
}

MatrixXd S5_hand::returnAngles(MatrixXd T_EEtobase) {
    
    MatrixXd output;
    double theta_pitch, sai_roll, phi_yaw;
    output.resize(3,1);

    if (T_EEtobase(2, 0) != 1 && T_EEtobase(2, 0) != -1)
    {
        theta_pitch = -asin(T_EEtobase(2, 0));
        sai_roll = atan2(T_EEtobase(2, 1) / cos(theta_pitch), T_EEtobase(2, 2) / cos(theta_pitch));
        phi_yaw = atan2(T_EEtobase(1, 0) / cos(theta_pitch), T_EEtobase(0, 0) / cos(theta_pitch));
    }
    else{
            phi_yaw = 0;
            if (T_EEtobase(2, 0) != -1)
            {
                theta_pitch = M_PI / 2;
                sai_roll = atan2(T_EEtobase(0, 1), T_EEtobase(0, 2));
            }
            else{
                theta_pitch = -M_PI / 2;
                sai_roll = atan2(-T_EEtobase(0, 1), -T_EEtobase(0, 2));
                }
    }
    output<< phi_yaw,sai_roll,theta_pitch;
    return output;

}

// --- WRIST IK ---
VectorXd S5_hand::solveQuadratic(double a, double b, double c) {
    double discriminant = b * b - 4 * a * c;
    VectorXd Roots(2);
    if (discriminant >= 0) {
        Roots(0) = (-b + sqrt(discriminant)) / (2 * a);
        Roots(1) = (-b - sqrt(discriminant)) / (2 * a);
    } else {
        // Return real part if complex, though this case might need specific handling
        Roots(0) = -b / (2 * a);
        Roots(1) = sqrt(-discriminant) / (2 * a);
    }
    return Roots;
}

double S5_hand::wrist_right_calc(double alpha, double beta) {
    double a1=10, b1=79, c1=5, r1=24.9723, pz=-75.885, phi=255.3027*M_PI/180, gama=104.6973*M_PI/180;
    Vector3d r1_(r1*cos(phi), r1*sin(phi), 0);
    Vector3d P(0,0,pz);
    Vector3d h1_P(20,17,0);
    Matrix3d R_PtoO = rot(3,gama,3)*rot(2,beta,3)*rot(1,alpha,3);
    Vector3d h1_O = P+R_PtoO*h1_P;
    Matrix3d R_OtoA1=rot(3,phi,3);
    Vector3d h1_A1=r1_+R_OtoA1*h1_O;
    double K1 = -2*a1*h1_A1(0);
    double M1 = 2*a1*h1_A1(2);
    double N1 = pow(a1,2)+pow(h1_A1.norm(),2)+pow(c1,2)-pow(b1,2)-2*c1*h1_A1(1);
    VectorXd Roots = solveQuadratic(N1-K1, 2*M1, K1+N1);
    Vector2d result(atan(Roots(0))*2*180/M_PI, atan(Roots(1))*2*180/M_PI);
    if(result(0)<0 && result(1)<0 ){
        result = -result; }
    else{
        result = result - Vector2d(180,180); }
    double tempRes = (abs(result(0)) < abs(result(1))) ? result(0) : result(1);
    
    if (abs(tempRes) >= wrist_clip_value){
        tempRes = (tempRes > 0) ? wrist_clip_value : -wrist_clip_value;
    }
    return tempRes;
}

double S5_hand::wrist_left_calc(double alpha, double beta) {
    double a1=10, b1=56, c1=5, r1=28.2062, pz=-53.4850, phi=301.0876*M_PI/180, gama=58.9124*M_PI/180;
    Vector3d r1_(r1*cos(phi), r1*sin(phi), 0);
    Vector3d P(0,0,pz);
    Vector3d h1_P(-20,17,0);
    Matrix3d R_PtoO=rot(3,gama,3)*rot(2,beta,3)*rot(1,alpha,3);
    Vector3d h1_O = P+R_PtoO*h1_P;
    Matrix3d R_OtoA1=rot(3,phi,3);
    Vector3d h1_A1=r1_+R_OtoA1*h1_O;
    double K1 = -2*a1*h1_A1(0);
    double M1 = 2*a1*h1_A1(2);
    double N1 = pow(a1,2)+pow(h1_A1.norm(),2)+pow(c1,2)-pow(b1,2)-2*c1*h1_A1(1);
    VectorXd Roots = solveQuadratic(N1-K1, 2*M1, K1+N1);
    Vector2d result(atan(Roots(0))*2*180/M_PI, atan(Roots(1))*2*180/M_PI);
    double tempRes = (abs(result(0)) < abs(result(1))) ? result(0) : result(1);

    if (abs(tempRes) >= wrist_clip_value){
        tempRes = (tempRes > 0) ? wrist_clip_value : -wrist_clip_value;
    }
    return tempRes;
}

// --- SIMULATION & HARDWARE ---
double S5_hand::move2pose(double max, double t_local, double T_start, double T_end) {
    double T_move = T_end - T_start;
    double c3 = (10 * max) / pow(T_move, 3);
    double c4 = -(15 * max) / pow(T_move, 4);
    double c5 = (6 * max) / pow(T_move, 5);
    double theta = 0;
    if (t_local < T_start) { 
        theta = 0; }
    else if (t_local < T_end) { 
        theta = c3 * pow(t_local - T_start, 3) + c4 * pow(t_local - T_start, 4) + c5 * pow(t_local - T_start, 5); }
    else { 
        theta = max; }
    return theta;
}

double S5_hand::wrist_pos2mot(double pos) {
    return 60 * M_PI / 63 * tan(pos);
}

vector<int> S5_hand::data2qc(vector<double> cntrl) {
    vector<int> qref(7);
    qref[0] = int((cntrl[0]) * (1 / (2 * M_PI)) * (2304) * 120);
    qref[1] = int((cntrl[1]) * (1 / (2 * M_PI)) * (2304) * 120);
    qref[2] = int((cntrl[2]) * (1 / (2 * M_PI)) * (2304) * 120);
    qref[3] = int((cntrl[3]) * (1 / (2 * M_PI)) * (2304) * 100);
    qref[4] = int(wrist_pos2mot(cntrl[4]));
    qref[5] = int(wrist_pos2mot(cntrl[5]));
    qref[6] = int(wrist_pos2mot(cntrl[6]));

    vector<int> minimumQC(7);
    vector<int> maximumQC(7);
    // These hardcoded limits should ideally not be needed if QP is correct, but kept for safety.
    vector<double> min_deg = {-90.0,-10.0,-45.0,-90.0,-60.0,-30.0,-30.0};
    vector<double> max_deg = {45.0,90.0,45.0,-5.0,60.0,30.0,30.0};

    minimumQC[0]=int(toRad(min_deg[0])*(1/(2*M_PI))*(2304)*120);
    minimumQC[1]=int(toRad(min_deg[1])*(1/(2*M_PI))*(2304)*120);
    minimumQC[2]=int(toRad(min_deg[2])*(1/(2*M_PI))*(2304)*120);
    minimumQC[3]=int(toRad(min_deg[3])*(1/(2*M_PI))*(2304)*100);
    minimumQC[4]=int(wrist_pos2mot(toRad(min_deg[4])));
    minimumQC[5]=int(wrist_pos2mot(toRad(min_deg[5])));
    minimumQC[6]=int(wrist_pos2mot(toRad(min_deg[6])));

    maximumQC[0]=int(toRad(max_deg[0])*(1/(2*M_PI))*(2304)*120);
    maximumQC[1]=int(toRad(max_deg[1])*(1/(2*M_PI))*(2304)*120);
    maximumQC[2]=int(toRad(max_deg[2])*(1/(2*M_PI))*(2304)*120);
    maximumQC[3]=int(toRad(max_deg[3])*(1/(2*M_PI))*(2304)*100);
    maximumQC[4]=int(wrist_pos2mot(toRad(max_deg[4])));
    maximumQC[5]=int(wrist_pos2mot(toRad(max_deg[5])));
    maximumQC[6]=int(wrist_pos2mot(toRad(max_deg[6])));
    
    for (int i = 0; i < 7; ++i) {
        if (qref[i] < minimumQC[i]) { qref[i] = minimumQC[i]; }
        if (qref[i] > maximumQC[i]) { qref[i] = maximumQC[i]; }
    }
    return qref;
}