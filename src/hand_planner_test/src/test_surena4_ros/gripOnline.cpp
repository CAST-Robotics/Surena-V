#include "S5mod_right_hand.h"
#include "S5mod_left_hand.h"
#include"MinimumJerkInterpolation.h"
#include "../Eigen/Dense"
#include "../Eigen/Geometry"
#include "../Eigen/QuadProg.h"
#include "../Eigen/testQPsolvers.hpp"
#include "../Eigen/eiquadprog.hpp"
#include "../Eigen/Core"
#include "../Eigen/Cholesky"
#include "../Eigen/LU"
#include <iostream>
#include <vector>
#include "fstream"
#include <string>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Int32MultiArray.h>
#include "hand_planner_test/DetectionInfoArray.h"
#include "ros/ros.h"
#include "hand_planner_test/move_hand_single.h"
#include "hand_planner_test/move_hand_both.h"
#include "hand_planner_test/gripOnline.h"
#include "hand_planner_test/home_service.h"

using namespace  std;
using namespace  Eigen;


class test_forgotten_srv{
    public:
        test_forgotten_srv(ros::NodeHandle *n){
            trajectory_data_pub  = n->advertise<std_msgs::Int32MultiArray>("jointdata/qc",100);
            camera_data_sub  = n->subscribe("/detection_info", 1, &test_forgotten_srv::object_detect,this);
            JointQcSub  = n->subscribe("jointdata/qc",100, &test_forgotten_srv::jointQC_sub,this);
            move_hand_single_service = n->advertiseService("move_hand_single_srv", &test_forgotten_srv::single,this);
            move_hand_both_service = n->advertiseService("move_hand_both_srv", &test_forgotten_srv::both,this);
            grip_Online_seivice = n->advertiseService("grip_Online_srv", &test_forgotten_srv::grip_Online,this);
            home_service = n->advertiseService("home_srv", &test_forgotten_srv::home,this);
        }
        void object_detect(const hand_planner_test::DetectionInfoArray & msg){
            if (msg.detections[0].class_id == 41 && msg.detections[0].distance != 0){
                dist = (msg.detections[0].distance)/1000;
                y = (msg.detections[0].x + (msg.detections[0].width)/2);
                z = (msg.detections[0].y + (msg.detections[0].height)/2);

                Y0 = -(y-L/2)/L*a;
                Z0 = -(z-W/2)/W*b;
                L0 = sqrt(pow(X0,2)+pow(Y0,2)+pow(Z0,2));

                X = X0*dist/L0;
                Y = Y0*dist/L0;
                Z = Z0*dist/L0;
                tempX = X;
                tempY = Y;
                tempZ = Z;
        
            }
            else{
                X = tempX;
                Y = tempY;
                Z = tempZ;
            }
            // cout<<"camera-> "<<"X: "<<X<<", Y: "<<Y<<", Z: "<<Z<<endl;
            }

        void jointQC_sub(const std_msgs::Int32MultiArray::ConstPtr & qcArray){
            int i = 0;
            // print all the remaining numbers
            for(std::vector<int>::const_iterator it = qcArray->data.begin(); it != qcArray->data.end(); ++it)
            {
                QcArr[i] = *it;
                i++;
            }
            //cout<<QcArr[0]<<", "<<QcArr[1]<<", "<<QcArr[2]<<", "<<QcArr[3]<<", "<<QcArr[4]<<", "<<QcArr[5]<<", "<<QcArr[6]<<", "<<QcArr[7]<<", "<<QcArr[8]<<", "<<QcArr[9]<<", "<<QcArr[10]<<", "<<QcArr[11]<<", "<<QcArr[12]<<", "<<QcArr[13]<<", "<<QcArr[14]<<", "<<QcArr[15]<<", "<<QcArr[16]<<", "<<QcArr[17]<<", "<<QcArr[18]<<", "<<QcArr[19]<<endl;
            return;
            }

        MatrixXd scenario_target_R (string scenario, int i, VectorXd ee_pos, string ee_ini_pos){
            MatrixXd result_r(6,3);
            r_start_r.resize(3);
            r_middle_r.resize(3);
            r_target_r.resize(3);
            R_target_r.resize(3,3);
            q_ra.resize(7);
            q_init_r.resize(7); 
            if (scenario=="shakeHands"){
                r_middle_r<<0.35,-0.1,-0.2  ; //shakehands
                r_target_r<<0.3,-0.03,-0.3;
                R_target_r=hand_func_R.rot(2,-65*M_PI/180,3);
            }
            else if (scenario=="Respect"){
                r_middle_r<<0.3,-0.1,-0.3  ; //respect
                r_target_r<<0.3,0.1,-0.3;
                R_target_r=hand_func_R.rot(2,-80*M_PI/180,3)*hand_func_R.rot(1,60*M_PI/180,3);
            }
            else if (scenario=="byebye"){
                r_middle_r<<0.35,-0.2,-0.15  ; //ByeBye
                r_target_r<<0.3,-0.1,0.22;
                R_target_r=hand_func_R.rot(3,90*M_PI/180,3)*hand_func_R.rot(1,-180*M_PI/180,3);
            }
            else if (scenario=="home"){
                r_middle_r<<0.3,-0.1,-0.25  ; //home
                r_target_r<<0.15,-0.07,-0.43;
                R_target_r=hand_func_R.rot(2,-20*M_PI/180,3);
            }
            else if (scenario=="fixed"){
                r_middle_r<<0.2,-0.1,-0.35  ; //fixed
                r_target_r<<0.15,-0.07,-0.43;
                R_target_r=hand_func_R.rot(2,-20*M_PI/180,3);
            }
                if (i==0){
                    if (ee_ini_pos=="init") {
                        //q_ra<<-12.3*M_PI/180,-5*M_PI/180,38*M_PI/180,-5*M_PI/180,0,0,0; // initial condition
                        q_ra<<10*M_PI/180,-10*M_PI/180,0,-25*M_PI/180,0,0,0; // initial condition
                        q_init_r = q_ra;
                        // define right_hand objs
                        right_hand hand0_r(q_ra,r_target_r,R_target_r,0,0);
                        r_start_r=hand0_r.r_right_palm;

                        d0_r=hand0_r.dist;       
                        d_r=d0_r;                       
                        d_des_r=hand0_r.d_des;             
                        theta_r=hand0_r.theta;           
                        theta_target_r=hand0_r.theta_target;
                        sai_r=hand0_r.sai;                
                        sai_target_r=hand0_r.sai_target;     
                        phi_r=hand0_r.phi;                 
                        phi_target_r=hand0_r.phi_target;    
                        hand0_r.HO_FK_right_palm(q_ra);
                        cout<<r_start_r<<endl;;
                    }
                    else{
                        r_start_r = next_ini_ee_posR;
                        cout<<r_start_r<<endl;
                    }   
                }
                else{
                r_start_r =  ee_pos;
                }
            result_r << r_middle_r(0),r_middle_r(1),r_middle_r(2),
                    r_target_r(0),r_target_r(1),r_target_r(2),
                    R_target_r(0,0),R_target_r(0,1),R_target_r(0,2),
                    R_target_r(1,0),R_target_r(1,1),R_target_r(1,2),
                    R_target_r(2,0),R_target_r(2,1),R_target_r(2,2),
                    r_start_r(0),r_start_r(1),r_start_r(2);
            return result_r;
            }

        MatrixXd scenario_target_L (string scenario, int i, VectorXd ee_pos, string ee_ini_pos){      
            MatrixXd result_l(6,3);
            r_start_l.resize(3);
            r_middle_l.resize(3);
            r_target_l.resize(3);
            R_target_l.resize(3,3);
            q_la.resize(7);
            q_init_l.resize(7);   
            if (scenario=="shakeHands"){
                r_middle_l<<0.35,0.1,-0.2  ; //shakehands
                r_target_l<<0.3,0.03,-0.3;
                R_target_l=hand_func_L.rot(2,-65*M_PI/180,3);
            }
            else if (scenario=="Respect"){
                r_middle_l<<0.3,0.1,-0.3  ; //respect
                r_target_l<<0.3,-0.1,-0.3;
                R_target_l=hand_func_L.rot(2,-80*M_PI/180,3)*hand_func_L.rot(1,-60*M_PI/180,3);
            }
            else if (scenario=="byebye"){
                r_middle_l<<0.35,0.2,-0.15  ; //ByeBye
                r_target_l<<0.3,0.1,0.22;
                R_target_l=hand_func_L.rot(3,90*M_PI/180,3)*hand_func_L.rot(2,-180*M_PI/180,3);
            }
            else if (scenario=="home"){
                r_middle_l<<0.3,0.1,-0.25; //home
                r_target_l<<0.15,0.07,-0.43;
                R_target_l=hand_func_L.rot(2,-20*M_PI/180,3);
            }
            else if (scenario=="fixed"){
                r_middle_l<<0.2,0.1,-0.35  ; //fixed
                r_target_l<<0.15,0.07,-0.43;
                R_target_l=hand_func_L.rot(2,-20*M_PI/180,3);
            }
                if (i==0){
                    if (ee_ini_pos=="init") {
                        //q_la<<-12.3*M_PI/180,5*M_PI/180,-38*M_PI/180,-5*M_PI/180,0,0,0; // initial condition
                        q_la<<10*M_PI/180, 10*M_PI/180,0,-25*M_PI/180,0,0,0; // initial condition
                        q_init_l = q_la;
                        // define right_hand objs
                        left_hand hand0_l( q_la, r_target_l,  R_target_l,0,0);
                        r_start_l=hand0_l.r_left_palm;

                        d0_l=hand0_l.dist;
                        d_l=d0_l;
                        d_des_l=hand0_l.d_des;
                        theta_l=hand0_l.theta;
                        theta_target_l=hand0_l.theta_target;
                        sai_l=hand0_l.sai; 
                        sai_target_l=hand0_l.sai_target;
                        phi_l=hand0_l.phi; 
                        phi_target_l=hand0_l.phi_target;
                        hand0_l.HO_FK_left_palm(q_la);
                        cout<<r_start_l<<endl;
                    }
                    else {
                        r_start_l = next_ini_ee_posL;
                        cout<<r_start_l<<endl;
                    }
                }
                else{
                r_start_l =  ee_pos;
                }
            result_l << r_middle_l(0),r_middle_l(1),r_middle_l(2),
                    r_target_l(0),r_target_l(1),r_target_l(2),
                    R_target_l(0,0),R_target_l(0,1),R_target_l(0,2),
                    R_target_l(1,0),R_target_l(1,1),R_target_l(1,2),
                    R_target_l(2,0),R_target_l(2,1),R_target_l(2,2),
                    r_start_l(0),r_start_l(1),r_start_l(2);
            return result_l;
}

        VectorXd reach_target_L(MatrixXd targets, string scenario, int M){
            qref_l.resize(7,M);
            P_x_l.resize(1,3); V_x_l.resize(1,3); A_x_l.resize(1,3);
            P_y_l.resize(1,3); V_y_l.resize(1,3); A_y_l.resize(1,3);
            P_z_l.resize(1,3); V_z_l.resize(1,3); A_z_l.resize(1,3);
            P_l.resize(3); V_l.resize(3);
            // define time parameters
            int count = 0;
            time_r=count*T; 
            MatrixXd t_r(1,3);
            t_r<<0,2,4;
            
            if (scenario=="byebye" || scenario=="shakeHands"){
                total = t_r(2) + 4;
                sum_l+=total;
            }
            else {
                total = t_r(2);
                sum_l+=total;
            }

            // set target values
            r_middle_l<<targets(0,0),targets(0,1),targets(0,2); //shakehands
            r_target_l<<targets(1,0),targets(1,1),targets(1,2);
            R_target_l<<targets(2,0),targets(2,1),targets(2,2),
                        targets(3,0),targets(3,1),targets(3,2),
                        targets(4,0),targets(4,1),targets(4,2);
            r_start_l<< targets(5,0),targets(5,1),targets(5,2);

            left_hand hand_l;

            V_x_l<<0,INFINITY,0;
            V_y_l<<0,INFINITY,0;
            V_z_l<<0,INFINITY,0;
            A_x_l<<0,INFINITY,0;
            A_y_l<<0,INFINITY,0;
            A_z_l<<0,INFINITY,0;
            
            
            while (time_r<total)
            {
            if (scenario=="fixed"){

            }
            else {
            if (time_r<t_r(2)){
                // define minJerk elements to calculate end effector velocity
                P_x_l<< r_start_l(0),r_middle_l(0),r_target_l(0);
                P_y_l<< r_start_l(1),r_middle_l(1),r_target_l(1);
                P_z_l<< r_start_l(2),r_middle_l(2),r_target_l(2);
                
                X_coef_l=coef_generator.Coefficient(t_r,P_x_l,V_x_l,A_x_l); 
                Y_coef_l=coef_generator.Coefficient(t_r,P_y_l,V_y_l,A_y_l);
                Z_coef_l=coef_generator.Coefficient(t_r,P_z_l,V_z_l,A_z_l);

                if(time_r<t_r(1)&& time_r>=t_r(0))
                {
                    P_l<<   coef_generator.GetAccVelPos(X_coef_l.row(0),time_r,t_r(0),5)(0,0),
                            coef_generator.GetAccVelPos(Y_coef_l.row(0),time_r,t_r(0),5)(0,0),
                            coef_generator.GetAccVelPos(Z_coef_l.row(0),time_r,t_r(0),5)(0,0);
                    V_l<<   coef_generator.GetAccVelPos(X_coef_l.row(0),time_r,t_r(0),5)(0,1),
                            coef_generator.GetAccVelPos(Y_coef_l.row(0),time_r,t_r(0),5)(0,1),
                            coef_generator.GetAccVelPos(Z_coef_l.row(0),time_r,t_r(0),5)(0,1);

                    hand_l.update_left_hand(q_la,V_l,r_target_l,R_target_l);
                    r_left_palm=hand_l.r_left_palm;
                    hand_l.doQP(q_la);
                    q_la=hand_l.q_next;
                    d_l=hand_l.dist;
                    theta_l=hand_l.theta;
                    sai_l=hand_l.sai;
                    phi_l=hand_l.phi;

                }

                else if(time_r< t_r(2) && time_r>=t_r(1))
                {
                    P_l<<   coef_generator.GetAccVelPos(X_coef_l.row(1),time_r,t_r(1),5)(0,0),
                            coef_generator.GetAccVelPos(Y_coef_l.row(1),time_r,t_r(1),5)(0,0),
                            coef_generator.GetAccVelPos(Z_coef_l.row(1),time_r,t_r(1),5)(0,0);
                    V_l<<   coef_generator.GetAccVelPos(X_coef_l.row(1),time_r,t_r(1),5)(0,1),
                            coef_generator.GetAccVelPos(Y_coef_l.row(1),time_r,t_r(1),5)(0,1),
                            coef_generator.GetAccVelPos(Z_coef_l.row(1),time_r,t_r(1),5)(0,1);

                        hand_l.update_left_hand(q_la,V_l,r_target_l,R_target_l);
                        r_left_palm=hand_l.r_left_palm;
                        hand_l.doQP(q_la);
                        q_la=hand_l.q_next;
                        d_l=hand_l.dist;
                        theta_l=hand_l.theta;
                        sai_l=hand_l.sai;
                        phi_l=hand_l.phi;
                }
            }
            else {
                    if (scenario=="byebye"){
                    q_la(2)=q_la(2)+0.5*M_PI/180*cos((time_r-t_r(2))*(2*M_PI)); //byebye
                    }
                    else if (scenario=="shakeHands") {
                    q_la(3)=q_la(3)-0.125*M_PI/180*cos((time_r-t_r(2))*(M_PI)); // shakeHands
                    }
                }
            }
            q_end = q_la - q_init_l;
            qref_l.block(0,count+(sum_l - total)/T,7,1)=q_end;

            count++;
            time_r=(count)*T;

            };

            r_midpoint_l = r_left_palm;
            return r_midpoint_l;
            
        };

        VectorXd reach_target_R(MatrixXd targets, string scenario, int M){
            qref_r.resize(7, M);
            r_midpoint_r.resize(3);
            P_x_r.resize(1,3); V_x_r.resize(1,3); A_x_r.resize(1,3);
            P_y_r.resize(1,3); V_y_r.resize(1,3); A_y_r.resize(1,3);
            P_z_r.resize(1,3); V_z_r.resize(1,3); A_z_r.resize(1,3);
            P_r.resize(3); V_r.resize(3);
            // define time parameters
            int count = 0;
            time_r=count*T; 
            MatrixXd t_r(1,3);
            t_r<<0,2,4;
            double total;
            if (scenario=="byebye" || scenario=="shakeHands"){
                total = t_r(2) + 4;
                sum_r+=total;
            }
            else {
                total = t_r(2);
                sum_r+=total;
            }

            // set target values
            r_middle_r<<targets(0,0),targets(0,1),targets(0,2); //shakehands
            r_target_r<<targets(1,0),targets(1,1),targets(1,2);
            R_target_r<<targets(2,0),targets(2,1),targets(2,2),
                        targets(3,0),targets(3,1),targets(3,2),
                        targets(4,0),targets(4,1),targets(4,2);
            r_start_r<< targets(5,0),targets(5,1),targets(5,2);

            right_hand hand_r;

            V_x_r<<0,INFINITY,0;
            V_y_r<<0,INFINITY,0;
            V_z_r<<0,INFINITY,0;
            A_x_r<<0,INFINITY,0;
            A_y_r<<0,INFINITY,0;
            A_z_r<<0,INFINITY,0;
            
            // cout<<targets<<endl;
            while (time_r<total)
            {
            if (scenario=="fixed"){

            }
            else {
            if (time_r<t_r(2)){
                // define minJerk elements to calculate end effector velocity
                P_x_r<< r_start_r(0),r_middle_r(0),r_target_r(0);
                P_y_r<< r_start_r(1),r_middle_r(1),r_target_r(1);
                P_z_r<< r_start_r(2),r_middle_r(2),r_target_r(2);
                
                X_coef_r=coef_generator.Coefficient(t_r,P_x_r,V_x_r,A_x_r); 
                Y_coef_r=coef_generator.Coefficient(t_r,P_y_r,V_y_r,A_y_r);
                Z_coef_r=coef_generator.Coefficient(t_r,P_z_r,V_z_r,A_z_r);

                if(time_r<t_r(1)&& time_r>=t_r(0))
                {
                    P_r<<   coef_generator.GetAccVelPos(X_coef_r.row(0),time_r,t_r(0),5)(0,0),
                            coef_generator.GetAccVelPos(Y_coef_r.row(0),time_r,t_r(0),5)(0,0),
                            coef_generator.GetAccVelPos(Z_coef_r.row(0),time_r,t_r(0),5)(0,0);
                    V_r<<   coef_generator.GetAccVelPos(X_coef_r.row(0),time_r,t_r(0),5)(0,1),
                            coef_generator.GetAccVelPos(Y_coef_r.row(0),time_r,t_r(0),5)(0,1),
                            coef_generator.GetAccVelPos(Z_coef_r.row(0),time_r,t_r(0),5)(0,1);

                    hand_r.update_right_hand(q_ra,V_r,r_target_r,R_target_r);
                    r_right_palm=hand_r.r_right_palm;
                    hand_r.doQP(q_ra);
                    q_ra=hand_r.q_next;
                    d_r=hand_r.dist;
                    theta_r=hand_r.theta;
                    sai_r=hand_r.sai;
                    phi_r=hand_r.phi;

                }

                else if(time_r< t_r(2) && time_r>=t_r(1))
                {
                    P_r<<   coef_generator.GetAccVelPos(X_coef_r.row(1),time_r,t_r(1),5)(0,0),
                            coef_generator.GetAccVelPos(Y_coef_r.row(1),time_r,t_r(1),5)(0,0),
                            coef_generator.GetAccVelPos(Z_coef_r.row(1),time_r,t_r(1),5)(0,0);
                    V_r<<   coef_generator.GetAccVelPos(X_coef_r.row(1),time_r,t_r(1),5)(0,1),
                            coef_generator.GetAccVelPos(Y_coef_r.row(1),time_r,t_r(1),5)(0,1),
                            coef_generator.GetAccVelPos(Z_coef_r.row(1),time_r,t_r(1),5)(0,1);

                    hand_r.update_right_hand(q_ra,V_r,r_target_r,R_target_r);
                    r_right_palm=hand_r.r_right_palm;
                    hand_r.doQP(q_ra);
                    q_ra=hand_r.q_next;
                    d_r=hand_r.dist;
                    theta_r=hand_r.theta;
                    sai_r=hand_r.sai;
                    phi_r=hand_r.phi;
                }
            }
            else {
                    if (scenario=="byebye"){
                    q_ra(2)=q_ra(2)-0.5*M_PI/180*cos((time_r-t_r(2))*(2*M_PI)); //byebye
                    }
                    else if (scenario=="shakeHands") {
                    q_ra(3)=q_ra(3)-0.125*M_PI/180*cos((time_r-t_r(2))*(M_PI)); // shakeHands
                    }
                }
            }
            q_end = q_ra - q_init_r;
            qref_r.block(0,count+(sum_r - total)/T,7,1)=q_end;

            count++;
            time_r=(count)*T;

            };

            r_midpoint_r = r_right_palm;
            return r_midpoint_r;
            
        };

        bool single(hand_planner_test::move_hand_single::Request  &req, hand_planner_test::move_hand_single::Response &res)
        {         
            ros::Rate rate_(rate);
            M = req.t_total/T;
            ee_pos.resize(3,1);
            q_motor.resize(29,0);
            q_gazebo.resize(29,0);

            int id = 0;
            if (req.mode == "righthand") {
                for (int i=0; i<req.scen_count; i++){
                    MatrixXd result_ = scenario_target_R (req.scenario[i], i, ee_pos, req.ee_ini_pos);
                    ee_pos = reach_target_R(result_, req.scenario[i], M);
            }
            }
            else if (req.mode == "lefthand") {
                for (int i=0; i<req.scen_count; i++){
                    MatrixXd result_ = scenario_target_L (req.scenario[i], i, ee_pos, req.ee_ini_pos);
                    ee_pos = reach_target_L(result_, req.scenario[i], M);
                    cout<<ee_pos<<endl;
                }       
            }
            // ROS
            while(id < M) {
                // gazebo
                if (simulation) {
                    if(req.mode=="righthand"){
                        q_gazebo[15]=qref_r(0,id);  
                        q_gazebo[16]=qref_r(1,id);   
                        q_gazebo[17]=qref_r(2,id);  
                        q_gazebo[18]=qref_r(3,id);
                        hand_func_R.SendGazebo(q_gazebo);
                        cout<<q_gazebo[15]<<','<<q_gazebo[16]<<','<<q_gazebo[17]<<','<<q_gazebo[18]<<','<<q_gazebo[19]<<','<<q_gazebo[20]<<','<<q_gazebo[21]<<endl;
                    }
                    else if(req.mode=="lefthand"){
                        q_gazebo[22]=qref_l(0,id);  
                        q_gazebo[23]=qref_l(1,id);   
                        q_gazebo[24]=qref_l(2,id);  
                        q_gazebo[25]=qref_l(3,id);   
                        hand_func_L.SendGazebo(q_gazebo);
                        // cout<<q_gazebo[22]<<','<<q_gazebo[23]<<','<<q_gazebo[24]<<','<<q_gazebo[25]<<','<<q_gazebo[26]<<','<<q_gazebo[27]<<','<<q_gazebo[28]<<endl;
                    }                    

                    }
                else{
                    if(req.mode=="righthand"){
                        q_motor[12]=0;//int(qref_r(0,id)*encoderResolution[0]*harmonicRatio[0]/M_PI/2); // be samte jelo
                        q_motor[13]=0;//-int(qref_r(1,id)*encoderResolution[0]*harmonicRatio[1]/M_PI/2); // be samte birun
                        q_motor[14]=0;//int(qref_r(2,id)*encoderResolution[1]*harmonicRatio[2]/M_PI/2); // be samte birun
                        q_motor[15]=-int(qref_r(3,id)*encoderResolution[1]*harmonicRatio[3]/M_PI/2);// be samte bala
                        cout<<q_motor[12]<<','<<q_motor[13]<<','<<q_motor[14]<<','<<q_motor[15]<<endl;
                    }
                    else if(req.mode=="lefthand"){
                        q_motor[16]=-int(qref_l(0,id)*encoderResolution[0]*harmonicRatio[0]/M_PI/2);
                        q_motor[17]=-int(qref_l(1,id)*encoderResolution[0]*harmonicRatio[1]/M_PI/2);
                        q_motor[18]=int(qref_l(2,id)*encoderResolution[1]*harmonicRatio[2]/M_PI/2);
                        q_motor[19]=int(qref_l(3,id)*encoderResolution[1]*harmonicRatio[3]/M_PI/2);
                        // cout<<q_motor[16]<<','<<q_motor[17]<<','<<q_motor[18]<<','<<q_motor[19]<<endl;
                    }
                    
                    trajectory_data.data.clear();
                    for(int  i = 0; i < 20; i++)
                    {
                        trajectory_data.data.push_back(q_motor[i]);
                    }
                    trajectory_data_pub.publish(trajectory_data);    
                    ros::spinOnce();
                    rate_.sleep();
                }
            id++;
            }
            sum_l = 0;
            sum_r = 0;
            cout<<ee_pos<<endl;
            if (req.mode=="righthand"){
                next_ini_ee_posR = ee_pos;
            }
            else if(req.mode=="lefthand"){
                next_ini_ee_posL = ee_pos;
            }
            
            res.ee_fnl_pos = req.scenario[req.scen_count-1];

            return true;
        }

        bool both(hand_planner_test::move_hand_both::Request  &req, hand_planner_test::move_hand_both::Response &res){
            
            ros::Rate rate_(rate);
            M = req.t_total/T;
            MatrixXd result_(12,3);
            ee_pos.resize(3,2);
            q_motor.resize(29,0);
            q_gazebo.resize(29,0);
            int id = 0;

            for (int i=0; i<req.scenR_count; i++){
                result_.block(0,0,6,3) = scenario_target_R (req.scenarioR[i], i, ee_pos.block(0,0,3,1), req.ee_ini_posR);
                ee_pos.block(0,0,3,1) = reach_target_R(result_.block(0,0,6,3), req.scenarioR[i], M);
                // cout<<ee_pos.block(0,0,3,1)<<endl;
            }
            for (int i=0; i<req.scenL_count; i++){
                result_.block(6,0,6,3) = scenario_target_L (req.scenarioL[i], i, ee_pos.block(0,1,3,1), req.ee_ini_posL);
                ee_pos.block(0,1,3,1) = reach_target_L(result_.block(6,0,6,3), req.scenarioL[i], M);
                // cout<<ee_pos.block(0,1,3,1)<<endl;
            } 
            while(id < M) {
                if (simulation) {
                    q_gazebo[15]=qref_r(0,id);  
                    q_gazebo[16]=qref_r(1,id);   
                    q_gazebo[17]=qref_r(2,id);  
                    q_gazebo[18]=qref_r(3,id);
                    q_gazebo[22]=qref_l(0,id);  
                    q_gazebo[23]=qref_l(1,id);   
                    q_gazebo[24]=qref_l(2,id);  
                    q_gazebo[25]=qref_l(3,id); 
                    hand_func_L.SendGazebo(q_gazebo);
                    hand_func_R.SendGazebo(q_gazebo); 
                }

                else {
                    q_motor[12]=int(qref_r(0,id)*encoderResolution[0]*harmonicRatio[0]/M_PI/2); // be samte jelo
                    q_motor[13]=-int(qref_r(1,id)*encoderResolution[0]*harmonicRatio[1]/M_PI/2); // be samte birun
                    q_motor[14]=int(qref_r(2,id)*encoderResolution[1]*harmonicRatio[2]/M_PI/2); // be samte birun
                    q_motor[15]=-int(qref_r(3,id)*encoderResolution[1]*harmonicRatio[3]/M_PI/2);// be samte bala
                    q_motor[16]=-int(qref_l(0,id)*encoderResolution[0]*harmonicRatio[0]/M_PI/2);
                    q_motor[17]=-int(qref_l(1,id)*encoderResolution[0]*harmonicRatio[1]/M_PI/2);
                    q_motor[18]=int(qref_l(2,id)*encoderResolution[1]*harmonicRatio[2]/M_PI/2);
                    q_motor[19]=int(qref_l(3,id)*encoderResolution[1]*harmonicRatio[3]/M_PI/2);
                
                    trajectory_data.data.clear();
                    for(int  i = 0; i < 20; i++)
                    {
                        trajectory_data.data.push_back(q_motor[i]);
                    }
                    trajectory_data_pub.publish(trajectory_data);    
                    ros::spinOnce();
                    rate_.sleep();
                }

                id++;
            }
            sum_l = 0;
            sum_r = 0;
            cout<<ee_pos.block(0,0,3,1)<<endl;
            cout<<ee_pos.block(0,1,3,1)<<endl;
            next_ini_ee_posR = ee_pos.block(0,0,3,1);
            next_ini_ee_posL = ee_pos.block(0,1,3,1);
            res.ee_fnl_posR = req.scenarioR[req.scenR_count-1];
            res.ee_fnl_posL = req.scenarioL[req.scenL_count-1];
            return true;
        }

        bool home(hand_planner_test::home_service::Request  &req, hand_planner_test::home_service::Response &res)
        {
            ros::Rate rate_(rate);
            double t_local = 0;
            int count = 0;
            q_motor.resize(20,0);
            q_gazebo.resize(29,0);

            while (count<int(req.T_home/T))
            {
                    if (simulation) {
                            if(req.mode=="righthand"){
                                q_gazebo[15]=qref_r(0,qref_r.cols()-1) + hand_func_R.move2pose(-qref_r(0,qref_r.cols()-1), t_local, 0, req.T_home);  
                                q_gazebo[16]=qref_r(1,qref_r.cols()-1) + hand_func_R.move2pose(-qref_r(1,qref_r.cols()-1), t_local, 0, req.T_home);  
                                q_gazebo[17]=qref_r(2,qref_r.cols()-1) + hand_func_R.move2pose(-qref_r(2,qref_r.cols()-1), t_local, 0, req.T_home);  
                                q_gazebo[18]=qref_r(3,qref_r.cols()-1) + hand_func_R.move2pose(-qref_r(3,qref_r.cols()-1), t_local, 0, req.T_home);
                                hand_func_R.SendGazebo(q_gazebo);
                                cout<<q_gazebo[15]<<','<<q_gazebo[16]<<','<<q_gazebo[17]<<','<<q_gazebo[18]<<','<<q_gazebo[19]<<','<<q_gazebo[20]<<','<<q_gazebo[21]<<endl;
                            }
                            else if(req.mode=="lefthand"){
                                q_gazebo[22]=qref_l(0,qref_r.cols()-1) + hand_func_R.move2pose(-qref_l(0,qref_r.cols()-1), t_local, 0, req.T_home);  
                                q_gazebo[23]=qref_l(1,qref_r.cols()-1) + hand_func_R.move2pose(-qref_l(1,qref_r.cols()-1), t_local, 0, req.T_home);  
                                q_gazebo[24]=qref_l(2,qref_r.cols()-1) + hand_func_R.move2pose(-qref_l(2,qref_r.cols()-1), t_local, 0, req.T_home);  
                                q_gazebo[25]=qref_l(3,qref_r.cols()-1) + hand_func_R.move2pose(-qref_l(3,qref_r.cols()-1), t_local, 0, req.T_home);  
                                hand_func_L.SendGazebo(q_gazebo);
                                // cout<<q_gazebo[22]<<','<<q_gazebo[23]<<','<<q_gazebo[24]<<','<<q_gazebo[25]<<','<<q_gazebo[26]<<','<<q_gazebo[27]<<','<<q_gazebo[28]<<endl;
                            }                    
                    }
                    else {
                        if (req.mode == "righthand"){
                            for (int i=12; i<16; i++){
                                q_motor[i] = int(QcArr[i] + hand_func_R.move2pose(-QcArr[i], t_local, 0, req.T_home));        
                            } 
                            }
                        else if (req.mode == "lefthand"){
                            for (int i=16; i<20; i++){
                                q_motor[i] = int(QcArr[i] + hand_func_R.move2pose(-QcArr[i], t_local, 0, req.T_home));
                            }
                            }
                        trajectory_data.data.clear();

                        for(int  i = 0; i < 20; i++)
                        {
                            trajectory_data.data.push_back(q_motor[i]);
                        }
                        trajectory_data_pub.publish(trajectory_data); 
                        ros::spinOnce();
                        rate_.sleep();
                        cout<<q_motor[0]<<", "<<q_motor[1]<<", "<<q_motor[2]<<", "<<q_motor[3]<<", "<<q_motor[4]<<", "<<q_motor[5]<<", "<<q_motor[6]<<", "<<q_motor[7]<<", "<<q_motor[8]<<", "<<q_motor[9]<<", "<<q_motor[10]<<", "<<q_motor[11]<<", "<<q_motor[12]<<", "<<q_motor[13]<<", "<<q_motor[14]<<", "<<q_motor[15]<<", "<<q_motor[16]<<", "<<q_motor[17]<<", "<<q_motor[18]<<", "<<q_motor[19]<<endl;
                    }
                    t_local+=T;
                    count = count + 1;   
                }
                res.home = "Done";

                return true; 
        }

        double norm(VectorXd V){
            double s=0;
            for (int i = 0; i < V.rows(); ++i) {
                s+=V(i)*V(i);
            }
            return sqrt(s);
        }

        bool grip_Online(hand_planner_test::gripOnline::Request  &req, hand_planner_test::gripOnline::Response &res){
            ros::Rate rate_(rate);
            ofstream fw;
            fw.open("/home/surenav/DynCont/Code/WalkTest/src/hand_planner_test/src/test_surena4_ros/gripOnline.txt", std::ofstream::out);
            q_motor.resize(29,0);
            q_gazebo.resize(29,0);
            q_ra.resize(7);
            q_init_r.resize(7);             
            camera_target.resize(3);
            r_right_palm.resize(3);
            R_target_r.resize(3,3);
            V_r.resize(3);
            T_EEtobase.resize(4, 4);
            camera.resize(3);
            camera << 0.1248, 0, 0.06746;
            q_ra<<-12.3*M_PI/180,-5*M_PI/180,38*M_PI/180,-4*M_PI/180,0,0,0; // initial condition
            q_init_r = q_ra;

            // define right_hand objs
            right_hand hand_r;
            right_hand hand0_r(q_ra,camera_target,R_target_r,0,0);
            r_right_palm=hand0_r.r_right_palm;

            while ((abs(r_right_palm(0) - camera_target(0))>0.025 || abs(r_right_palm(1) - camera_target(1))>0.025 || abs(r_right_palm(2) - camera_target(2))>0.025) || t_grip<=60) { //t_grip<=40 || 
            camera_target<< X, Y, Z;

            R_target_r = hand_func_R.rot(2,-65*M_PI/180,3);
            // R_target_r = hand_func_R.rot(3,90*M_PI/180,3)*hand_func_R.rot(1,-180*M_PI/180,3);
            V_r << 0.8*(camera_target - r_right_palm);
            if ((abs(r_right_palm(0) - camera_target(0))>0.025 || abs(r_right_palm(1) - camera_target(1))>0.025 || abs(r_right_palm(2) - camera_target(2))>0.025)) {
                cout<<"palmX: "<<r_right_palm(0)<<"/ X: "<<X<<"  palmY: "<<r_right_palm(1)<<"/ Y: "<<Y<<"  palmZ: "<<r_right_palm(2)<<"/ Z: "<<Z<<"/ baseX: "<<T_EEtobase(0)<<"/ baseY: "<<T_EEtobase(1)<<"/ baseZ: "<<T_EEtobase(2)<<endl;
                cout<<"--"<<endl;
            }
            else {
                cout<< "TARGET RICHED"<<endl;
            }
            
            
            hand_r.update_right_hand(q_ra,V_r,camera_target,R_target_r);
            r_right_palm=hand_r.r_right_palm;
            hand_r.doQP(q_ra);
            q_ra=hand_r.q_next;
            d_r=hand_r.dist;
            theta_r=hand_r.theta;
            sai_r=hand_r.sai;
            phi_r=hand_r.phi;

            if (abs(Y) > 0.02) {

                if (abs(h_yaw) < abs(atan2(Y,X))) {
                    h_yaw += Ky*atan2(Y,X);
                }
                else {
                    h_yaw = -atan2(Y,X);
                }
                if (abs(h_yaw)*180/M_PI>90){
                    if (h_yaw > 0) {
                        h_yaw = 90*M_PI/180;
                    }
                    else{
                        h_yaw = -90*M_PI/180;
                        }
                }
                // cout<<"h_yaw: "<<h_yaw<<endl;
            }        
            if (abs(Z) > 0.03) {
                
                if (abs(h_pitch) < abs(atan2(Z,sqrt(pow(Y,2)+pow(X,2))))) {
                    h_pitch += Kp*atan2(Z,sqrt(pow(Y,2)+pow(X,2)));
                }
                else {
                    h_pitch = atan2(Z,sqrt(pow(Y,2)+pow(X,2)));
                }
                if (abs(h_pitch)*180/M_PI>25){
                    if (h_pitch > 0) {
                        h_pitch = 25*M_PI/180;
                    }
                    else{
                        h_pitch = -25*M_PI/180;
                        }
                }
                // cout<<"h_pitch: "<<h_pitch<<endl; 
            }
            
            T_EEtobase << hand_r.ObjToNeck(camera, h_pitch, h_roll, h_yaw, PtoR, YtoP);

            if (simulation) {
                q_gazebo[15]=q_ra(0)-q_init_r(0);  
                q_gazebo[16]=q_ra(1)-q_init_r(1); 
                q_gazebo[17]=q_ra(2)-q_init_r(2);  
                q_gazebo[18]=q_ra(3)-q_init_r(3);
                hand_func_R.SendGazebo(q_gazebo);
                cout<<q_gazebo[15]<<','<<q_gazebo[16]<<','<<q_gazebo[17]<<','<<q_gazebo[18]<<endl;
                }
            else{

                q_motor[12]=int((q_ra(0)-q_init_r(0))*encoderResolution[0]*harmonicRatio[0]/M_PI/2); // be samte jelo
                q_motor[13]=-int((q_ra(1)-q_init_r(1))*encoderResolution[0]*harmonicRatio[1]/M_PI/2); // be samte birun
                q_motor[14]=int((q_ra(2)-q_init_r(2))*encoderResolution[1]*harmonicRatio[2]/M_PI/2); // be samte birun
                q_motor[15]=-int((q_ra(3)-q_init_r(3))*encoderResolution[1]*harmonicRatio[3]/M_PI/2);// be samte bala
                q_motor[21] = int(pitch_command_range[0] + (pitch_command_range[1] - pitch_command_range[0]) * ((-(h_pitch*180/M_PI) - pitch_range[0]) / (pitch_range[1] - pitch_range[0])));
                q_motor[20] = int(roll_command_range[0] + (roll_command_range[1] - roll_command_range[0]) * ((-(h_roll*180/M_PI) - (roll_range[0])) / (roll_range[1] - (roll_range[0]))));
                q_motor[22] = int(yaw_command_range[0] + (yaw_command_range[1] - yaw_command_range[0]) * ((-(h_yaw*180/M_PI) - yaw_range[0]) / (yaw_range[1] - yaw_range[0])));

                // cout<<q_motor[12]<<','<<q_motor[13]<<','<<q_motor[14]<<','<<q_motor[15]<<endl;
                
                trajectory_data.data.clear();
                for(int  i = 0; i < 23; i++)
                {
                    trajectory_data.data.push_back(q_motor[i]);
                }
                trajectory_data_pub.publish(trajectory_data);    
                }

                ros::spinOnce();
                rate_.sleep();

            for (int i = 0; i < q_ra.size(); i++) {
                fw << q_ra(i)-q_init_r(i) << "\n";
                }
            
            t_grip += 0.005;
            }

            fw.close();
            cout<<r_right_palm<<endl;
            cout<<t_grip<<endl;
            t_grip = 0;
            res.finish = "end";

            return true;
        }




    private:
        std_msgs::Int32MultiArray trajectory_data;
        ros::Publisher  trajectory_data_pub;
        ros::Subscriber  camera_data_sub;
        ros::ServiceServer move_hand_single_service;
        ros::ServiceServer move_hand_both_service;
        ros::ServiceServer grip_Online_seivice;
        ros::ServiceServer home_service;
        ros::Subscriber  JointQcSub;

        double dist;
        double y, z;
        double a = 0.5; // 0.55
        double b = 0.4; // 0.385
        double X0 = 0.5;
        double X = 1;
        double Y, Z, Y0, Z0, L0;
        int L = 640; int W = 480;
        double tempX = 1;
        double tempY, tempZ;

        MinimumJerkInterpolation coef_generator;

        MatrixXd X_coef_r;    MatrixXd X_coef_l;        
        MatrixXd Y_coef_r;    MatrixXd Y_coef_l;         
        MatrixXd Z_coef_r;    MatrixXd Z_coef_l;  
        VectorXd P_r; VectorXd V_r;
        VectorXd P_l; VectorXd V_l;

        VectorXd r_start_r;       VectorXd r_start_l;
        VectorXd r_middle_r;      VectorXd r_middle_l;      
        VectorXd r_target_r;      VectorXd r_target_l; 
        MatrixXd R_target_r;      MatrixXd R_target_l;
        VectorXd r_midpoint_r;    VectorXd r_midpoint_l; 
        MatrixXd r_midpoint_b;    VectorXd camera_target;
        VectorXd r_right_palm;    VectorXd r_left_palm;

        MatrixXd P_x_r;    MatrixXd P_x_l;          
        MatrixXd V_x_r;    MatrixXd V_x_l;        
        MatrixXd A_x_r;    MatrixXd A_x_l;  
        MatrixXd P_y_r;    MatrixXd P_y_l;    
        MatrixXd V_y_r;    MatrixXd V_y_l;       
        MatrixXd A_y_r;    MatrixXd A_y_l;       
        MatrixXd P_z_r;    MatrixXd P_z_l;       
        MatrixXd V_z_r;    MatrixXd V_z_l;      
        MatrixXd A_z_r;    MatrixXd A_z_l;

        double d0_r;            double d0_l; 
        double d_r ;            double d_l;
        double d_des_r;         double d_des_l;    
        double theta_r;         double theta_l;      
        double theta_target_r;  double theta_target_l;  
        double sai_r;           double sai_l;    
        double sai_target_r;    double sai_target_l; 
        double phi_r;           double phi_l;           
        double phi_target_r;    double phi_target_l;   
        double v0_r=0;          double v0_l=0;
        double v_target_r =.4;  double v_target_l =.4;

        // define joint variables
        VectorXd q_ra;   VectorXd q_la;
        VectorXd q_init_r; VectorXd q_init_l;
        MatrixXd qref;
        MatrixXd ee_pos;
        vector<double> q_motor;
        vector<double> q_gazebo;

        double sum_r = 0;
        double sum_l = 0;
        double T = 0.005; 
        double Kp = 0.01;
        double Ky = -0.01;

        VectorXd q_end;
        MatrixXd qref_r;
        MatrixXd qref_l;
        double time_r;
        left_hand hand_func_L;
        right_hand hand_func_R;
        double total;
        double t_grip = 0;
        MatrixXd next_ini_ee_posR;
        MatrixXd next_ini_ee_posL;
        int M;
        int rate = 200;
        bool simulation = false;
        int encoderResolution[2] = {4096*4, 2048*4};
        int harmonicRatio[4] = {100, 100, 100, 400};
        vector<int> pitch_range = {-30, 30};
        vector<int> roll_range = {-50, 50};
        vector<int> yaw_range = {-90, 90};
        vector<int> pitch_command_range = {180, 110};
        vector<int> roll_command_range = {100, 190};
        vector<int> yaw_command_range = {90, 210};
        double h_pitch = 0;
        double h_roll = 0;
        double h_yaw = 0;
        double PtoR = 0.0825;
        double YtoP = 0.06025;
        MatrixXd T_EEtobase;
        VectorXd camera;
        int QcArr[20];
};



    int main(int argc, char **argv){

        ros::init(argc, argv, "handnode");
        ros::NodeHandle n;
        test_forgotten_srv wt(&n);
        ROS_INFO("Ready to call service...");
        ros::spin();
        return 0 ;
    }

//g++ 