using System;
using System.Collections.Generic;
using System.Collections.Specialized;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;

namespace DualRobotLib
{
    internal class SceneX
    {
        // Scene1A Params
        public double cr15_scene1a_R;
        public double cr15_scene1a_motion1_arc;
        public double cr15_scene1a_motion1_step_angle;
        public double cr15_scene1a_motion2_arc;
        public double cr15_scene1a_motion2_step_angle;
        public double cr7_scene1a_r;
        public double cr7_scene1a_arc;
        public double cr7_scene1a_step_angle;
        public double motor_scene1a_arc;
        public double motor_scene1a_step_angle;
        public double lift_table_scene1a_height;
        public double lift_table_scene1a_align_error;

        // Scene1B Params
        public double tcp_distance;
        public double cr15_arc, cr15_step_angle;
        public double cr7_motion2_arc, cr7_motion2_step_angle, cr7_motion3_arc, cr7_motion3_step_angle;

        // Scene1C Params
        public  double d_separation;
        public  double span_width;  // horizontal
        public  double noOfWStep;
        public  double span_height; // vertical
        public  double noOfHStep;
        public  double tilt_angle;

        // Scene2 Params
        public double cr15_scene2_R;
        public double cr15_scene2_motion1_arc;
        public double cr15_scene2_motion1_step_angle;
        public double cr15_scene2_motion2_arc;
        public double cr15_scene2_motion2_step_angle;
        public double cr7_scene2_r;
        public double cr7_scene2_arc;
        public double cr7_scene2_step_angle;
        public double motor_scene2_angle;
        public double lift_table_scene2_height;
        public double lift_table_scene2_align_error;
        public double stage34_fixture_height;
        public double antenna_height;

        // Scene3 Params
        public double s3_cr15_R;
        public double s3_span_width;  // horizontal
        public double s3_noOfWStep;
        public double s3_span_height; // vertical
        public double s3_noOfHStep;
        public double s3_tilt_angle;
        public double s3_cr7_r;
        public double s3_cr7_arc;
        public double s3_cr15_scene2_motion1_arc;
        public double s3_cr15_scene2_motion1_step_angle;
        public double s3_cr15_scene2_motion2_arc;
        public double s3_cr15_scene2_motion2_step_angle;
        public double s3_cr7_scene2_step_angle;
        public double s3_motor_scene2_angle;
        public double s3_lift_table_scene2_height;
        public double s3_lift_table_scene2_align_error;
        public double s3_stage34_fixture_height;
        public double s3_antenna_height;

        // Scene4 Params
        public double s4_d_separation; // 0
        public double s4_span_lenght;  // x
        public double s4_noOfLStep;    
        public double s4_span_width;   // y
        public double s4_noOfWStep;
        public double s4_span_height;  // z
        public double s4_noOfHStep;

        public double cr7_scene2_r_max = 100;

        private float[] init_pos_cr7, init_pos_cr15;

        private Transformation tf = new Transformation();

        public void ParamsInit(SceneName scene_name, double[] param)
        {
            switch (scene_name)
            {
                case SceneName.Scene1A:
                {
                    cr15_scene1a_R = param[0];
                    cr15_scene1a_motion1_arc = param[1];
                    cr15_scene1a_motion1_step_angle = param[2];
                    motor_scene1a_arc = param[3];
                    motor_scene1a_step_angle = param[4];
                    lift_table_scene1a_height = param[5];
                    lift_table_scene1a_align_error = param[6];
                    stage34_fixture_height = param[7];
                    antenna_height = param[8];

                    break;
                }
                case SceneName.Scene1A_Sim:
                {
                    cr15_scene1a_R = param[0];
                    cr15_scene1a_motion1_arc = param[1];
                    cr15_scene1a_motion1_step_angle = param[2];
                    motor_scene1a_arc = param[3];
                    motor_scene1a_step_angle = param[4];
                    lift_table_scene1a_height = param[5];
                    lift_table_scene1a_align_error = param[6];

                    break;
                }
                case SceneName.Scene1B:
                {
                    tcp_distance           = param[0];
                    cr15_arc               = param[1];
                    cr15_step_angle        = param[2];
                    cr7_motion2_arc        = param[3];
                    cr7_motion2_step_angle = param[4];
                    cr7_motion3_arc        = param[5];
                    cr7_motion3_step_angle = param[6];

                    break;
                }
                case SceneName.Scene1C:
                {
                    d_separation      = tcp_distance   = param[0];
                    span_width        = param[1];
                    span_height       = param[2];
                    noOfWStep         = param[3];
                    noOfHStep         = param[4];
                    tilt_angle        = param[5];

                    break;
                }
                case SceneName.Scene2:
                {
                    cr15_scene2_R   = param[0];
                    cr15_scene2_motion1_arc = param[1];
                    cr15_scene2_motion1_step_angle = param[2];
                    cr15_scene2_motion2_arc = param[3];
                    cr15_scene2_motion2_step_angle = param[4];
                    cr7_scene2_r   = param[5];
                    cr7_scene2_arc = param[6];
                    motor_scene2_angle  = param[7];
                    lift_table_scene2_height = param[8];
                    lift_table_scene2_align_error = param[9];
                    stage34_fixture_height = param[10];

                    break;
                }
                case SceneName.Scene2_Sim:
                {
                    cr15_scene2_R = param[0];
                    cr15_scene2_motion1_arc = param[1];
                    cr15_scene2_motion1_step_angle = param[2];
                    cr15_scene2_motion2_arc = param[3];
                    cr15_scene2_motion2_step_angle = param[4];
                    cr7_scene2_r = param[5];
                    cr7_scene2_arc = param[6];
                    motor_scene2_angle = param[7];
                    lift_table_scene2_height = param[8];
                    lift_table_scene2_align_error = param[9];
                    stage34_fixture_height = param[10];

                    break;
                }
                case SceneName.Scene3_Sim:
                {
                    s3_cr15_R = param[0];
                    s3_span_width = param[1];
                    s3_span_height = param[2];
                    s3_noOfWStep = param[3];
                    s3_noOfHStep = param[4];
                    s3_cr7_r = param[5];
                    s3_cr7_arc = param[6];
                    motor_scene2_angle = param[7];
                    lift_table_scene2_height = param[8];
                    lift_table_scene2_align_error = param[9];
                    stage34_fixture_height = param[10];

                    break;
                }
                case SceneName.Scene3:
                {
                    s3_cr15_R = param[0];
                    s3_span_width = param[1];
                    s3_span_height = param[2];
                    s3_noOfWStep = param[3];
                    s3_noOfHStep = param[4];
                    cr7_scene2_r = param[5];
                    cr7_scene2_arc = param[6];
                    motor_scene2_angle = param[7];
                    lift_table_scene2_height = param[8];
                    lift_table_scene2_align_error = param[9];
                    stage34_fixture_height = param[10];

                    break;
                }
                case SceneName.Scene4_Sim:
                {
                    s4_d_separation = param[0];
                    s4_span_lenght = param[1];
                    s4_noOfLStep = param[2];
                    s4_span_width = param[3];
                    s4_noOfWStep = param[4];
                    s4_span_height = param[5];
                    s4_noOfHStep = param[6];

                    break;
                }
                case SceneName.Scene4:
                {
                    s4_d_separation = param[0];
                    s4_span_lenght = param[1];
                    s4_noOfLStep = param[2];
                    s4_span_width = param[3];
                    s4_noOfWStep = param[4];
                    s4_span_height = param[5];
                    s4_noOfHStep = param[6];

                    break;
                }
            }
        }

        // Scene1A: Get Cr15 Init Position
        public void GetScene1AInitPositionXyzwpr(ref Calibration cal)
        {
            // Cr15 Init Position 

            var T_Cr15_R = cal.Set_T_Cr15_R(ref cr15_scene1a_R);

            // var T_Cr15_Scene2_temp =  cal.T_Cr7_PlateCenter * T_Cr15_R * T_diff;
            var T_Cr15_Scene2_temp = cal.T_Cr7_PlateCenter * T_Cr15_R;

            var T_rotate2 = cal.Set_T_rotate2();

            cal.T_Cr15_Scene2 = cal.T_Cr15_Cr7 * T_Cr15_Scene2_temp * T_rotate2;

            init_pos_cr15 = tf.T2pos(cal.T_Cr15_Scene2);
        }

        public void GetScene1ASimCr15InitPositionXyzwpr(ref Calibration cal)
        {
            // 3. Cr15 Init Position 

            var T_Cr15_R = cal.Set_T_Cr15_R(ref cr15_scene1a_R); // R = 160

            var T_Cr15_Scene2_temp = cal.T_Cr7_PlateCenter * T_Cr15_R; // T recorded in Cr7

            var T_rotate2 = cal.Set_T_rotate2();

            cal.T_Cr15_Scene2 = cal.T_Cr15_Cr7 * T_Cr15_Scene2_temp * T_rotate2; // T recorded in Cr15

            init_pos_cr15 = tf.T2pos(cal.T_Cr15_Scene2);
        }

        // Scene1B & Scene1C: Get Cr15 Init Position
        public void GetCr15InitPositionXyzwpr(ref DualRobot drCR7, ref DualRobot drCR15, ref Calibration cal, SceneName scene_name)
        {
            // drCR7.SwitchToRobotBase();

            // 1. get Cr7 Init Position Xyzwpr
            switch (scene_name)
            {
                case SceneName.Scene1B:
                {
                    init_pos_cr7 = drCR7.GetPosXyzwpr(Position.InitPos_Scene1B);
                    break;
                }
                case SceneName.Scene1C:
                {
                    init_pos_cr7 = drCR7.GetPosXyzwpr(Position.InitPos_Scene1C);
                    break;
                }
            }

            // 2. get T Cr7 Init Position
            var T_Cr7_Scene1B = tf.pos2T(ref init_pos_cr7);

            // 3. set Tr
            var Tr = cal.SetTr(ref this.tcp_distance);

            // 4. get T Cr15 Init Position
            var T_Cr15_Pos_temp = cal.T_Cr15_Cr7 * T_Cr7_Scene1B;
            cal.T_Cr15_Pos = T_Cr15_Pos_temp * Tr;

            // 5. Cr15 Init Position
            init_pos_cr15 = tf.T2pos(cal.T_Cr15_Pos);

            // cout
            // var init_pos_cr7_j = drCR7.GetPosJoint(Position.InitPos_Scene1B);
            // cout_pos("init_pos_cr7_j", init_pos_cr7_j);
            cout_pos("init_pos_cr7", init_pos_cr7);
            cout_pos("init_pos_cr15", init_pos_cr15);
            // cout_TMat("T_Cr15_Cr7", cal.T_Cr15_Cr7);
            // cout_TMat("Tr", Tr);
            // cout_TMat("T_Cr7_Scene1B", T_Cr7_Scene1B);
        }

        // Scene2_Sim
        public void GetScene2SimInitPositionXyzwpr(ref Calibration cal)
        {
            // 1. get T_Cr7_PlateCenter

            // 2. Cr7 Init Position
            var T_Cr7_r = cal.Set_T_Cr7_r(ref this.cr7_scene2_r, ref this.cr7_scene2_arc); // cr7_scene2_r

            var T_rotate = cal.Set_T_rotate();

            var T_rot_y = cal.Set_T_rot_y(this.cr7_scene2_arc);

            // cal.T_Cr7_Scene2 = T_Cr7_r * cal.T_Cr7_PlateCenter * T_rotate;
            //
            cal.T_Cr7_Scene2 =  cal.T_Cr7_PlateCenter * T_Cr7_r * T_rotate  * T_rot_y;

            init_pos_cr7 = tf.T2pos(cal.T_Cr7_Scene2);

            // 3. Cr15 Init Position 

            //ori - init pos 1217.266,102.5263,-42.7094,-179.898,-0.9529985,177.7163
            var T_Cr15_R = cal.Set_T_Cr15_R(ref cr15_scene2_R);
            
            var T_Cr15_Scene2_temp =  cal.T_Cr7_PlateCenter * T_Cr15_R;
            
            var T_rotate2 = cal.Set_T_rotate2();
            
            cal.T_Cr15_Scene2 = cal.T_Cr15_Cr7 * T_Cr15_Scene2_temp * T_rotate2;

            init_pos_cr15 = tf.T2pos(cal.T_Cr15_Scene2);

            Console.WriteLine("init pos = " + String.Join(",", init_pos_cr15));
        }

        // Scene2
        public void GetScene2InitPositionXyzwpr(ref Calibration cal)
        {
            // 1. get T_Cr7_PlateCenter

            // 2. Cr7 Init Position
            var T_Cr7_r = cal.Set_T_Cr7_r(ref this.cr7_scene2_r, ref this.cr7_scene2_arc); // cr7_scene2_r

            var T_rotate = cal.Set_T_rotate();

            var T_rot_y = cal.Set_T_rot_y(this.cr7_scene2_arc);

            // cal.T_Cr7_Scene2 = T_Cr7_r * cal.T_Cr7_PlateCenter * T_rotate;
            //
            cal.T_Cr7_Scene2 = cal.T_Cr7_PlateCenter * T_Cr7_r * T_rotate * T_rot_y;

            init_pos_cr7 = tf.T2pos(cal.T_Cr7_Scene2);

            // 3. Cr15 Init Position 

            var T_Cr15_R = cal.Set_T_Cr15_R(ref cr15_scene2_R);

            var T_Cr15_Scene2_temp =  cal.T_Cr7_PlateCenter * T_Cr15_R;

            var T_rotate2 = cal.Set_T_rotate2();

            cal.T_Cr15_Scene2 = cal.T_Cr15_Cr7 * T_Cr15_Scene2_temp * T_rotate2;

            init_pos_cr15 = tf.T2pos(cal.T_Cr15_Scene2);
        }

        // Scene3_Sim
        public void GetScene3SimInitPositionXyzwpr(ref Calibration cal)
        {
            // 1. get T_Cr7_PlateCenter

            // 2. Cr7 Init Position
            var T_Cr7_r = cal.Set_T_Cr7_r(ref this.s3_cr7_r, ref this.s3_cr7_arc);

            var T_rotate = cal.Set_T_rotate4();

            var T_rot_y = cal.Set_T_rot_y(this.s3_cr7_arc);

            // cal.T_Cr7_Scene2 = T_Cr7_r * cal.T_Cr7_PlateCenter * T_rotate;
            //
            cal.T_Cr7_Scene2 = cal.T_Cr7_PlateCenter * T_Cr7_r * T_rotate * T_rot_y;

            init_pos_cr7 = tf.T2pos(cal.T_Cr7_Scene2);

            // 3. Cr15 Init Position 

            //ori - init pos 1217.266,102.5263,-42.7094,-179.898,-0.9529985,177.7163
            var T_Cr15_R = cal.Set_T_Cr15_R(ref s3_cr15_R);

            var T_Cr15_Scene2_temp = cal.T_Cr7_PlateCenter * T_Cr15_R;

            var T_rotate2 = cal.Set_T_rotate2();

            cal.T_Cr15_Scene2 = cal.T_Cr15_Cr7 * T_Cr15_Scene2_temp * T_rotate2;

            init_pos_cr15 = tf.T2pos(cal.T_Cr15_Scene2);

            Console.WriteLine("init pos = " + String.Join(",", init_pos_cr15));
        }

        // Overall
        public void GetInitPositionXyzwpr(Env env, SceneName scene_name, ref Calibration cal)
        {
            switch (env)
            {
                case Env.Simulation:
                {
                    switch (scene_name)
                    {
                        case SceneName.Scene4_Sim:
                        {
                            // 1. get T_Cr7_PlateCenter

                            //// 2. Cr7 Init Position
                            //var T_Cr7_r = cal.Set_T_Cr7_r(ref this.s3_cr7_r, ref this.s3_cr7_arc);

                            //var T_rotate = cal.Set_T_rotate4();

                            //var T_rot_y = cal.Set_T_rot_y(this.s3_cr7_arc);

                            //// cal.T_Cr7_Scene2 = T_Cr7_r * cal.T_Cr7_PlateCenter * T_rotate;
                            ////
                            //cal.T_Cr7_Scene2 = cal.T_Cr7_PlateCenter * T_Cr7_r * T_rotate * T_rot_y;

                            //init_pos_cr7 = tf.T2pos(cal.T_Cr7_Scene2);

                            // 3. Cr15 Init Position 
                            var T_Cr15_R = cal.Set_T_Cr15_R(ref this.s4_d_separation);

                            var T_Cr15_temp = cal.T_Cr7_PlateCenter * T_Cr15_R;

                            var T_rotate2 = cal.Set_T_rotate2();

                            cal.T_Cr15_Scene2 = cal.T_Cr15_Cr7 * T_Cr15_temp * T_rotate2;

                            init_pos_cr15 = tf.T2pos(cal.T_Cr15_Scene2);

                            Console.WriteLine("init pos = " + string.Join(",", init_pos_cr15));
                            break;
                        }

                    }

                    break;
                }

                case Env.Real:
                {
                    switch (scene_name)
                    {
                        case SceneName.Scene4:
                        {
                            // 3. Cr15 Init Position 
                            var T_Cr15_R = cal.Set_T_Cr15_R(ref this.s4_d_separation);

                            var T_Cr15_temp = cal.T_Cr7_PlateCenter * T_Cr15_R;

                            var T_rotate2 = cal.Set_T_rotate2();

                            cal.T_Cr15_Scene2 = cal.T_Cr15_Cr7 * T_Cr15_temp * T_rotate2;

                            init_pos_cr15 = tf.T2pos(cal.T_Cr15_Scene2);

                            Console.WriteLine("init pos = " + string.Join(",", init_pos_cr15));
                            break;
                        }
                    }
                    break;
                }
            }
        }

        // Basic Methods
        public void SetCr15InitPositionJoint(ref DualRobot drCR15)
        {
            // Set Cr15 Init Position Xyzwpr
            drCR15.SetPosXyzwpr(Position.InitPos_Scene1B, init_pos_cr15);
            // cout_pos("init_pos_cr15", init_pos_cr15);

            // transfer to pos in joint
            var pos_in_joint = drCR15.GetPosJoint(Position.InitPos_Scene1B);
            
            // Set Cr15 Init Position Joint
            drCR15.SetPosJoint(Position.InitPos_Scene1B, pos_in_joint); 
        }
        public void SetCr15InitPositionXyzwpr(ref DualRobot dr, Position pos)
        {
            // Set Cr15 Init Position Xyzwpr
            dr.SetPosXyzwpr(pos, init_pos_cr15);
            // cout_pos("init_pos_cr15", init_pos_cr15);
            
            // // transfer to pos in joint
            // var pos_in_joint = drCR15.GetPosJoint(Position.InitPos_Scene1B);
            //
            // // Set Cr15 Init Position Joint
            // drCR15.SetPosJoint(Position.InitPos_Scene1B, pos_in_joint);
        }
        public void SetCr7InitPositionXyzwpr(ref DualRobot dr, Position pos)
        {
            // Set Cr15 Init Position Xyzwpr
            dr.SetPosXyzwpr(pos, init_pos_cr7);
            // cout_pos("init_pos_cr15", init_pos_cr15);

            // // transfer to pos in joint
            // var pos_in_joint = drCR15.GetPosJoint(Position.InitPos_Scene1B);
            //
            // // Set Cr15 Init Position Joint
            // drCR15.SetPosJoint(Position.InitPos_Scene1B, pos_in_joint);
        }

        private void cout_pos(string name, float[] pos)
        {
            Console.WriteLine(name + ": " + pos[0] + ", " + pos[1] + ", " + 
                              pos[2] + ", " + pos[3] + ", " + pos[4] + ", " + pos[5]);
        }
        private void cout_TMat(string name, Matrix<double> tmat)
        {
            Console.WriteLine(name + ": ");
            Console.WriteLine();
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    Console.Write(tmat[i, j] + "\t");
                }
                Console.WriteLine();
            }
        }

        // Scene4
    }
}
