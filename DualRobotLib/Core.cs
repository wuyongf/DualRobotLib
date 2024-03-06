using System;
using System.Collections.Generic;
using System.Collections.Specialized;
using System.Diagnostics;
using System.Globalization;
using System.Linq;
using System.Runtime.InteropServices;
using System.Runtime.Remoting.Channels;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using FRRJIf;
using FRRobotIFLib;
using MathNet.Numerics.LinearAlgebra;
using SinoDev;

namespace DualRobotLib
{
    public class Core
    {
        // Robot & TF
        private DualRobot drCR7 = new DualRobot();
        private Transformation tfCR7 = new Transformation();
        private DualRobot drCR15 = new DualRobot();
        private Transformation tfCR15 = new Transformation();
        private Trans _trans = new Trans();

        // LiftTable
        private LiftTable _liftTable = new LiftTable();

        // Motor
        private Motor _motor = new Motor();
        private Transformation tfMotor = new Transformation();

        // Calibration
        private Calibration cal = new Calibration();

        // TCP Manager
        private TCPManager cr7_tcpManager = new TCPManager();
        private TCPManager cr15_tcpManager = new TCPManager();

        // SceneX
        private SceneX scene1a = new SceneX();
        private SceneX scene1b = new SceneX();
        private SceneX scene1c = new SceneX();
        private SceneX scene2 = new SceneX();
        private SceneX scene3 = new SceneX();
        private SceneX scene4 = new SceneX();

        // Thread
        private Thread th_scene1c;

        // SceneStatus
        private int SceneStatus_Cr7, SceneStatus_Cr15;

        private bool SysIsEStop()
        {
            // check cr7, cr15 status
            if (drCR7.isEStop() || drCR15.isEStop())
                return true;
            else
                return false;
        }

        /// <summary>
        /// Get Robotic Arm Moving Status.
        /// </summary>
        /// <returns>
        /// 0: Idle
        /// 1: Moving
        /// 2: Arrived
        /// 3: Measurement Finish
        /// 4: Waiting
        /// </returns>
        public int GetMoveFlag(Model model)
        {
            switch (model)
            {
                case Model.CR15:
                {
                    // Console.WriteLine("MoveFlag: " + drCR15.GetMoveFlag());
                    return drCR15.GetMoveFlag();
                }
                case Model.CR7:
                {
                    // Console.WriteLine("MoveFlag: " + drCR7.GetMoveFlag());
                    return drCR7.GetMoveFlag();
                }
                default: return -2;
            }
        }

        /// <summary>
        /// Set Robotic Arm Moving Status.
        /// 0: Idle
        /// 1: Moving
        /// 2: Arrived
        /// 3: Measurement Finish
        /// 4: Waiting
        /// </summary>
        public bool SetMoveFlag(Model model, int move_flag)
        {
            switch (model)
            {
                case Model.CR15:
                {
                    return drCR15.SetMoveFlag(move_flag);
                }
                case Model.CR7:
                {
                    // Console.WriteLine("MoveFlag: " + drCR7.GetMoveFlag());
                    return drCR7.SetMoveFlag(move_flag);
                }
                default:
                    return false;
            }
        }

        /// <summary>
        /// Get TCP speed
        /// </summary>
        /// <param name="model"></param>
        /// <returns> speed(mm/sec) </returns>
        public int GetSpeed(Model model)
        {
            switch (model)
            {
                case Model.CR15:
                {
                    return drCR15.GetSpeed();
                }
                case Model.CR7:
                {
                    return drCR7.GetSpeed();
                }
                default: return -2;
            }
        }

        /// <summary>
        /// Set TCP speed
        /// </summary>
        /// <param name="model"> robotic arm model</param>
        /// <param name="speed"> default：100 mm/sec </param>
        /// <returns></returns>
        public bool SetSpeed(Model model, int speed)
        {
            switch (model)
            {
                case Model.CR15:
                {
                    return drCR15.SetSpeed(speed);
                }
                case Model.CR7:
                {
                    return drCR7.SetSpeed(speed);
                }
                default:
                    return false;
            }
        }

        /// <summary>
        /// Set Robotic Arm Payload Number 1-10.
        /// </summary>
        /// <param name="model"> robotic arm model</param>
        /// <param name="payloadNo"> default：[1,10] </param>
        /// <returns></returns>
        public bool SetPayloadNo(Model model, int payloadNo)
        {
            switch (model)
            {
                case Model.CR15:
                {
                    return drCR15.SetPayloadNo(payloadNo);
                }
                case Model.CR7:
                {
                    return drCR7.SetPayloadNo(payloadNo);
                }
                default:
                    return false;
            }
        }

        // thread
        public void thread_MoveFlag(Model model)
        {
            while (true)
            {
                switch (model)
                {
                    case Model.CR15:
                    {
                        var res = GetMoveFlag(Model.CR15);

                        Console.WriteLine("CR15 MoveFlag: " + res);

                        break;
                    }
                    case Model.CR7:
                    {
                        var res = GetMoveFlag(Model.CR7);

                        Console.WriteLine("CR7 MoveFlag: " + res);

                        break;
                    }
                }
            }
        }

        /// <summary>
        /// Check the status of the robotic arm program. 
        /// </summary>
        /// <returns>
        /// 0: program stop
        /// 1: program paused
        /// 2: program executing
        /// </returns>
        public int GetProgramStatus(Model model)
        {
            switch (model)
            {
                case Model.CR15:
                {
                    drCR15.GetProgramStatus();
                    return drCR15.State;
                }
                case Model.CR7:
                {
                    drCR7.GetProgramStatus();
                    return drCR7.State;
                }
                default: return 0;
            }
        }

        /// <summary>
        /// Get Scene Status
        /// </summary>
        /// <param name="model"></param>
        /// <returns>
        /// 0: Idle
        /// 1: Running
        /// 2: Cancel
        /// 3: Finish
        /// </returns>
        private int GetSceneStatus(Model model)
        {
            switch (model)
            {
                case Model.CR15:
                {
                    return drCR15.GetSceneStatus();
                }
                case Model.CR7:
                {
                    return drCR7.GetSceneStatus();
                }
                default: return -2;
            }
        }

        /// <summary>
        /// Set Scene Status
        /// 0: Idle
        /// 1: Running
        /// 2: Cancel
        /// 3: Finish
        /// </summary>
        private bool SetSceneStatus(int state)
        {
            var res1 = drCR15.SetSceneStatus(state);
            var res2 = drCR7.SetSceneStatus(state);

            return res1 && res2;
        }

        private void SetSceneStart()
        {
            // Scene Status
            this.SetSceneStatus(1);
        }

        private void SetSceneFinish()
        {
            // Cr7 Status
            SetMoveFlag(Model.CR7, 0);
            // Cr15 Status
            SetMoveFlag(Model.CR15, 0);
            // Scene Status
            this.SetSceneStatus(3);
        }

        private void SetSceneCancel()
        {
            // Cr7 Status
            SetMoveFlag(Model.CR7, 0);
            // Cr15 Status
            SetMoveFlag(Model.CR15, 0);
            // Scene Status
            this.SetSceneStatus(2);
        }

        /// <summary>
        /// Cancel current scene. Stop robotic arm motions.
        /// </summary>
        /// <returns></returns>
        public bool CancelCurrentScene()
        {
            // tfCR7.via_orbit_points_part2.Clear();
            // tfCR15.via_points.Clear();

            return SetSceneStatus(2);
        }

        /// <summary>
        /// Connect to a robot. 
        /// </summary>
        /// <remarks> Please check the ip address first. Default port number: 60008 </remarks>
        public bool Connect(Model model, string ip, int port, string comPort = "")
        {
            switch (model)
            {
                case Model.CR7:
                {
                    drCR7.IP = ip;
                    drCR7.Port = port;

                    if (drCR7.Init())
                    {
                        drCR7.IsConnectCalled = true;
                        return true;
                    }

                    Console.WriteLine("Connection Failed! Please Check!");
                    return false;
                }
                case Model.CR15:
                {
                    drCR15.IP = ip;
                    drCR15.Port = port;

                    if (drCR15.Init())
                    {
                        drCR15.IsConnectCalled = true;
                        return true;
                    }

                    Console.WriteLine("Connection Failed! Please Check!");
                    return false;
                }
                case Model.LiftTable:
                {
                    // 1. LiftTable Connection          
                    _liftTable.Connect(ip, port, comPort);

                    // Wait for connection successful
                    Thread.Sleep(2000);

                    //
                    return _liftTable.IsConnected();
                }
                default:
                {
                    Console.WriteLine("Please Select Correct Model");
                    return false;
                }
            }
        }

        /// <summary>
        /// Connect Motor comPort
        /// </summary>
        /// <param name="model"></param>
        /// <param name="comPort"></param>
        /// <returns></returns>
        public bool Connect(Model model, string comPort)
        {
            switch (model)
            {
                case Model.Motor:
                {
                    _motor.Connect(comPort);

                    return _motor.IsConnected();
                }
                default:
                {
                    Console.WriteLine("Please Select Correct Model and comPort");
                    return false;
                }
            }
        }

        /// <summary>
        /// Disconnect from a robot
        /// </summary>
        /// <param name="model"></param>
        /// <returns>Success returns True, fails returns False.</returns>
        public bool Disconnect(Model model)
        {
            switch (model)
            {
                case Model.CR7:
                {
                    return drCR7.Clear();
                }
                case Model.CR15:
                {
                    return drCR15.Clear();
                }
                case Model.LiftTable:
                {
                    return _liftTable.Disconnect();
                }
                case Model.Motor:
                {
                    _motor.Disconnect();
                    return !_motor.IsConnected();
                }
                default:
                {
                    Console.WriteLine("Please Select Correct Model");
                    return false;
                }
            }
        }

        /// <summary>
        /// Reset robotic arm movement.
        /// </summary>
        public void ResetMovement(Model model)
        {
            switch (model)
            {
                case Model.CR7:
                {
                    drCR7.setRegisterInt(1, 0, 1);
                    drCR7.setRegisterInt(2, 0, 1);
                    break;
                }
                case Model.CR15:
                {
                    drCR15.setRegisterInt(1, 0, 1);
                    drCR15.setRegisterInt(2, 0, 1);
                    break;
                }
            }
        }

        /// <summary>
        /// Configure the tool center point(TCP) of the robotic arm.
        /// </summary>
        public void SetTCP(Model model, float[] tcp)
        {
            switch (model)
            {
                case Model.CR15:
                {
                    drCR15.SetOffsetTCP(tcp);
                    break;
                }
                case Model.CR7:
                {
                    drCR7.SetOffsetTCP(tcp);
                    break;
                }
            }
        }

        /// <summary>
        /// Configure the tool center point(TCP) of the robotic arm.
        /// </summary>
        public float[] GetTCP(Model model)
        {
            float[] cur_tcp = new float[6];

            switch (model)
            {
                case Model.CR15:
                {
                    drCR15.GetOffsetTCP();
                    cur_tcp = drCR15.offset_tcp;
                    break;
                }
                case Model.CR7:
                {
                    drCR7.GetOffsetTCP();
                    cur_tcp = drCR7.offset_tcp;
                    break;
                }
            }

            return cur_tcp;
        }

        /// <summary>
        /// Get current position(tcp position).
        /// </summary>
        public double[] GetCurPos(Model model)
        {
            double[] cur_pos = new double[6];

            switch (model)
            {
                case Model.CR15:
                {
                    return drCR15.GetCurPos();
                }
                case Model.CR7:
                {
                    return drCR7.GetCurPos();
                }
                default:
                    return cur_pos;
            }
        }

        /// <summary>
        /// Set current position as a new frame.
        /// </summary>
        /// <param name="model"></param>
        public void SetUserFrame(Model model)
        {
            switch (model)
            {
                case Model.CR15:
                {
                    drCR15.SetUserFrame();
                    break;
                }
                case Model.CR7:
                {
                    drCR7.SetUserFrame();
                    break;
                }
            }
        }

        public bool SetSinglePoint(Model model, float[] point)
        {
            switch (model)
            {
                case Model.CR15:
                {
                    var res = drCR15.setRegisterPosXyzwpr(99, point, drCR15.config, drCR15.RpParamUF, drCR15.RpParamUT);
                    robotWaitForReady(drCR15);
                    return res;
                }
                case Model.CR7:
                {
                    var res = drCR7.setRegisterPosXyzwpr(99, point, drCR7.config, drCR7.RpParamUF, drCR7.RpParamUT);
                    robotWaitForReady(drCR7);
                    return res;
                }
                default:
                {
                    return false;
                }
            }
        }

        public void MoveSinglePoint(Model model)
        {
            switch (model)
            {
                case Model.CR15:
                {
                    // 0. Initialization
                    drCR15.setRegisterInt(1, 0, 1);
                    drCR15.setRegisterInt(2, 0, 1);
                    drCR15.setRegisterInt(3, 0, 1);

                    // 1. Move to uf_via_point
                    drCR15.setRegisterInt(1, 4, 1);
                    drCR15.setRegisterInt(2, 2, 1);

                    robotWaitForReady(drCR15);

                    break;
                }
                case Model.CR7:
                {
                    // 0. Initialization
                    drCR7.setRegisterInt(1, 0, 1);
                    drCR7.setRegisterInt(2, 0, 1);
                    drCR7.setRegisterInt(3, 0, 1);

                    // 1. Move to uf_via_point
                    drCR7.setRegisterInt(1, 4, 1);
                    drCR7.setRegisterInt(2, 2, 1);

                    robotWaitForReady(drCR7);
                    break;
                }
            }
        }

        private double GetTcpDistance_Scene2(Model model)
        {
            double r = 0;

            switch (model)
            {
                case Model.CR15:
                {
                    // var station_antenna_tcp_cr7 = this.cr15_tcpManager.StationAntennaTCP;
                    r = cal.GetTcpDistance_Scene2(Model.CR15, drCR7.GetRBCurPos(), drCR15.GetRBCurPos());
                    break;
                }
                case Model.CR7:
                {
                    r = cal.GetTcpDistance_Scene2(Model.CR7, drCR7.GetRBCurPos(), drCR15.GetRBCurPos());
                    break;
                }
            }
            return r;
        }

        private double GetTcpDistance_Scene1A(Model model)
        {
            double r = 0;

            switch (model)
            {
                case Model.CR15:
                {
                    r = cal.GetTcpDistance_Scene2(Model.CR15, drCR7.GetRBCurPos(), drCR15.GetRBCurPos()) 
                        - _liftTable._linear.GetData(0) - 0.85; 
                    // need to subtract lift table current height，because we calibrate the lift at the lowest position.
                    // need to subtract 0.85 because of the real error between DUT and Cr15 Probe
                    break;
                }
                default:break;
            }
            return r;
        }

        public void RobotBaseCalibrationInit(double[] Pos_Cr7_CalliBase, double[] Pos_Cr15_CalliBase)
        {
            cal.SetCalibrationTMat(Pos_Cr7_CalliBase, Pos_Cr15_CalliBase);
        }

        public float[] GetRobotBaseCalibrationPose()
        {
            return _trans.T2pos(cal.T_Cr15_Cr7);
        }

        // RotationPlateCalibrationInit
        public double[] GetStationAntennaTCP_Cr7()
        {
            return _trans.T2posd(cal.T_Cr7_PlateCenter);
        }
        public void SetStationAntennaTCP_Cr7(double[] pos)
        {
            cal.SetCr7PlateCenter(pos);
        }

        /// <summary>
        /// Start Scene1B. //deprecated
        /// </summary>
        /// <param name="movement_type"></param>
        public void Scene1B(MovementType movement_type)
        {
            // Set Scene Status
            SetSceneStatus(1);

            // 0. assign the param

            double cr15_radius = 0;
            double cr7_motion2_radius = 0;
            double cr7_motion3_radius = 0;

            tfCR15.uf_measure_radius = cr15_radius;
            tfCR15.uf_measure_arc = scene1b.cr15_arc;
            tfCR15.uf_measure_step_angle = scene1b.cr15_step_angle;

            tfCR7.uf_measure_radius = cr7_motion2_radius;
            tfCR7.uf_measure_arc = scene1b.cr7_motion2_arc;
            tfCR7.uf_measure_step_angle = scene1b.cr7_motion2_step_angle;

            // 1. CR7 Path -- motion3
            tfCR7.via_orbit_points_part2 = tfCR7.get_uf_orbit_points_part2_v01(cr7_motion3_radius,
                scene1b.cr7_motion3_arc, scene1b.cr7_motion3_step_angle);
            // tfCR7.via_orbit_points_part2 = tfCR7.get_uf_orbit_points_part2_v01(0, 180, 45);

            var via_orbit_points_part2_no = tfCR7.via_orbit_points_part2.Count;

            // loop -- part 2

            for (int i = 0; i < via_orbit_points_part2_no; i++)
            {
                // Check Scene Status
                SceneStatus_Cr7 = this.GetSceneStatus(Model.CR7);
                SceneStatus_Cr15 = this.GetSceneStatus(Model.CR15);
                if (SceneStatus_Cr7 == 2 || SceneStatus_Cr15 == 2)
                {
                    break;
                }

                // move to via_point
                drCR7.Motion_aPoint(tfCR7.via_orbit_points_part2[i]);

                robotWaitForReady(drCR7);

                // (1). change to offset_tcp_temp
                var alpha = -tfCR7.via_orbit_points_part2[i][5];

                // Console.WriteLine("drCR7.offset_tcp:" + drCR7.offset_tcp[3] + "," + drCR7.offset_tcp[4] + "," +
                //                   drCR7.offset_tcp[5]);

                var rpy = tfCR7.GetTempRPY(drCR7.offset_tcp, (float)alpha);

                var res_offset = drCR7.SetOffsetTCPTemp(rpy);

                // (2). loop -- part 1 -- swing (motion2)
                tfCR7.via_orbit_points_part1 = tfCR7.get_uf_orbit_points_part1_v03(scene1b.cr7_motion2_arc,
                    scene1b.cr7_motion2_step_angle, tfCR7.via_orbit_points_part2[i], rpy);
                // tfCR7.via_orbit_points_part1 = tfCR7.get_uf_orbit_points_part1_v03(180, 90, PosArray_part2, rpy);

                var via_orbit_points_part1_no = tfCR7.via_orbit_points_part1.Count;

                robotWaitForReady(drCR7);

                for (int j = 0; j < via_orbit_points_part1_no; j++)
                {
                    // Check Scene Status
                    SceneStatus_Cr7 = this.GetSceneStatus(Model.CR7);
                    SceneStatus_Cr15 = this.GetSceneStatus(Model.CR15);
                    if (SceneStatus_Cr7 == 2 || SceneStatus_Cr15 == 2)
                    {
                        break;
                    }

                    // move to via_point
                    drCR7.Motion_aPoint(tfCR7.via_orbit_points_part1[j]);

                    robotWaitForReady(drCR7);

                    // CR15 Via Points
                    tfCR15.via_points = tfCR15.get_1B_points_Cr15_motion1(cr15_radius, scene1b.cr15_arc, scene1b.cr15_step_angle,
                        scene1b.tcp_distance, tfCR7.via_orbit_points_part2[i][5], j+1);

                    robot_Motion_ViaPoints(SceneName.None, drCR15, ref tfCR15, ref movement_type);
                }

                // Check Scene Status One More Time
                if (SceneStatus_Cr7 == 2 || SceneStatus_Cr15 == 2)
                {
                    // Do Nothing
                }
                else
                {
                    // (3). change to offset_tcp
                    robotWaitForReady(drCR7);
                    drCR7.SetOffsetTCP();
                    robotWaitForReady(drCR7);

                    // CR7 returns to via_position of part2.
                    drCR7.SetOrigin();
                    drCR7.MovetoOrigin();
                    robotWaitForReady(drCR7);
                }
            }
        }
        private void Scene1B_Bak(MovementType movement_type)
        {
            // Set Scene Status
            SetSceneStatus(1);

            // 0. assign the param

            double cr15_radius = 0;
            double cr7_motion2_radius = 0;
            double cr7_motion3_radius = 0;

            tfCR15.uf_measure_radius = cr15_radius;
            tfCR15.uf_measure_arc = scene1b.cr15_arc;
            tfCR15.uf_measure_step_angle = scene1b.cr15_step_angle;

            tfCR7.uf_measure_radius = cr7_motion2_radius;
            tfCR7.uf_measure_arc = scene1b.cr7_motion2_arc;
            tfCR7.uf_measure_step_angle = scene1b.cr7_motion2_step_angle;

            // 1. CR15 Path
            // tfCR15.via_points = tfCR15.get_uf_measure_points_v03(cr15_radius, cr15_arc, cr15_step_angle);
            tfCR15.via_points = tfCR15.get_uf_measure_points_v04(cr15_radius, scene1b.cr15_arc, scene1b.cr15_step_angle,
                scene1b.tcp_distance);
            // tfCR15.via_points = tfCR15.get_uf_measure_points_v03(0, 130, 0.5);

            // 2. CR7 Path -- motion3
            tfCR7.via_orbit_points_part2 = tfCR7.get_uf_orbit_points_part2_v01(cr7_motion3_radius,
                scene1b.cr7_motion3_arc, scene1b.cr7_motion3_step_angle);
            // tfCR7.via_orbit_points_part2 = tfCR7.get_uf_orbit_points_part2_v01(0, 180, 45);

            var via_orbit_points_part2_no = tfCR7.via_orbit_points_part2.Count;

            // loop -- part 2

            for (int i = 0; i < via_orbit_points_part2_no; i++)
            {
                // Check Scene Status
                SceneStatus_Cr7 = this.GetSceneStatus(Model.CR7);
                SceneStatus_Cr15 = this.GetSceneStatus(Model.CR15);
                if (SceneStatus_Cr7 == 2 || SceneStatus_Cr15 == 2)
                {
                    break;
                }

                // config the next via_point
                short UF = 1;
                short UT = 2;

                float[] PosArray_part2 = new float[6];
                short[] ConfigArray = new short[6];

                PosArray_part2[0] = float.Parse(tfCR7.via_orbit_points_part2[i][0].ToString());
                PosArray_part2[1] = float.Parse(tfCR7.via_orbit_points_part2[i][1].ToString());
                PosArray_part2[2] = float.Parse(tfCR7.via_orbit_points_part2[i][2].ToString());
                PosArray_part2[3] = float.Parse(tfCR7.via_orbit_points_part2[i][3].ToString());
                PosArray_part2[4] = float.Parse(tfCR7.via_orbit_points_part2[i][4].ToString());
                PosArray_part2[5] = float.Parse(tfCR7.via_orbit_points_part2[i][5].ToString());

                var res = drCR7.setRegisterPosXyzwpr(99, PosArray_part2, drCR7.config, UF, UT);

                // Move
                drCR7.setRegisterInt(1, 4, 1);
                drCR7.setRegisterInt(2, 2, 1);

                robotWaitForReady(drCR7);

                // (1). change to offset_tcp_temp
                var alpha = -PosArray_part2[5];

                // Console.WriteLine("drCR7.offset_tcp:" + drCR7.offset_tcp[3] + "," + drCR7.offset_tcp[4] + "," +
                //                   drCR7.offset_tcp[5]);

                var rpy = tfCR7.GetTempRPY(drCR7.offset_tcp, alpha);

                var res_offset = drCR7.SetOffsetTCPTemp(rpy);

                // (2). loop -- part 1 -- swing (motion2)
                tfCR7.via_orbit_points_part1 = tfCR7.get_uf_orbit_points_part1_v03(scene1b.cr7_motion2_arc,
                    scene1b.cr7_motion2_step_angle, PosArray_part2.ToDouble(), rpy);
                // tfCR7.via_orbit_points_part1 = tfCR7.get_uf_orbit_points_part1_v03(180, 90, PosArray_part2, rpy);

                var via_orbit_points_part1_no = tfCR7.via_orbit_points_part1.Count;

                robotWaitForReady(drCR7);

                for (int j = 0; j < via_orbit_points_part1_no; j++)
                {
                    // Check Scene Status
                    SceneStatus_Cr7 = this.GetSceneStatus(Model.CR7);
                    SceneStatus_Cr15 = this.GetSceneStatus(Model.CR15);
                    if (SceneStatus_Cr7 == 2 || SceneStatus_Cr15 == 2)
                    {
                        break;
                    }

                    // config the next via_point
                    UF = 1;
                    UT = 2;

                    float[] PosArray_part1 = new float[6];
                    ConfigArray = new short[6];

                    PosArray_part1[0] = float.Parse(tfCR7.via_orbit_points_part1[j][0].ToString());
                    PosArray_part1[1] = float.Parse(tfCR7.via_orbit_points_part1[j][1].ToString());
                    PosArray_part1[2] = float.Parse(tfCR7.via_orbit_points_part1[j][2].ToString());
                    PosArray_part1[3] = float.Parse(tfCR7.via_orbit_points_part1[j][3].ToString());
                    PosArray_part1[4] = float.Parse(tfCR7.via_orbit_points_part1[j][4].ToString());
                    PosArray_part1[5] = float.Parse(tfCR7.via_orbit_points_part1[j][5].ToString());

                    res = drCR7.setRegisterPosXyzwpr(99, PosArray_part1, drCR7.config, UF, UT);

                    // Move
                    drCR7.setRegisterInt(1, 4, 1);
                    drCR7.setRegisterInt(2, 2, 1);

                    robotWaitForReady(drCR7);

                    /// CR15
                    robot_Motion_ViaPoints(SceneName.None, drCR15, ref tfCR15, ref movement_type);
                }

                // Check Scene Status One More Time
                if (SceneStatus_Cr7 == 2 || SceneStatus_Cr15 == 2)
                {
                    // Do Nothing
                }
                else
                {
                    // (3). change to offset_tcp
                    robotWaitForReady(drCR7);
                    drCR7.SetOffsetTCP();
                    robotWaitForReady(drCR7);

                    // CR7 returns to via_position of part2.
                    drCR7.SetOrigin();
                    drCR7.MovetoOrigin();
                    robotWaitForReady(drCR7);
                }
            }
        }
        private void thread_uf_motion_circle(object syncLock, ref DualRobot dr, ref Transformation tf)
        {
            lock (syncLock)
            {
                uf_motion_circle(ref dr, ref tf);
            }
        }

        private void uf_motion_circle(ref DualRobot dr, ref Transformation tf)
        {
            // check List Size
            var via_points_no = tf.via_points.Count;

            Console.WriteLine("via_point number: " + via_points_no);

            // loop
            for (int i = 0; i < via_points_no; i++)
            {
                robotWaitForReady(dr);

                // config the next via_point
                short UF = 1;
                short UT = 1;

                float[] PosArray = new float[6];

                PosArray[0] = float.Parse(tf.via_points[i][0].ToString());
                PosArray[1] = float.Parse(tf.via_points[i][1].ToString());
                PosArray[2] = float.Parse(tf.via_points[i][2].ToString());
                PosArray[3] = float.Parse(tf.via_points[i][3].ToString());
                PosArray[4] = float.Parse(tf.via_points[i][4].ToString());
                PosArray[5] = float.Parse(tf.via_points[i][5].ToString());

                var res = dr.setRegisterPosXyzwpr(99, PosArray, dr.config, UF, UT);

                // Move
                dr.setRegisterInt(1, 4, 1);
                dr.setRegisterInt(2, 2, 1);

                robotWaitForReady(dr);

                if (i == 0)
                {
                    Console.WriteLine("CR15 Arrived!");
                }

                if (i == via_points_no)
                {
                    Console.WriteLine("CR15 Arrived!");
                }

                // Count
                dr.setRegisterInt(3, i + 1, 1);

                // MessageBox.Show("via_point_no: " + (i + 1).ToString());
            }
        }

        /// <summary>
        /// Get the current point location.
        /// </summary>
        /// <returns>
        /// double[0]: angle
        /// double[1]: line_no
        /// double[2]: point_no
        /// </returns>
        public double[] GetViaPointLocation()
        {
            Console.WriteLine("ViaPointLocation: " + drCR15.via_point_location[0] + " " +
                              drCR15.via_point_location[1] + " " + drCR15.via_point_location[2]);
            return drCR15.via_point_location;
        }

        /// <summary>
        /// Get Via Points Info
        /// </summary>
        /// <param name="scene"></param>
        /// <returns>
        /// double[0]: total_angle_no
        /// double[1]: total_line_no
        /// double[2]: total_point_no (each line)
        /// </returns>
        public double[] GetViaPointsInfo(SceneName scene)
        {
            double[] info = new double[3];

            switch (scene)
            {
                case SceneName.Scene1A:
                {
                    info[0] = tfMotor.get_1a_points_motor_motion(scene1a.motor_scene1a_arc,
                        scene1a.motor_scene1a_step_angle).Count;
                    info[1] = 1;
                    info[2] = tfCR15.get_1a_points_Cr15_motion1a(scene1a.cr15_scene1a_R, scene1a.cr15_scene1a_motion1_arc, scene1a.cr15_scene1a_motion1_step_angle, 0).Count;
                    break;
                }
                case SceneName.Scene1A_Sim:
                {
                    info[0] = tfMotor.get_1a_points_motor_motion(scene1a.motor_scene1a_arc,
                        scene1a.motor_scene1a_step_angle).Count;
                    info[1] = 1;
                    info[2] = tfCR15.get_1a_points_Cr15_motion1a(scene1a.cr15_scene1a_R, scene1a.cr15_scene1a_motion1_arc, scene1a.cr15_scene1a_motion1_step_angle, 0).Count;
                    break;
                }
                case SceneName.Scene1B:
                {
                    info[0] = tfCR7.get_uf_orbit_points_part2_v01(0, scene1b.cr7_motion3_arc, scene1b.cr7_motion3_step_angle).Count;
                    info[1] = tfCR7.get_uf_orbit_points_part1_v03_no(scene1b.cr7_motion2_arc, scene1b.cr7_motion2_step_angle).Count;
                    info[2] = tfCR15.get_1B_points_Cr15_motion1(0, scene1b.cr15_arc, scene1b.cr15_step_angle, scene1b.tcp_distance, 0, 0).Count;
                    break;
                }
                case SceneName.Scene1C:
                {
                    info[0] = 2;
                    info[1] = scene1c.noOfWStep;
                    info[2] = scene1c.noOfHStep;
                    break;
                }
                case SceneName.Scene2:
                {
                    info[0] = 2;
                    info[1] = tfCR15.get_2_points_Cr15_motion2(scene2.cr15_scene2_motion2_arc, scene2.cr15_scene2_motion2_step_angle).Count;
                    info[2] = tfCR15.get_2_points_Cr15_motion1a(scene2.cr15_scene2_R, scene2.cr15_scene2_motion1_arc, scene2.cr15_scene2_motion1_step_angle, 0).Count;
                    break;
                }
                case SceneName.Scene2_Sim:
                {
                    info[0] = 2;
                    info[1] = tfCR15.get_2_points_Cr15_motion2(scene2.cr15_scene2_motion2_arc, scene2.cr15_scene2_motion2_step_angle).Count;
                    info[2] = tfCR15.get_2_points_Cr15_motion1a(scene2.cr15_scene2_R, scene2.cr15_scene2_motion1_arc, scene2.cr15_scene2_motion1_step_angle, 0).Count;
                    break;
                }
            }

            return info;
        }

        /// <summary>
        /// Get Spanning Width.
        /// </summary>
        /// <param name="separationZ"> range:[0,800]; unit:mm;</param>
        /// <returns></returns>
        public double GetSpanningWidth(double separationZ)
        {
            // reference: https://mycurvefit.com/

            // fit data:
            // 0,450
            // 200,550
            // 400,700
            // 600,850
            // 800,900

            // return 1041.335 + (451.9924 - 1041.335) / (1 + Math.Pow(( separationZ / 447.2165), 2.097963));
            return 4920943 + (201.1823 - 4920943) / (1 + Math.Pow((separationZ / 125452000000.00002), 0.4818042));

            // return 807.7229 * Math.Exp((-Math.Pow((separationZ - 889.4056), 2) / (2 * Math.Pow(598.5435, 2))));
        }

        /// <summary>
        /// Get Spanning Height.
        /// </summary>
        /// <param name="separationZ"> range:[0,800]; unit:mm;</param>
        /// <returns></returns>
        public double GetSpanningHeight(double separationZ)
        {
            // reference: https://mycurvefit.com/

            // fit data:
            // 0,450
            // 200,550
            // 400,700
            // 600,850
            // 800,900

            return 588.1711 * Math.Exp((-Math.Pow((separationZ - 666.809), 2) / (2 * Math.Pow(413.5609, 2))));
        }

        // Calibration
        public float[] GetToolAntennaTCP(Model model, float[] fixture_tcp, float[] offset)
        {
            float[] tool_antenna_tcp = new float[6];

            switch (model)
            {
                case Model.CR7:
                {
                    tool_antenna_tcp = cr7_tcpManager.GetToolAntennaTCP(fixture_tcp, offset);
                    break;
                }
                case Model.CR15:
                {
                    tool_antenna_tcp = cr15_tcpManager.GetToolAntennaTCP(fixture_tcp, offset);
                    break;
                }
            }

            return tool_antenna_tcp;
        }

        public float[] GetToolFixtureTCP(Model model, float[] cal_pin_tcp, float offset)
        {
            float[] tool_fixruere_tcp = new float[6];

            switch (model)
            {
                case Model.CR7:
                {
                    tool_fixruere_tcp = cr7_tcpManager.GetToolFixtureTCP(cal_pin_tcp, offset);
                    break;
                }
                case Model.CR15:
                {
                    tool_fixruere_tcp = cr15_tcpManager.GetToolFixtureTCP(cal_pin_tcp, offset);
                    break;
                }
            }

            return tool_fixruere_tcp;
        }
        public double[] GetStationAntennaTCP(float[] station_center_tcp, float[] antenna_offset, float station_offset)
        {
            float[] tool_fixruere_tcp = new float[6];

            tool_fixruere_tcp = cr7_tcpManager.GetStationAntennaTCP(station_center_tcp, antenna_offset, station_offset);

            return tool_fixruere_tcp.ToDouble();
        }

        public float[] GetStationCenterZeroTCP(float[] station_cal_pin_tcp_cr7, float station_cal_pin_length)
        {
            return cr7_tcpManager.GetStationCenterZeroTCP(station_cal_pin_tcp_cr7, station_cal_pin_length);
        }

        public float[] GetToolFixtureWPR(SceneName scene, Model robot, 
                                        float[] originPosWPR, float[] defaultPosWPR)
        {
            float[] wpr = new float[3];

            float[] originPos = new float[6];
            float[] defaultPos = new float[6];

            originPos[3] = originPosWPR[0]; originPos[4] = originPosWPR[1]; originPos[5] = originPosWPR[2];
            defaultPos[3] = defaultPosWPR[0]; defaultPos[4] = defaultPosWPR[1]; defaultPos[5] = defaultPosWPR[2];

            switch (robot)
            {
                case Model.CR15:
                {
                    wpr =  cr15_tcpManager.GetToolFixtureWPR(scene, robot, cal.T_Cr15_Cr7, originPos, defaultPos);
                    break;
                }
                case Model.CR7:
                {
                    wpr = cr7_tcpManager.GetToolFixtureWPR(scene, robot, cal.T_Cr15_Cr7, originPos, defaultPos);
                    break;
                }
            }

            return wpr;
        }

        public float[] GetToolFixtureWPR(Model robot,
            float[] originPosWPR, float[] defaultPosWPR)
        {
            float[] wpr = new float[3];

            float[] originPos = new float[6];
            float[] defaultPos = new float[6];

            originPos[3] = originPosWPR[0]; originPos[4] = originPosWPR[1]; originPos[5] = originPosWPR[2];
            defaultPos[3] = defaultPosWPR[0]; defaultPos[4] = defaultPosWPR[1]; defaultPos[5] = defaultPosWPR[2];

            switch (robot)
            {
                case Model.CR15:
                {
                    wpr = cr15_tcpManager.GetToolFixtureWPR(robot, cal.T_Cr15_Cr7, originPos, defaultPos);
                    break;
                }
                case Model.CR7:
                {
                    wpr = cr7_tcpManager.GetToolFixtureWPR(robot, cal.T_Cr15_Cr7, originPos, defaultPos);
                    break;
                }
            }

            return wpr;
        }

        // Motor
        /// <summary>
        /// Motor Initialization.
        /// </summary>
        public bool MotorInit()
        {
            // 1. Initialization
            _motor.Init();

            // // 2. Return Zero Position
            // MotorAbsMoveTo(0);

            if (_motor.IsConnected())
                return true;
            return false;
        }

        /// <summary>
        /// Get motor connection status.
        /// </summary>
        /// <returns>
        /// True: Connected <br/>
        /// False: Disconnected <br/>
        /// </returns>
        public bool MotorIsConnected()
        {
            return _motor.IsConnected();
        }

        /// <summary>
        /// Motor absolute movement.
        /// </summary>
        /// <param name="degree"></param>
        public void MotorAbsMoveTo(double degree)
        {
            _motor.SetAbsDegree(degree);

            int count = 0;
            bool check_flag = true;
            while (check_flag)
            {
                Thread.Sleep(1000);

                if (!this.MotorIsMoving())
                {
                    count++;
                    if (count == 3)
                        break;
                }
            }
        }

        /// <summary>
        /// Motor relative movement.
        /// </summary>
        /// <param name="degree"></param>
        public void MotorRelMoveTo(double degree)
        {
            _motor.SetRelDegree(degree);

            int count = 0;
            bool check_flag = true;
            while (check_flag)
            {
                Thread.Sleep(1000);

                if (!this.MotorIsMoving())
                {
                    count++;
                    if (count == 3)
                        break;
                }
            }
        }

        /// <summary>
        /// Get motor movement status.
        /// </summary>
        /// <returns>
        /// True: is moving <br/>
        /// False: is not moving <br/>
        /// </returns>
        public bool MotorIsMoving()
        {
            return _motor.Get_IsMoving();
        }

        /// <summary>
        /// Get motor current position.
        /// </summary>
        /// <returns>
        /// current position(deg)
        /// </returns>
        public double MotorGetDegree()
        {
            return _motor.Get_Degree();
        }

        // LiftTable
        /// <summary>
        /// LiftTable Initialization.
        /// </summary>
        public bool LiftTableInit()
        {
            // 1. Init
            _liftTable.Init();

            // 2. LiftTable Move to Lowest Position.
            _liftTable.MoveToLowestPos();
            // 3. Linear Scale: Set Cls
            _liftTable._linear.CLRxyzAxis();

            if (_liftTable.IsConnected())
                return true;

            return false;
        }

        private bool LiftTableInit2()
        {
            var res = _liftTable.CheckIsLowest();
            Console.WriteLine("_liftTable.CheckIsLowest(): " + res);

            return false;
        }

        /// <summary>
        /// LiftTable absolute movement.
        /// </summary>
        /// <param name="height"></param>
        public void LiftTableAbsMoveTo(double height)
        {
            LiftTableMoveTo(0);

            LiftTableMoveTo(height);

            int count = 0;
            bool check_flag = true;
            while (check_flag)
            {
                Thread.Sleep(1000);

                if (!this.LiftTableIsMoving())
                {
                    count++;
                    if (count == 3)
                        break;
                }
            }
        }

        private void LiftTableMoveTo(double target_height)
        {
            // // Safety Issue: Move to Lowest Position first.
            // this.MoveToLowestPos();

            if (_liftTable.IsSafe())
            {
                // Safety Issue: highest & lowest
                if (target_height > _liftTable._highest)
                {
                    Console.WriteLine("LiftTable Limit! Cannot reach the height.");
                    Console.WriteLine("LiftTable Range: [" + _liftTable._lowest + ", " + _liftTable._highest + "]");
                    return;
                }
                if (target_height < 0) target_height = 0;

                var cur_x = _liftTable._linear.GetData(0);

                int count = 0;

                while (count < 10)
                {
                    count++;

                    if (cur_x < target_height - 4 * _liftTable._buffer)
                    {
                        if (this.SysIsEStop())
                        {
                            _liftTable.Stop();
                            break;
                        }
                        _liftTable.Up();

                        Debug.WriteLine("target linear scale height: " + target_height);
                        Debug.WriteLine("buffer: +-" + _liftTable._buffer);

                        while (true)
                        {
                            if (this.SysIsEStop())
                            {
                                _liftTable.Stop();
                                break;
                            }

                            cur_x = _liftTable._linear.GetData(0);

                            if (cur_x >= target_height - 4 * _liftTable._buffer)
                            {
                                _liftTable.Stop();
                                Thread.Sleep(100);
                                _liftTable.Stop();
                                Debug.WriteLine("cur linear scale height: " + _liftTable._linear.GetData(0));

                                break;
                            }

                            // Check one more time.
                            Thread.Sleep(500);
                            cur_x = _liftTable._linear.GetData(0);
                            Debug.WriteLine("cur linear scale height: " + cur_x);
                        }
                    }

                    if (cur_x > target_height + 4 * _liftTable._buffer)
                    {
                        if (this.SysIsEStop())
                        {
                            _liftTable.Stop();
                            break;
                        }
                        _liftTable.Down();

                        while (true)
                        {
                            if (this.SysIsEStop())
                            {
                                _liftTable.Stop();
                                break;
                            }

                            cur_x = _liftTable._linear.GetData(0);

                            if (cur_x <= target_height + 4 * _liftTable._buffer)
                            {
                                _liftTable.Stop();
                                Thread.Sleep(100);
                                _liftTable.Stop();

                                Debug.WriteLine("cur linear scale height: " + _liftTable._linear.GetData(0));

                                break;
                            }

                            // Check one more time.
                            Thread.Sleep(2000);
                            cur_x = _liftTable._linear.GetData(0);
                            Debug.WriteLine("cur linear scale height: " + cur_x);
                        }
                    }

                    if (cur_x >= target_height - 4 * _liftTable._buffer && cur_x <= target_height + 4 * _liftTable._buffer)
                        break;
                }

                if (this.SysIsEStop())
                {
                    _liftTable.Stop();
                    Debug.WriteLine("LiftTable: EStop!");
                }

                if (count < 10 && (!drCR15.isEStop() && !drCR7.isEStop()))
                {
                    Debug.WriteLine("Linear scale tried " + count + " attempts to move to the exact position. buffer: " + _liftTable._buffer);
                }

                if (count >= 10)
                {
                    Debug.WriteLine("Linear scale cannot move to the exact position in 10 attempts. buffer: " + _liftTable._buffer);
                }
            }
            else
            {
                Console.WriteLine("Error! Please Check Lift Table Cable Connection.");
            }
        }

        public void LiftTableReCalibrate()
        {
            _liftTable.ClearXYZ();
        }

        private void LiftTableUp()
        {
            _liftTable.Up();
        }

        private void LiftTableStop()
        {
            _liftTable.Stop();
        }

        private void LiftTableDown()
        {
            _liftTable.Down();
        }

        /// <summary>
        /// Get LiftTable movement status.
        /// </summary>
        /// <returns></returns>
        public bool LiftTableIsMoving()
        {
            return _liftTable.Get_IsMoving();
        }

        /// <summary>
        /// Get LiftTable connection status.
        /// </summary>
        /// <returns></returns>
        public bool LiftTableIsConnected()
        {
            return _liftTable.IsConnected();
        }

        /// <summary>
        /// Get LiftTable current height.
        /// </summary>
        /// <returns></returns>
        public double LiftTableGetCurHeight()
        {
            return _liftTable.Get_CurHeight();
        }

        //todo:
        private void LiftTableRelMoveTo(double height)
        {

        }

        // For Ice
        /// <summary>
        /// Get TCP Current Position Based On User Frame.
        /// </summary>
        /// <param name="model"></param>
        /// <returns></returns>
        public double[] GetUFCurPos(Model model)
        {
            double[] cur_pos = new double[6];

            switch (model)
            {
                case Model.CR15:
                {
                    return drCR15.GetCurPos();
                }
                case Model.CR7:
                {
                    return drCR7.GetCurPos();
                }
                default:
                    return cur_pos;
            }
        }

        /// <summary>
        /// Get TCP Current Position Based On Robot Base.
        /// </summary>
        /// <param name="model"></param>
        /// <returns></returns>
        public double[] GetRBCurPos(Model model)
        {
            double[] cur_pos = new double[6];

            switch (model)
            {
                case Model.CR15:
                {
                    return drCR15.GetRBCurPos();
                }
                case Model.CR7:
                {
                    return drCR7.GetRBCurPos();
                }
                default:
                    return cur_pos;
            }
        }

        /// <summary>
        /// 1. Absolute Move Based On Robot Base.
        /// </summary>
        /// <param name="model"></param>
        /// <param name="pos"></param>
        public void RBMove(Model model, double[] pos)
        {
            switch (model)
            {
                case Model.CR15:
                {
                    drCR15.RBMove(pos);
                    break;
                }
                case Model.CR7:
                {
                    drCR7.RBMove(pos);
                    break;
                }
            }
        }

        /// <summary>
        /// 0. Please define TCP, User Frame first.
        /// 1. Absolute Move Based On User Frame. 
        /// </summary>
        /// <param name="model"></param>
        /// <param name="pos"></param>
        public void UFMove(Model model, double[] pos)
        {
            switch (model)
            {
                case Model.CR15:
                {
                    drCR15.UFMove(pos);
                    break;
                }
                case Model.CR7:
                {
                    drCR7.RBMove(pos);
                    break;
                }
            }
        }
        public void MoveTo(Model model, Position pos)
        {
            if (this.SysIsEStop()) return;

            switch (model)
            {
                case Model.CR15:
                    {
                        robotMoveToJoint(drCR15, pos);
                        break;
                    }
                case Model.CR7:
                    {
                        robotMoveToJoint(drCR7, pos);
                        break;
                    }
            }
        }

        private void robotMoveToJoint(DualRobot dr, Position pos)
        {
            if (this.SysIsEStop()) return;

            // get pos info
            var PosArray = dr.GetPosJoint(pos);

            // update config/UF/UT
            // this.SwitchToRobotBase();
            dr.getRBCurPos();

            // assign to pos_in_joint
            var res = dr.setRegisterPosJoint(95, PosArray, dr.RpParamUF, dr.RpParamUT);

            // Console.WriteLine("res: " + res);
            // cout_pos("pos", PosArray);
            // Console.WriteLine("UF: " + this.RpParamUF);
            // Console.WriteLine("UT: " + this.RpParamUT);

            // move to 
            // a. Change to Robot Base
            dr.setRegisterInt(1, 1, 1);
            dr.setRegisterInt(2, 2, 1);
            // b. Move to rb_via_point
            dr.setRegisterInt(1, 7, 1);
            dr.setRegisterInt(2, 2, 1);

            // wait until arrive
            robotWaitForReady(dr);
        }

        private bool robotWaitForReady(DualRobot dr)
        {
            if (this.SysIsEStop()) return false;

            object Register1 = -1;
            object Register2 = -1;

            bool res1, res2;

            // wait for R[1], R[2] = 0
            res1 = dr.getRegisterInt(1, ref Register1);
            res2 = dr.getRegisterInt(2, ref Register2);

            // Console.WriteLine("Register1: " + Register1);
            // Console.WriteLine("Register2: " + Register2);

            while (Register1.ToString() != 0.ToString() || Register2.ToString() != 0.ToString())
            {
                res1 = dr.getRegisterInt(1, ref Register1);
                res2 = dr.getRegisterInt(2, ref Register2);

                if (SysIsEStop()) return false;
            }

            if (res1 && res2)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        private bool robotWaitForMeasurementDone(DualRobot dr)
        {
            if (this.SysIsEStop()) return false;

            object Register = -1;

            bool res;

            // wait for R[1], R[2] = 0
            res = dr.getRegisterInt(102, ref Register);

            // Console.WriteLine("Register1: " + Register1);

            while (Register.ToString() != 3.ToString())
            {
                res = dr.getRegisterInt(102, ref Register);

                if (SysIsEStop()) return false;
            }

            if (res)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        /// <summary>
        /// Get Tcp Distance.
        /// </summary>
        /// <remarks>
        /// Prerequisite: Finish (1) RobotBase Calibration
        /// </remarks>
        /// <returns></returns>

        public double GetTcpDistance(SceneName Scene = SceneName.None, Model model1 = Model.CR7, Model model2 = Model.CR15)
        {
            return cal.GetTcpDistance(Scene, model1, model2, drCR7.GetRBCurPos(), drCR15.GetRBCurPos());
        }

        private void robot_Motion_ViaPoints(SceneName Scene, DualRobot dr, ref Transformation tf, ref MovementType movement_type)
        {
            if (this.SysIsEStop()) return;

            // check List Size
            var via_points_no = tf.via_points.Count;

            bool res; short UF, UT;

            // loop
            robotWaitForReady(dr);

            for (int i = 0; i < via_points_no; i++)
            {
                // Check Scene Status
                dr.GetSceneStatus();
                if (dr.SceneStatus == 2)
                {
                    break;
                }

                // config the next via_point
                UF = UT = 1;

                float[] PosArray = new float[6];
                short[] ConfigArray = new short[6];

                PosArray[0] = float.Parse(tf.via_points[i][0].ToString());
                PosArray[1] = float.Parse(tf.via_points[i][1].ToString());
                PosArray[2] = float.Parse(tf.via_points[i][2].ToString());
                PosArray[3] = float.Parse(tf.via_points[i][3].ToString());
                PosArray[4] = float.Parse(tf.via_points[i][4].ToString());
                PosArray[5] = float.Parse(tf.via_points[i][5].ToString());

                res = dr.setRegisterPosXyzwpr(99, PosArray, dr.config, UF, UT);

                // Move
                dr.setRegisterInt(1, 4, 1);
                dr.setRegisterInt(2, 2, 1);

                // Count
                dr.setRegisterInt(3, i + 1, 1);

                // Update Via Point Location
                dr.via_point_location[0] = tf.via_points[i][6];
                dr.via_point_location[1] = tf.via_points[i][7];
                dr.via_point_location[2] = tf.via_points[i][8];

                // Check Movement Type
                switch (movement_type)
                {
                    case MovementType.QuickCheck:
                        {
                            robotWaitForReady(dr);
                            break;
                        }
                    case MovementType.StepRun:
                        {
                            robotWaitForMeasurementDone(dr);
                            break;
                        }
                }

                var dis = this.GetTcpDistance(Scene, Model.CR15, Model.LiftTable);
                Console.WriteLine("distance = " + dis);
            }

            // Check Scene Status One More
            if (dr.SceneStatus == 2)
            {
                // do nothing. just finish
            }
            else
            {
                dr.SetOrigin();
                dr.MovetoOrigin();
                robotWaitForReady(dr);
            }
        }

        /// <summary>
        /// Init Scene Parameters.
        ///
        /// <para>
        /// For Scene1A:                                     <br/>
        /// param[0]: cr15_scene1a_R                        [160,250]<br/>
        /// param[1]: cr15_scene1a_motion1_arc              [180°]<br/>
        /// param[2]: cr15_scene1a_motion1_step_angle       [0.1°,10°]<br/>
        /// param[3]: motor_scene1a_arc                     [180°]<br/>
        /// param[4]: motor_scene1a_step_angle              [0.1°,10°]<br/>
        /// param[5]: lift_table_scene1a_height            preset value [0,290]<br/>
        /// param[6]: lift_table_scene1a_align_error       preset value<br/>
        /// param[7]: stage34_fixture_height               preset value<br/>
        /// param[8]: antenna_height                        <br/>
        /// </para>
        /// 
        /// <para>
        /// Params for Scene1B:                             <br/>
        /// param[0]: tcp_distance                          <br/>
        /// param[1]: cr15_arc                              (0°,130°]<br/>
        /// param[2]: cr15_step_angle                       [0.1°,10°]<br/>
        /// param[3]: cr7_motion2_arc                       [0°,180°]<br/>
        /// param[4]: cr7_motion2_step_angle                [0.1°,10°]<br/>
        /// param[5]: cr7_motion3_arc                       [0°,180°]<br/>
        /// param[6]: cr7_motion3_step_angle                [0.1°,10°]<br/>
        /// </para>
        ///
        /// <para>
        /// For Scene1C:                                    <br/>
        /// param[0]: AUT / Probe Separation                [0,800]<br/>
        /// param[1]: CR15 span width(Horizontal)           <br/>
        /// param[2]: CR15 span height(Vertical)            <br/>
        /// param[3]: CR15 noOfWStep(no. of points)         <br/>
        /// param[4]: CR15 noOfHStep(no. of points)         <br/>
        /// param[5]: Cr7 Tilt Angle                        <br/>
        /// </para>
        ///
        /// <para>
        /// For Scene2/Scene2_Sim:                                          <br/>
        /// param[0]: Cr15_scene2_R                     [160,250]<br/>
        /// param[1]: Cr15_scene2_motion1_arc          [90°,100°]<br/>
        /// param[2]: Cr15_scene2_motion1_step_angle   [0.1°,10°]<br/>
        /// param[3]: Cr15_scene2_motion2_arc              [180°]<br/>
        /// param[4]: Cr15_scene2_motion2_step_angle   [0.1°,90°]<br/>
        /// param[5]: Cr7_scene2_r                       [20,200]<br/>
        /// param[6]: Cr7_scene2_arc                     [0°,90°]<br/>
        /// param[7]: motor_scene2_angle                [0°,360°]<br/>
        /// param[8]: lift_table_scene2_height            [0,290]<br/>
        /// param[9]: lift_table_align_error           preset value<br/>
        /// param[10]: stage34_fixture_height          preset value<br/>
        /// </para>
        /// 
        /// <para>
        /// For Scene3:                                    <br/>
        /// param[0]: Cr15_scene3_R                     [160,250]<br/>
        /// param[1]: CR15 span width(Horizontal)           <br/>
        /// param[2]: CR15 span height(Vertical)            <br/>
        /// param[3]: CR15 noOfWStep(no. of points)         <br/>
        /// param[4]: CR15 noOfHStep(no. of points)         <br/>
        /// param[5]: Cr7 Tilt Angle                        <br/>
        /// param[5]: Cr7_scene3_r                       [20,200]<br/>
        /// param[6]: Cr7_scene3_arc                     [0°,90°]<br/>
        /// param[7]: motor_scene2_angle                [0°,360°]<br/>
        /// param[8]: lift_table_scene2_height            [0,290]<br/>
        /// param[9]: lift_table_align_error           preset value<br/>
        /// param[10]: stage34_fixture_height          preset value<br/>        
        /// </para>
        /// <para>
        /// For Scene4:                                     <br/>
        /// param[0]: distance between lift table TCP to robot antenna TCP e.g.0<br/>                  
        /// param[1]: length                     e.g 200   <br/>
        /// param[2]: no. of points(length)      e.g 5   <br/>
        /// param[3]: width                      e.g 200   <br/>
        /// param[4]: no. of points(width)       e.g 5   <br/>
        /// param[5]: height                     e.g 300   <br/>
        /// param[5]: no. of points(height)      e.g 5   <br/>
        /// </para>
        /// </summary>
        /// <param name="scene_name"></param>
        /// <param name="param"></param>
        public void SceneParamInit(SceneName scene_name, double[] param)
        {
            switch (scene_name)
            {
                case SceneName.Scene1A:
                    {
                        // basic info
                        scene1a.ParamsInit(scene_name, param);
                        break;
                    }
                case SceneName.Scene1A_Sim:
                    {
                        // basic info
                        scene1a.ParamsInit(scene_name, param);
                        break;
                    }
                case SceneName.Scene1B:
                    {
                        // basic info
                        scene1b.ParamsInit(scene_name, param);
                        break;
                    }
                case SceneName.Scene1C:
                    {
                        // basic info
                        scene1c.ParamsInit(scene_name, param);
                        break;
                    }
                case SceneName.Scene2:
                    {
                        // basic info
                        scene2.ParamsInit(scene_name, param);
                        break;
                    }
                case SceneName.Scene2_Sim:
                    {
                        // basic info
                        scene2.ParamsInit(scene_name, param);
                        break;
                    }
                case SceneName.Scene3:
                    {
                        // basic info
                        scene3.ParamsInit(scene_name, param);
                        break;
                    }
                case SceneName.Scene3_Sim:
                    {
                        // basic info
                        scene3.ParamsInit(scene_name, param);
                        break;
                    }
                case SceneName.Scene4:
                {
                    // basic info
                    scene4.ParamsInit(scene_name, param);
                    break;
                }
                case SceneName.Scene4_Sim:
                {
                    // basic info
                    scene4.ParamsInit(scene_name, param);
                    break;
                }
            }
        }

        /// <summary>
        /// Robots will move to init position.
        /// </summary>
        /// <param name="scene_name"></param>
        public void SceneRobotInit(SceneName scene_name, Model model = Model.Null, MovementStage stage = MovementStage.Null)
        {
            if (model == Model.Null)
            {
                switch (scene_name)
                {
                    case SceneName.Scene1A:
                        {
                            // 1. cr7/cr15 moves to home position
                            this.MoveTo(Model.CR7, Position.Ready_Scene1A);
                            this.MoveTo(Model.CR15, Position.Home2);

                            // 2. Motor Rotation
                            MotorAbsMoveTo(_motor.Get_homePos());

                            // 3. LiftTable lifts Up
                            LiftTableAbsMoveTo(scene1a.lift_table_scene1a_height - scene1a.antenna_height);

                            // 4. Get Current LiftTable Height
                            var cur_height = LiftTableGetCurHeight();
                            //var diff = cur_height - scene1a.lift_table_scene1a_align_error;
                            // var diff = cur_height - scene1a.lift_table_scene1a_height + scene1a.antenna_height - scene1a.lift_table_scene1a_align_error;
                            var diff = cur_height - scene1a.lift_table_scene1a_height - scene1a.lift_table_scene1a_align_error;

                            // 5. Update lt_tcp.
                            var T_diff = cal.Set_T_diff(ref diff);
                            cal.T_Cr7_PlateCenter = cal.T_Cr7_PlateCenter * T_diff;

                            // 6. get cr15 Init Position (re-calibrate both).
                            scene1a.GetScene1AInitPositionXyzwpr(ref cal);

                            // (2)
                            // set cr15 init position
                            scene1a.SetCr15InitPositionXyzwpr(ref this.drCR15, Position.InitPos_Scene2);

                            // cr15 moves to home position
                            this.MoveTo(Model.CR15, Position.Home2);

                            // cr15 moves to init position
                            this.MoveTo(Model.CR15, Position.InitPos_Scene2);

                            break;
                        }
                    case SceneName.Scene1A_Sim:
                        {
                            // 1. cr7/cr15 moves to home position
                            this.MoveTo(Model.CR7, Position.Ready_Scene1A);
                            this.MoveTo(Model.CR15, Position.Home2);

                            // get cr7 and cr15 Init Position.
                            scene1a.GetScene1ASimCr15InitPositionXyzwpr(ref cal);

                            // (2) 
                            // set cr15 init position
                            scene1a.SetCr15InitPositionXyzwpr(ref this.drCR15, Position.InitPos_Scene2);

                            // cr15 moves to home position
                            this.MoveTo(Model.CR15, Position.Home2);

                            // cr15 moves to init position
                            this.MoveTo(Model.CR15, Position.InitPos_Scene2);

                            break;
                        }
                    case SceneName.Scene1B:
                        {
                            // // switch to robot base
                            // drCR7.SwitchToRobotBase();
                            // drCR15.SwitchToRobotBase();

                            // get cr15 init position
                            scene1b.GetCr15InitPositionXyzwpr(ref this.drCR7, ref this.drCR15, ref this.cal, scene_name);
                            // set cr15 init position
                            scene1b.SetCr15InitPositionXyzwpr(ref this.drCR15, Position.InitPos_Scene1B);

                            // cr7/cr15 moves to home position
                            this.MoveTo(Model.CR7, Position.Home);
                            this.MoveTo(Model.CR15, Position.Home);

                            // cr7/cr15 moves to init position
                            this.MoveTo(Model.CR7, Position.InitPos_Scene1B);
                            this.MoveTo(Model.CR15, Position.InitPos_Scene1B);
                            break;
                        }
                    case SceneName.Scene1C:
                        {
                            // get cr15 init position
                            scene1c.GetCr15InitPositionXyzwpr(ref this.drCR7, ref this.drCR15, ref this.cal, scene_name);
                            // set cr15 init position
                            scene1c.SetCr15InitPositionXyzwpr(ref this.drCR15, Position.InitPos_Scene1C);

                            // cr7/cr15 moves to home position
                            this.MoveTo(Model.CR7, Position.Home);
                            this.MoveTo(Model.CR15, Position.Home);
                            this.MoveTo(Model.CR15, Position.InitPos_Scene1C);

                            // cr7/cr15 moves to init position
                            this.MoveTo(Model.CR7, Position.InitPos_Scene1C);
                            
                            break;
                        }
                    case SceneName.Scene2_Sim:
                        {
                            // for stage 1&2
                            if (stage != MovementStage.Three && stage != MovementStage.Four)
                            {
                                // 1. cr7/cr15 moves to home position
                                this.MoveTo(Model.CR7, Position.Home2);
                                this.MoveTo(Model.CR15, Position.Home2);

                                // get cr7 and cr15 Init Position.
                                scene2.GetScene2SimInitPositionXyzwpr(ref cal);

                                // (2) 
                                // set cr7 init position
                                scene2.SetCr7InitPositionXyzwpr(ref this.drCR7, Position.InitPos_Scene2);
                                // set cr15 init position
                                scene2.SetCr15InitPositionXyzwpr(ref this.drCR15, Position.InitPos_Scene2);

                                // cr7/cr15 moves to home position
                                this.MoveTo(Model.CR7, Position.Home2);
                                this.MoveTo(Model.CR15, Position.Home2);

                                // cr7/cr15 moves to init position
                                this.MoveTo(Model.CR7, Position.InitPos_Scene2);
                                this.MoveTo(Model.CR15, Position.InitPos_Scene2);
                            }

                            // for stage 3&4
                            if (stage == MovementStage.Three || stage == MovementStage.Four)
                            {
                                // 1. cr7/cr15 moves to home position
                                this.MoveTo(Model.CR7, Position.Ready_Scene1A);
                                this.MoveTo(Model.CR15, Position.Home2);

                                // get cr7 and cr15 Init Position.
                                scene2.GetScene2SimInitPositionXyzwpr(ref cal);

                                // (2) 
                                // set cr7 init position
                                scene2.SetCr7InitPositionXyzwpr(ref this.drCR7, Position.InitPos_Scene2);
                                // set cr15 init position
                                scene2.SetCr15InitPositionXyzwpr(ref this.drCR15, Position.InitPos_Scene2);

                                // cr15 moves to home position
                                this.MoveTo(Model.CR15, Position.Home2);

                                // cr15 moves to init position
                                this.MoveTo(Model.CR15, Position.InitPos_Scene2);
                            }

                            break;
                        }
                    case SceneName.Scene2:
                        {
                            // Tricks: because of the lift table will move after robot moves to init position. so:
                            // 1. we need to update lift table tcp.

                            // for stage 1&2
                            if (stage != MovementStage.Three && stage != MovementStage.Four)
                            {
                                // 1. cr7/cr15 moves to home position
                                this.MoveTo(Model.CR7, Position.Home2);
                                this.MoveTo(Model.CR15, Position.Home2);

                                // 2. Motor Rotation
                                MotorAbsMoveTo(_motor.Get_homePos());
                                // MotorAbsMoveTo(scene2.motor_scene2_angle);

                                // 3. LiftTable lifts Up
                                LiftTableAbsMoveTo(scene2.lift_table_scene2_height);

                                // 4. Get Current LiftTable Height
                                var cur_height = LiftTableGetCurHeight();
                                var diff = cur_height - scene2.lift_table_scene2_height - scene2.lift_table_scene2_align_error;

                                // 5. Update lt_tcp.
                                var T_diff = cal.Set_T_diff(ref diff);
                                cal.T_Cr7_PlateCenter = cal.T_Cr7_PlateCenter * T_diff;

                                // 6. Get cr7 and cr15 Init Position (re-calibrate both).
                                scene2.GetScene2InitPositionXyzwpr(ref cal);

                                // (2)
                                // set cr7 init position
                                scene2.SetCr7InitPositionXyzwpr(ref this.drCR7, Position.InitPos_Scene2);
                                // set cr15 init position
                                scene2.SetCr15InitPositionXyzwpr(ref this.drCR15, Position.InitPos_Scene2);

                                // cr7/cr15 moves to home position
                                this.MoveTo(Model.CR7, Position.Home2);
                                this.MoveTo(Model.CR15, Position.Home2);

                                // cr7/cr15 moves to init position
                                this.MoveTo(Model.CR7, Position.InitPos_Scene2);
                                this.MoveTo(Model.CR15, Position.InitPos_Scene2);
                            }

                            // for stage 3&4
                            if (stage == MovementStage.Three || stage == MovementStage.Four)
                            {
                                // 1. cr7 cr15 moves to init position
                                this.MoveTo(Model.CR7, Position.Ready_Scene1A);
                                this.MoveTo(Model.CR15, Position.Home2);

                                // 2. Motor Rotation
                                MotorAbsMoveTo(scene2.motor_scene2_angle);

                                // 3. LiftTable lifts Up
                                LiftTableAbsMoveTo(scene2.lift_table_scene2_height);

                                // 4. Get Current LiftTable Height
                                var cur_height = LiftTableGetCurHeight();
                                var diff = cur_height - scene2.lift_table_scene2_height - scene2.lift_table_scene2_align_error;

                                // diff = diff + scene2.stage34_fixture_height;

                                // 5. Update lt_tcp.
                                var T_diff = cal.Set_T_diff(ref diff);
                                cal.T_Cr7_PlateCenter = cal.T_Cr7_PlateCenter * T_diff;

                                // 6. Get cr7 and cr15 Init Position (re-calibrate both).
                                scene2.GetScene2InitPositionXyzwpr(ref cal);

                                // (2)
                                // set cr7 init position
                                scene2.SetCr7InitPositionXyzwpr(ref this.drCR7, Position.InitPos_Scene2);
                                // set cr15 init position
                                scene2.SetCr15InitPositionXyzwpr(ref this.drCR15, Position.InitPos_Scene2);

                                // cr15 moves to home position
                                this.MoveTo(Model.CR15, Position.Home2);

                                // cr15 moves to init position
                                this.MoveTo(Model.CR15, Position.InitPos_Scene2);
                            }

                            break;
                        }
                    case SceneName.Scene3_Sim:
                    {
                        // // 1. cr7/cr15 moves to home position
                        // this.MoveTo(Model.CR7, Position.Home3);
                        // this.MoveTo(Model.CR15, Position.Home3);

                        // get cr7 and cr15 Init Position.
                        scene3.GetScene3SimInitPositionXyzwpr(ref cal);

                        // (2) 
                        // set cr7 init position
                        scene3.SetCr7InitPositionXyzwpr(ref this.drCR7, Position.InitPos_Scene3);
                        // set cr15 init position
                        scene3.SetCr15InitPositionXyzwpr(ref this.drCR15, Position.InitPos_Scene3);

                        // cr7/cr15 moves to home position
                        this.MoveTo(Model.CR7, Position.Home3);
                        this.MoveTo(Model.CR15, Position.Home3);

                        // cr7/cr15 moves to init position
                        this.MoveTo(Model.CR7, Position.InitPos_Scene3);
                        this.MoveTo(Model.CR15, Position.InitPos_Scene3);

                        break;
                    }
                    case SceneName.Scene3:
                    {
                        // get cr15 init position
                        scene1c.GetCr15InitPositionXyzwpr(ref this.drCR7, ref this.drCR15, ref this.cal, scene_name);
                        // set cr15 init position
                        scene1c.SetCr15InitPositionXyzwpr(ref this.drCR15, Position.InitPos_Scene1C);

                        // cr7/cr15 moves to home position
                        this.MoveTo(Model.CR7, Position.Home);
                        this.MoveTo(Model.CR15, Position.Home);
                        this.MoveTo(Model.CR15, Position.InitPos_Scene1C);

                        // cr7/cr15 moves to init position
                        this.MoveTo(Model.CR7, Position.InitPos_Scene1C);

                        break;
                    }
                    case SceneName.Scene4_Sim:
                        {
                            // // 1. cr7/cr15 moves to home position
                            // this.MoveTo(Model.CR7, Position.Home3);
                            // this.MoveTo(Model.CR15, Position.Home3);
                            this.MoveTo(Model.CR7, Position.Ready_Scene1A);

                            // calculate cr15 Init Position.
                            scene4.GetInitPositionXyzwpr(Env.Simulation, SceneName.Scene4_Sim, ref cal);
                            // set cr15 init position
                            scene4.SetCr15InitPositionXyzwpr(ref this.drCR15, Position.InitPos_Scene3);

                            // cr7/cr15 moves to home position
                            //this.MoveTo(Model.CR7, Position.Home3);
                            this.MoveTo(Model.CR15, Position.Home2);

                            // cr7/cr15 moves to init position
                            //this.MoveTo(Model.CR7, Position.InitPos_Scene3);
                            this.MoveTo(Model.CR15, Position.InitPos_Scene3);

                            break;
                        }
                    case SceneName.Scene4:
                        {
                            // // 1. cr7/cr15 moves to home position
                            // this.MoveTo(Model.CR7, Position.Home3);
                            this.MoveTo(Model.CR7, Position.Ready_Scene1A);
                            // this.MoveTo(Model.CR15, Position.Home3);

                            // calculate cr15 Init Position.
                            scene4.GetInitPositionXyzwpr(Env.Real, SceneName.Scene4, ref cal);
                            // set cr15 init position
                            scene4.SetCr15InitPositionXyzwpr(ref this.drCR15, Position.InitPos_Scene3);
                            
                            // cr7/cr15 moves to home position
                            //this.MoveTo(Model.CR7, Position.Home3);
                            this.MoveTo(Model.CR15, Position.Home2);
                            
                            // cr7/cr15 moves to init position
                            //this.MoveTo(Model.CR7, Position.InitPos_Scene3);
                            this.MoveTo(Model.CR15, Position.InitPos_Scene3);

                            break;
                        }
                }
            }

            if (model == Model.CR7)
            {
                switch (scene_name)
                {
                    case SceneName.Scene2_Sim:
                        {
                            // get cr7 and cr15 Init Position.
                            scene2.GetScene2SimInitPositionXyzwpr(ref cal);

                            // (2) 
                            // set cr7 init position
                            scene2.SetCr7InitPositionXyzwpr(ref this.drCR7, Position.InitPos_Scene2);

                            // cr7/cr15 moves to home position
                            this.MoveTo(Model.CR7, Position.Home2);

                            // cr7 moves to init position
                            this.MoveTo(Model.CR7, Position.InitPos_Scene2);

                            break;
                        }
                    case SceneName.Scene2:
                        {
                            //(1)
                            // 1. Get Current LiftTable Height
                            var cur_height = LiftTableGetCurHeight();
                            var diff = cur_height - scene2.lift_table_scene2_height - scene2.lift_table_scene2_align_error;

                            // 5. Update lt_tcp.
                            var T_diff = cal.Set_T_diff(ref diff);
                            cal.T_Cr7_PlateCenter = cal.T_Cr7_PlateCenter * T_diff;

                            // 6. Get cr7 and cr15 Init Position (re-calibrate both).
                            scene2.GetScene2InitPositionXyzwpr(ref cal);

                            // (2)
                            // set cr7 init position
                            scene2.SetCr7InitPositionXyzwpr(ref this.drCR7, Position.InitPos_Scene2);

                            // cr7/cr15 moves to home position
                            this.MoveTo(Model.CR7, Position.Home2);

                            // cr7/cr15 moves to init position
                            this.MoveTo(Model.CR7, Position.InitPos_Scene2);

                            break;
                        }
                }
            }
        }
        

        // Scene Simulation & Real (1A/1C/2/3/4)
        public void Scene1C(MovementType movement_type, MovementStage movement_stage = MovementStage.Null)
        {
            if(SysIsEStop()) return;

            // Scene Start
            this.SetSceneStart();

            // CR7 Path -- motion3
            tfCR7.via_orbit_points_part2 = tfCR7.get_1C_points_Cr7(scene1c.tilt_angle);

            var via_orbit_points_part2_no = tfCR7.via_orbit_points_part2.Count;

            // loop -- part 2
            for (int i = 0; i < via_orbit_points_part2_no; i++)
            {
                // Check Scene Status
                SceneStatus_Cr7 = this.GetSceneStatus(Model.CR7);
                SceneStatus_Cr15 = this.GetSceneStatus(Model.CR15);
                if (SceneStatus_Cr7 == 2 || SceneStatus_Cr15 == 2)
                    break;

                // move to via_point
                drCR7.Motion_aPoint(tfCR7.via_orbit_points_part2[i]);

                robotWaitForReady(drCR7);
            }

            /// CR15
            switch (movement_stage)
            {
                case MovementStage.One:
                {
                    // 1. CR15 Path
                    tfCR15.via_points = tfCR15.get_1C_points_Cr15_motion1_test(scene1c.span_width, scene1c.span_height,
                        scene1c.noOfWStep, scene1c.noOfHStep);

                    robot_Motion_ViaPoints(SceneName.None, drCR15, ref tfCR15, ref movement_type);

                    break;
                }
                case MovementStage.Two:
                {
                    tfCR15.via_points = tfCR15.get_1C_points_Cr15_motion2_test(scene1c.span_width, scene1c.span_height,
                        scene1c.noOfWStep, scene1c.noOfHStep);

                    robot_Motion_ViaPoints(SceneName.None, drCR15, ref tfCR15, ref movement_type);

                    break;
                }
            }

            // Check whether the system is estop. if yes. return.
            if (this.SysIsEStop()) return;

            // Scene End
            this.SetSceneFinish();
        }
        public void Scene2(MovementType movement_type, MovementStage movement_stage = MovementStage.Null)
        {
            if (SysIsEStop()) return;

            // Scene Start
            this.SetSceneStart();

            // 1. CR15 Path -- motion2
            tfCR15.motion2_points = tfCR15.get_2_points_Cr15_motion2(
                scene2.cr15_scene2_motion2_arc, scene2.cr15_scene2_motion2_step_angle);

            var motion2_points_no = tfCR15.motion2_points.Count;

            switch (movement_stage)
            {
                case MovementStage.One:
                    {
                        // loop -- CR15 motion2 --> +-90
                        for (int i = 0; i < motion2_points_no; i++)
                        {
                            if (this.SysIsEStop()) return;

                            // Check Scene Status
                            SceneStatus_Cr7 = this.GetSceneStatus(Model.CR7);
                            SceneStatus_Cr15 = this.GetSceneStatus(Model.CR15);
                            if (SceneStatus_Cr7 == 2 || SceneStatus_Cr15 == 2)
                            {
                                break;
                            }

                            drCR15.SwitchToUserFrame();

                            drCR15.Motion_aPoint(tfCR15.motion2_points[i]);

                            robotWaitForReady(drCR15);

                            drCR15.SetUserFrameTemp();

                            tfCR15.via_points = tfCR15.get_2_points_Cr15_motion1a(scene2.cr15_scene2_R, scene2.cr15_scene2_motion1_arc, scene2.cr15_scene2_motion1_step_angle, i + 1);

                            robot_Motion_ViaPoints(SceneName.None, drCR15, ref tfCR15, ref movement_type);

                            drCR15.SwitchToUserFrame();

                            robotWaitForReady(drCR15);

                            // Check Scene Status One More Time
                            if (SceneStatus_Cr7 == 2 || SceneStatus_Cr15 == 2)
                            {
                                // Do Nothing
                            }
                            else
                            {
                                drCR15.SetOrigin();
                                drCR15.MovetoOrigin();
                                robotWaitForReady(drCR15);
                            }
                        }
                        break;
                    }
                case MovementStage.Two:
                    {
                        // loop2 -- CR15 motion2 --> +-90
                        for (int i = 0; i < motion2_points_no; i++)
                        {
                            if (this.SysIsEStop()) return;

                            // Check Scene Status
                            SceneStatus_Cr7 = this.GetSceneStatus(Model.CR7);
                            SceneStatus_Cr15 = this.GetSceneStatus(Model.CR15);
                            if (SceneStatus_Cr7 == 2 || SceneStatus_Cr15 == 2)
                            {
                                break;
                            }

                            drCR15.SwitchToUserFrame();

                            drCR15.Motion_aPoint(tfCR15.motion2_points[i]);

                            robotWaitForReady(drCR15);

                            drCR15.SetUserFrameTemp();

                            tfCR15.via_points = tfCR15.get_2_points_Cr15_motion1b(scene2.cr15_scene2_R, scene2.cr15_scene2_motion1_arc, scene2.cr15_scene2_motion1_step_angle, i + 1);

                            robot_Motion_ViaPoints(SceneName.None, drCR15, ref tfCR15, ref movement_type);

                            drCR15.SwitchToUserFrame();

                            robotWaitForReady(drCR15);

                            // Check Scene Status One More Time
                            if (SceneStatus_Cr7 == 2 || SceneStatus_Cr15 == 2)
                            {
                                // Do Nothing
                            }
                            else
                            {
                                drCR15.SetOrigin();
                                drCR15.MovetoOrigin();
                                robotWaitForReady(drCR15);
                            }
                        }
                        break;
                    }
                case MovementStage.Three:
                    {
                        // loop -- CR15 motion2 --> +-90
                        for (int i = 0; i < motion2_points_no; i++)
                        {
                            if (this.SysIsEStop()) return;

                            // Check Scene Status
                            SceneStatus_Cr7 = this.GetSceneStatus(Model.CR7);
                            SceneStatus_Cr15 = this.GetSceneStatus(Model.CR15);
                            if (SceneStatus_Cr7 == 2 || SceneStatus_Cr15 == 2)
                            {
                                break;
                            }

                            drCR15.SwitchToUserFrame();

                            drCR15.Motion_aPoint(tfCR15.motion2_points[i]);

                            robotWaitForReady(drCR15);

                            drCR15.SetUserFrameTemp();

                            tfCR15.via_points = tfCR15.get_2_points_Cr15_motion1c(scene2.cr15_scene2_R, scene2.cr15_scene2_motion1_arc, scene2.cr15_scene2_motion1_step_angle, i + 1);

                            robot_Motion_ViaPoints(SceneName.None, drCR15, ref tfCR15, ref movement_type);

                            drCR15.SwitchToUserFrame();

                            robotWaitForReady(drCR15);

                            // Check Scene Status One More Time
                            if (SceneStatus_Cr7 == 2 || SceneStatus_Cr15 == 2)
                            {
                                // Do Nothing
                            }
                            else
                            {
                                drCR15.SetOrigin();
                                drCR15.MovetoOrigin();
                                robotWaitForReady(drCR15);
                            }
                        }
                        break;
                    }
                case MovementStage.Four:
                    {
                        // loop2 -- CR15 motion2 --> +-90
                        for (int i = 0; i < motion2_points_no; i++)
                        {
                            if (this.SysIsEStop()) return;

                            // Check Scene Status
                            SceneStatus_Cr7 = this.GetSceneStatus(Model.CR7);
                            SceneStatus_Cr15 = this.GetSceneStatus(Model.CR15);
                            if (SceneStatus_Cr7 == 2 || SceneStatus_Cr15 == 2)
                            {
                                break;
                            }

                            drCR15.SwitchToUserFrame();

                            drCR15.Motion_aPoint(tfCR15.motion2_points[i]);

                            robotWaitForReady(drCR15);

                            drCR15.SetUserFrameTemp();

                            tfCR15.via_points = tfCR15.get_2_points_Cr15_motion1d(scene2.cr15_scene2_R, scene2.cr15_scene2_motion1_arc, scene2.cr15_scene2_motion1_step_angle, i + 1);

                            robot_Motion_ViaPoints(SceneName.None, drCR15, ref tfCR15, ref movement_type);

                            drCR15.SwitchToUserFrame();

                            robotWaitForReady(drCR15);

                            // Check Scene Status One More Time
                            if (SceneStatus_Cr7 == 2 || SceneStatus_Cr15 == 2)
                            {
                                // Do Nothing
                            }
                            else
                            {
                                drCR15.SetOrigin();
                                drCR15.MovetoOrigin();
                                robotWaitForReady(drCR15);
                            }
                        }
                        break;
                    }
            }

            // Check whether the system is estop. if yes. return.
            if (this.SysIsEStop()) return;

            // 2. LiftTable lifts Return Zero
            LiftTableAbsMoveTo(0);

            // 3. Motor Rotation Return Zero
            MotorAbsMoveTo(_motor.Get_homePos());

            this.MoveTo(Model.CR7, Position.Home2);
            robotWaitForReady(drCR7);

            this.MoveTo(Model.CR15, Position.Home2);
            robotWaitForReady(drCR15);

            // Scene End
            this.SetSceneFinish();
        }
        public void Scene2_Sim(MovementType movement_type, MovementStage movement_stage = MovementStage.Null)
        {
            if (SysIsEStop()) return;

            // Scene Start
            this.SetSceneStart();

            // 2. CR15 Path -- motion2
            tfCR15.motion2_points = tfCR15.get_2_points_Cr15_motion2(
                scene2.cr15_scene2_motion2_arc, scene2.cr15_scene2_motion2_step_angle);

            var motion2_points_no = tfCR15.motion2_points.Count;

            switch (movement_stage)
            {
                case MovementStage.One:
                    {
                        // loop -- CR15 motion2 --> +-90
                        for (int i = 0; i < motion2_points_no; i++)
                        {
                            if (this.SysIsEStop()) return;

                            // Check Scene Status
                            SceneStatus_Cr7 = this.GetSceneStatus(Model.CR7);
                            SceneStatus_Cr15 = this.GetSceneStatus(Model.CR15);
                            if (SceneStatus_Cr7 == 2 || SceneStatus_Cr15 == 2)
                            {
                                break;
                            }

                            drCR15.SwitchToUserFrame();

                            drCR15.Motion_aPoint(tfCR15.motion2_points[i]);

                            robotWaitForReady(drCR15);

                            drCR15.SetUserFrameTemp();

                            tfCR15.via_points = tfCR15.get_2_points_Cr15_motion1a(scene2.cr15_scene2_R, scene2.cr15_scene2_motion1_arc, scene2.cr15_scene2_motion1_step_angle, i + 1);
                            
                            robot_Motion_ViaPoints(SceneName.None, drCR15, ref tfCR15, ref movement_type);

                            drCR15.SwitchToUserFrame();

                            robotWaitForReady(drCR15);

                            // Check Scene Status One More Time
                            if (SceneStatus_Cr7 == 2 || SceneStatus_Cr15 == 2)
                            {
                                // Do Nothing
                            }
                            else
                            {
                                drCR15.SetOrigin();
                                drCR15.MovetoOrigin();
                                robotWaitForReady(drCR15);
                            }
                        }
                        break;
                    }
                case MovementStage.Two:
                    {
                        // loop2 -- CR15 motion2 --> +-90
                        for (int i = 0; i < motion2_points_no; i++)
                        {
                            if (this.SysIsEStop()) return;

                            // Check Scene Status
                            SceneStatus_Cr7 = this.GetSceneStatus(Model.CR7);
                            SceneStatus_Cr15 = this.GetSceneStatus(Model.CR15);
                            if (SceneStatus_Cr7 == 2 || SceneStatus_Cr15 == 2)
                            {
                                break;
                            }

                            drCR15.SwitchToUserFrame();

                            drCR15.Motion_aPoint(tfCR15.motion2_points[i]);

                            robotWaitForReady(drCR15);

                            drCR15.SetUserFrameTemp();

                            tfCR15.via_points = tfCR15.get_2_points_Cr15_motion1b(scene2.cr15_scene2_R, scene2.cr15_scene2_motion1_arc, scene2.cr15_scene2_motion1_step_angle, i + 1);
                            
                            robot_Motion_ViaPoints(SceneName.None, drCR15, ref tfCR15, ref movement_type);

                            drCR15.SwitchToUserFrame();

                            robotWaitForReady(drCR15);

                            // Check Scene Status One More Time
                            if (SceneStatus_Cr7 == 2 || SceneStatus_Cr15 == 2)
                            {
                                // Do Nothing
                            }
                            else
                            {
                                drCR15.SetOrigin();
                                drCR15.MovetoOrigin();
                                robotWaitForReady(drCR15);
                            }
                        }
                        break;
                    }
                case MovementStage.Three:
                    {
                        // loop -- CR15 motion2 --> +-90
                        for (int i = 0; i < motion2_points_no; i++)
                        {
                            if (this.SysIsEStop()) return;

                            // Check Scene Status
                            SceneStatus_Cr7 = this.GetSceneStatus(Model.CR7);
                            SceneStatus_Cr15 = this.GetSceneStatus(Model.CR15);
                            if (SceneStatus_Cr7 == 2 || SceneStatus_Cr15 == 2)
                            {
                                break;
                            }

                            drCR15.SwitchToUserFrame();

                            drCR15.Motion_aPoint(tfCR15.motion2_points[i]);

                            robotWaitForReady(drCR15);

                            drCR15.SetUserFrameTemp();

                            tfCR15.via_points = tfCR15.get_2_points_Cr15_motion1c(scene2.cr15_scene2_R, scene2.cr15_scene2_motion1_arc, scene2.cr15_scene2_motion1_step_angle, i + 1);

                            robot_Motion_ViaPoints(SceneName.None, drCR15, ref tfCR15, ref movement_type);

                            drCR15.SwitchToUserFrame();

                            robotWaitForReady(drCR15);

                            // Check Scene Status One More Time
                            if (SceneStatus_Cr7 == 2 || SceneStatus_Cr15 == 2)
                            {
                                // Do Nothing
                            }
                            else
                            {
                                drCR15.SetOrigin();
                                drCR15.MovetoOrigin();
                                robotWaitForReady(drCR15);
                            }
                        }
                        break;
                    }
                case MovementStage.Four:
                    {
                        // loop2 -- CR15 motion2 --> +-90
                        for (int i = 0; i < motion2_points_no; i++)
                        {
                            if (this.SysIsEStop()) return;

                            // Check Scene Status
                            SceneStatus_Cr7 = this.GetSceneStatus(Model.CR7);
                            SceneStatus_Cr15 = this.GetSceneStatus(Model.CR15);
                            if (SceneStatus_Cr7 == 2 || SceneStatus_Cr15 == 2)
                            {
                                break;
                            }

                            drCR15.SwitchToUserFrame();

                            drCR15.Motion_aPoint(tfCR15.motion2_points[i]);

                            robotWaitForReady(drCR15);

                            drCR15.SetUserFrameTemp();

                            tfCR15.via_points = tfCR15.get_2_points_Cr15_motion1d(scene2.cr15_scene2_R, scene2.cr15_scene2_motion1_arc, scene2.cr15_scene2_motion1_step_angle, i + 1);

                            robot_Motion_ViaPoints(SceneName.None, drCR15, ref tfCR15, ref movement_type);

                            drCR15.SwitchToUserFrame();

                            robotWaitForReady(drCR15);

                            // Check Scene Status One More Time
                            if (SceneStatus_Cr7 == 2 || SceneStatus_Cr15 == 2)
                            {
                                // Do Nothing
                            }
                            else
                            {
                                drCR15.SetOrigin();
                                drCR15.MovetoOrigin();
                                robotWaitForReady(drCR15);
                            }
                        }
                        break;
                    }
            }

            if (this.SysIsEStop()) return;

            this.MoveTo(Model.CR7, Position.Home2);
            robotWaitForReady(drCR7);

            this.MoveTo(Model.CR15, Position.Home2);
            robotWaitForReady(drCR15);

            // Scene End
            this.SetSceneFinish();
        }
        public void Scene1A(MovementType movement_type, MovementStage movement_stage = MovementStage.Null)
        {
            if (SysIsEStop()) return;

            // Scene Start
            this.SetSceneStart();

            // 2. Motor viaPath 
            tfMotor.motor_points = tfMotor.get_1a_points_motor_motion(
                scene1a.motor_scene1a_arc, scene1a.motor_scene1a_step_angle);

            var motor_points_no = tfMotor.motor_points.Count;

            switch (movement_stage)
            {
                case MovementStage.One:
                    {
                        // loop -- motor points
                        for (int i = 0; i < motor_points_no; i++)
                        {
                            if (this.SysIsEStop()) return;

                            // 1. motor
                            // MotorAbsMoveTo(tfMotor.motor_points[i]);
                            // MotorAbsMoveTo(-tfMotor.motor_points[i]);
                            if (i == 0)
                            {
                                MotorAbsMoveTo(_motor.Get_initPos());
                            }
                            else
                            {
                                MotorRelMoveTo(scene1a.motor_scene1a_step_angle);
                            }

                            // 2. CR15
                            // Check Scene Status
                            SceneStatus_Cr7 = this.GetSceneStatus(Model.CR7);
                            SceneStatus_Cr15 = this.GetSceneStatus(Model.CR15);
                            if (SceneStatus_Cr7 == 2 || SceneStatus_Cr15 == 2)
                            {
                                break;
                            }

                            drCR15.SwitchToUserFrame();

                            robotWaitForReady(drCR15);

                            drCR15.SetUserFrameTemp();

                            tfCR15.via_points = tfCR15.get_1a_points_Cr15_motion1a(scene1a.cr15_scene1a_R, scene1a.cr15_scene1a_motion1_arc, scene1a.cr15_scene1a_motion1_step_angle, i + 1);

                            robot_Motion_ViaPoints(SceneName.None, drCR15, ref tfCR15, ref movement_type);

                            drCR15.SwitchToUserFrame();

                            robotWaitForReady(drCR15);

                            // Check Scene Status One More Time
                            if (SceneStatus_Cr7 == 2 || SceneStatus_Cr15 == 2)
                            {
                                // Do Nothing
                            }
                            else
                            {
                                drCR15.SetOrigin();
                                drCR15.MovetoOrigin();
                                robotWaitForReady(drCR15);
                            }
                        }

                        break;
                    }
                case MovementStage.Two:
                    {
                        // loop -- motor points
                        for (int i = 0; i < motor_points_no; i++)
                        {
                            if (this.SysIsEStop()) return;

                            // 1. motor
                            // MotorAbsMoveTo(tfMotor.motor_points[i]);
                            // MotorAbsMoveTo(-tfMotor.motor_points[i]);
                            if (i == 0)
                            {
                                MotorAbsMoveTo(_motor.Get_initPos());
                            }
                            else
                            {
                                MotorRelMoveTo(scene1a.motor_scene1a_step_angle);
                            }

                            // 2. CR15
                            // Check Scene Status
                            SceneStatus_Cr7 = this.GetSceneStatus(Model.CR7);
                            SceneStatus_Cr15 = this.GetSceneStatus(Model.CR15);
                            if (SceneStatus_Cr7 == 2 || SceneStatus_Cr15 == 2)
                            {
                                break;
                            }

                            drCR15.SwitchToUserFrame();

                            robotWaitForReady(drCR15);

                            drCR15.SetUserFrameTemp();

                            tfCR15.via_points = tfCR15.get_1a_points_Cr15_motion1b(scene1a.cr15_scene1a_R, scene1a.cr15_scene1a_motion1_arc, scene1a.cr15_scene1a_motion1_step_angle, i + 1);

                            robot_Motion_ViaPoints(SceneName.None, drCR15, ref tfCR15, ref movement_type);

                            drCR15.SwitchToUserFrame();

                            robotWaitForReady(drCR15);

                            // Check Scene Status One More Time
                            if (SceneStatus_Cr7 == 2 || SceneStatus_Cr15 == 2)
                            {
                                // Do Nothing
                            }
                            else
                            {
                                drCR15.SetOrigin();
                                drCR15.MovetoOrigin();
                                robotWaitForReady(drCR15);
                            }
                        }

                        break;
                    }
            }

            // Check whether the system is estop. if yes. return.
            if (this.SysIsEStop()) return;

            // 2. LiftTable lifts Return Zero
            LiftTableAbsMoveTo(0);

            // 3. Motor Rotation Return Zero
            MotorAbsMoveTo(_motor.Get_homePos());

            this.MoveTo(Model.CR7, Position.Home2);
            robotWaitForReady(drCR7);

            this.MoveTo(Model.CR15, Position.Home2);
            robotWaitForReady(drCR15);

            // Scene End
            this.SetSceneFinish();
        }
        public void Scene1A_Sim(MovementType movement_type, MovementStage movement_stage = MovementStage.Null)
        {
            if (SysIsEStop()) return;

            // Scene Start
            this.SetSceneStart();

            // 2. Motor viaPath 
            tfMotor.motor_points = tfMotor.get_1a_points_motor_motion(
                scene1a.motor_scene1a_arc, scene1a.motor_scene1a_step_angle);

            var motor_points_no = tfMotor.motor_points.Count;

            switch (movement_stage)
            {
                case MovementStage.One:
                    {
                        // loop -- motor points
                        for (int i = 0; i < motor_points_no; i++)
                        {
                            if (this.SysIsEStop()) return;

                            // // 1. motor
                            // MotorAbsMoveTo(tfMotor.motor_points[i]);

                            // 2. CR15
                            // Check Scene Status
                            SceneStatus_Cr7 = this.GetSceneStatus(Model.CR7);
                            SceneStatus_Cr15 = this.GetSceneStatus(Model.CR15);
                            if (SceneStatus_Cr7 == 2 || SceneStatus_Cr15 == 2)
                            {
                                break;
                            }

                            drCR15.SwitchToUserFrame();

                            robotWaitForReady(drCR15);

                            drCR15.SetUserFrameTemp();

                            tfCR15.via_points = tfCR15.get_1a_points_Cr15_motion1a(scene1a.cr15_scene1a_R, scene1a.cr15_scene1a_motion1_arc, scene1a.cr15_scene1a_motion1_step_angle, i + 1);

                            robot_Motion_ViaPoints(SceneName.None, drCR15, ref tfCR15, ref movement_type);

                            drCR15.SwitchToUserFrame();

                            robotWaitForReady(drCR15);

                            // Check Scene Status One More Time
                            if (SceneStatus_Cr7 == 2 || SceneStatus_Cr15 == 2)
                            {
                                // Do Nothing
                            }
                            else
                            {
                                drCR15.SetOrigin();
                                drCR15.MovetoOrigin();
                                robotWaitForReady(drCR15);
                            }
                        }

                        break;
                    }
                case MovementStage.Two:
                    {
                        // loop -- motor points
                        for (int i = 0; i < motor_points_no; i++)
                        {
                            if (this.SysIsEStop()) return;

                            // 2. CR15
                            // Check Scene Status
                            SceneStatus_Cr7 = this.GetSceneStatus(Model.CR7);
                            SceneStatus_Cr15 = this.GetSceneStatus(Model.CR15);
                            if (SceneStatus_Cr7 == 2 || SceneStatus_Cr15 == 2)
                            {
                                break;
                            }

                            drCR15.SwitchToUserFrame();

                            robotWaitForReady(drCR15);

                            drCR15.SetUserFrameTemp();

                            tfCR15.via_points = tfCR15.get_1a_points_Cr15_motion1b(scene1a.cr15_scene1a_R, scene1a.cr15_scene1a_motion1_arc, scene1a.cr15_scene1a_motion1_step_angle, i+1);

                            robot_Motion_ViaPoints(SceneName.None, drCR15, ref tfCR15, ref movement_type);

                            drCR15.SwitchToUserFrame();

                            robotWaitForReady(drCR15);

                            // Check Scene Status One More Time
                            if (SceneStatus_Cr7 == 2 || SceneStatus_Cr15 == 2)
                            {
                                // Do Nothing
                            }
                            else
                            {
                                drCR15.SetOrigin();
                                drCR15.MovetoOrigin();
                                robotWaitForReady(drCR15);
                            }
                        }

                        break;
                    }
            }

            if (this.SysIsEStop()) return;

            this.MoveTo(Model.CR7, Position.Home2);
            robotWaitForReady(drCR7);

            this.MoveTo(Model.CR15, Position.Home2);
            robotWaitForReady(drCR15);

            // Scene End
            this.SetSceneFinish();
        }
        public void Scene3_Sim(MovementType movement_type, MovementStage movement_stage = MovementStage.Null)
        {
            Console.WriteLine("Flag1");

            if (SysIsEStop()) return;

            Console.WriteLine("Flag2");
            // Scene Start
            this.SetSceneStart();

            // CR7 Path -- motion3
            tfCR7.via_orbit_points_part2 = tfCR7.get_1C_points_Cr7(scene3.s3_tilt_angle);

            Console.WriteLine("Flag3");
            /// CR15
            switch (movement_stage)
            {
                case MovementStage.One:
                {
                    // 1. CR15 Path
                    tfCR15.via_points = tfCR15.get_3_points_Cr15_motion1(scene3.s3_span_width, scene3.s3_span_height,
                        scene3.s3_noOfWStep, scene3.s3_noOfHStep);
                    
                    robot_Motion_ViaPoints(SceneName.None, drCR15, ref tfCR15, ref movement_type);
                    
                    Console.WriteLine("Flag4");

                    break;
                }
                case MovementStage.Two:
                {
                    tfCR15.via_points = tfCR15.get_3_points_Cr15_motion2(scene3.s3_span_width, scene3.s3_span_height,
                        scene3.s3_noOfWStep, scene3.s3_noOfHStep);
                    
                    robot_Motion_ViaPoints(SceneName.None, drCR15, ref tfCR15, ref movement_type);
                    
                    Console.WriteLine("Flag5");

                    break;
                }
            }

            if (this.SysIsEStop()) return;

            this.MoveTo(Model.CR15, Position.Home3);
            robotWaitForReady(drCR15);

            this.MoveTo(Model.CR7, Position.Home3);
            robotWaitForReady(drCR7);

            Console.WriteLine("Flag6");

            // Scene End
            this.SetSceneFinish();
        }
        public void Scene4_Sim(MovementType movement_type, MovementStage movement_stage = MovementStage.Null)
        {
            if (SysIsEStop()) return;

            // Scene Start
            this.SetSceneStart();

            //// CR7 Path -- motion3
            //tfCR7.via_orbit_points_part2 = tfCR7.get_1C_points_Cr7(scene3.s3_tilt_angle);

            /// CR15
            switch (movement_stage)
            {
                case MovementStage.One:
                {
                    // 1. CR15 Path
                    tfCR15.via_points = tfCR15.get_4_points_Cr15_motion1(scene4.s4_span_lenght, scene4.s4_span_width,
                        scene4.s4_span_height, scene4.s4_noOfLStep, scene4.s4_noOfWStep, scene4.s4_noOfHStep);

                    robot_Motion_ViaPoints(SceneName.Scene4_Sim, drCR15, ref tfCR15, ref movement_type);

                    Console.WriteLine("Flag4");

                    break;
                }
                case MovementStage.Two:
                {
                    // 1. CR15 Path
                    tfCR15.via_points = tfCR15.get_4_points_Cr15_motion2(scene4.s4_span_lenght, scene4.s4_span_width,
                        scene4.s4_span_height, scene4.s4_noOfLStep, scene4.s4_noOfWStep, scene4.s4_noOfHStep);

                    robot_Motion_ViaPoints(SceneName.Scene4_Sim, drCR15, ref tfCR15, ref movement_type);

                    Console.WriteLine("Flag4");

                    break;
                }
            }

            if (this.SysIsEStop()) return;

            this.MoveTo(Model.CR15, Position.Home2);
            robotWaitForReady(drCR15);

            // this.MoveTo(Model.CR7, Position.Home2);
            // robotWaitForReady(drCR7);

            // Scene End
            this.SetSceneFinish();
        }
        public void Scene4(MovementType movement_type, MovementStage movement_stage = MovementStage.Null)
        {
            if (SysIsEStop()) return;

            // Scene Start
            this.SetSceneStart();

            //// CR7 Path -- motion3
            //tfCR7.via_orbit_points_part2 = tfCR7.get_1C_points_Cr7(scene3.s3_tilt_angle);

            /// CR15
            switch (movement_stage)
            {
                case MovementStage.One:
                {
                    // 1. CR15 Path
                    tfCR15.via_points = tfCR15.get_4_points_Cr15_motion1(scene4.s4_span_lenght, scene4.s4_span_width,
                        scene4.s4_span_height, scene4.s4_noOfLStep, scene4.s4_noOfWStep, scene4.s4_noOfHStep);

                    robot_Motion_ViaPoints(SceneName.Scene4, drCR15, ref tfCR15, ref movement_type);

                    break;
                }
                case MovementStage.Two:
                {
                    // 1. CR15 Path
                    tfCR15.via_points = tfCR15.get_4_points_Cr15_motion2(scene4.s4_span_lenght, scene4.s4_span_width,
                        scene4.s4_span_height, scene4.s4_noOfLStep, scene4.s4_noOfWStep, scene4.s4_noOfHStep);

                    robot_Motion_ViaPoints(SceneName.Scene4, drCR15, ref tfCR15, ref movement_type);

                    break;
                }
            }

            if (this.SysIsEStop()) return;

            this.MoveTo(Model.CR15, Position.Home2);
            robotWaitForReady(drCR15);

            // this.MoveTo(Model.CR7, Position.Home2);
            // robotWaitForReady(drCR7);

            // Scene End
            this.SetSceneFinish();
        }

        // For testing
        public void thread_GetViaPoints()
        {
            // 1. get status -- cr15
            // 2. if arrived --  print current position. wait 1s. 
            // 3. start measuring, wait 1s, set measurement done.
            // 4. move to next position.
            int cr15_move_flag = -1;

            while (true)
            {
                cr15_move_flag = this.GetMoveFlag(Model.CR15);

                if (cr15_move_flag == 2)
                {
                    Thread.Sleep(100);

                    // print current position.
                    this.GetViaPointLocation();

                    // // sleep 1s
                    Thread.Sleep(100);

                    // // set measurement done.

                    this.SetMoveFlag(Model.CR15, 3);
                }

                Thread.Sleep(100);
            }

        }
        
        // To be deleted
        private void TestUO()
        {
            Console.WriteLine("Cr7 UO[" + (2).ToString() + "]: " + drCR7.readUO(2).ToString());
        }
        private void LiftTable_testIOBoard(string HEXstring)
        {
            _liftTable.IOBoardSendCommand(HEXstring);
        }
    }
}   
