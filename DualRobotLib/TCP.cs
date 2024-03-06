using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace DualRobotLib
{
    internal class TCPManager
    {
        // Basic Components
        private Transformation tf = new Transformation();

        // For Tool.
        // _cal_pin_tcp --> _fixture_tcp --> _antenna_tcp
        private float[] _cal_pin_tcp;
        private float[] _fixture_tcp;
        private float[] _antenna_tcp;
        private float[] _cal_pin_length = new float[6]; 

        // For Rotatory Station.
        private float[] _station_pin_tcp;
        private float[] _station_center_zero_tcp;
        private float[] _station_antenna_tcp;
        private float[] _station_pin_length = new float[6];
        private float[] _station_height = new float[6];

        public float[] StationAntennaTCP
        {
            get { return _station_antenna_tcp;}
            set { _station_antenna_tcp = value; }
        }

        // Debug
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

        // Methods: For Tool

        public float[] GetToolFixtureWPR(Model robot, Matrix<double> T_Cr15_Cr7, float[] originPos, float[] defaultPos)
        {
            float[] wpr = new float[3];

            switch (robot)
            {
                case Model.CR15:
                    {
                        var T_Cr7_Origin = tf.pos2T(ref originPos);

                        var T_Cr15 = T_Cr15_Cr7 * T_Cr7_Origin;
                        //cout_TMat("T_Cr15", T_Cr15);

                        var T_R = MatrixExtension.Set_T_R_Scene1B1C();
                        //cout_TMat("T_R", T_R);

                        var T_Cr15_Pos = T_Cr15 * T_R;
                        //cout_TMat("T_Cr15_Pos", T_Cr15_Pos);

                        // T1 = inv(T2) * T3;
                        // 
                        // T3 = T2 * T1
                        // T1: tcp to default_tool
                        // T2: default_tool to robot_base(UF: 10 Tool: 10)
                        // T3: tcp to robot_base (T_Cr15_Pos)

                        var T2 = tf.pos2T(ref defaultPos);

                        var T1 = T2.Inverse() * T_Cr15_Pos;

                        var xyzwpr = tf.T2pos(T1);

                        wpr[0] = xyzwpr[3];
                        wpr[1] = xyzwpr[4];
                        wpr[2] = xyzwpr[5];

                        break;
                    }

                case Model.CR7:
                    {
                        var T_Cr15_Origin = tf.pos2T(ref originPos);

                        var T_Cr7 = T_Cr15_Cr7.Inverse() * T_Cr15_Origin;

                        var T_R = MatrixExtension.Set_T_R_Scene1B1C();

                        var T_Cr7_Pos = T_Cr7 * T_R;

                        // T1 = inv(T2) * T3;
                        // 
                        // T3 = T2 * T1
                        // T1: tcp to default_tool
                        // T2: default_tool to robot_base(UF: 10 Tool: 10)
                        // T3: tcp to robot_base (T_Cr15_Pos)

                        var T2 = tf.pos2T(ref defaultPos);

                        var T1 = T2.Inverse() * T_Cr7_Pos;

                        var xyzwpr = tf.T2pos(T1);

                        wpr[0] = xyzwpr[3];
                        wpr[1] = xyzwpr[4];
                        wpr[2] = xyzwpr[5];

                        break;
                    }
            }

            return wpr;
        }

        public float[] GetToolFixtureWPR(SceneName scene, Model robot, Matrix<double> T_Cr15_Cr7, float[] originPos, float[] defaultPos)
        {
            float[] wpr = new float[3];

            if (scene == SceneName.Scene1B || scene == SceneName.Scene1C || 
                scene == SceneName.Scene2 || scene == SceneName.Scene2_Sim)
            {
                switch (robot)
                {
                    case Model.CR15:
                    {
                        var T_Cr7_Origin = tf.pos2T(ref originPos);

                        var T_Cr15 = T_Cr15_Cr7 * T_Cr7_Origin;
                        //cout_TMat("T_Cr15", T_Cr15);

                        var T_R = MatrixExtension.Set_T_R_Scene1B1C();
                        //cout_TMat("T_R", T_R);

                        var T_Cr15_Pos = T_Cr15 * T_R;
                        //cout_TMat("T_Cr15_Pos", T_Cr15_Pos);

                        // T1 = inv(T2) * T3;
                        // 
                        // T3 = T2 * T1
                        // T1: tcp to default_tool
                        // T2: default_tool to robot_base(UF: 10 Tool: 10)
                        // T3: tcp to robot_base (T_Cr15_Pos)

                        var T2 = tf.pos2T(ref defaultPos);

                        var T1 = T2.Inverse() * T_Cr15_Pos;

                        var xyzwpr = tf.T2pos(T1);

                        wpr[0] = xyzwpr[3];
                        wpr[1] = xyzwpr[4];
                        wpr[2] = xyzwpr[5];

                        break;
                    }

                    case Model.CR7:
                    {
                        var T_Cr15_Origin = tf.pos2T(ref originPos);

                        var T_Cr7 = T_Cr15_Cr7.Inverse() * T_Cr15_Origin;

                        var T_R = MatrixExtension.Set_T_R_Scene1B1C();

                        var T_Cr7_Pos = T_Cr7 * T_R;

                        // T1 = inv(T2) * T3;
                        // 
                        // T3 = T2 * T1
                        // T1: tcp to default_tool
                        // T2: default_tool to robot_base(UF: 10 Tool: 10)
                        // T3: tcp to robot_base (T_Cr15_Pos)

                        var T2 = tf.pos2T(ref defaultPos);

                        var T1 = T2.Inverse() * T_Cr7_Pos;

                        var xyzwpr = tf.T2pos(T1);

                        wpr[0] = xyzwpr[3];
                        wpr[1] = xyzwpr[4];
                        wpr[2] = xyzwpr[5];

                        break;
                    }
                }
            }

            return wpr;
        }

        private double[] GetToolFixtureTCP(SceneName SceneName)
        {
            double[] data = new double[6];

            switch (SceneName)
            {
                case SceneName.Scene1A:
                {
                    break;
                }
            }
            return data;
        }

        public float[] GetToolFixtureTCP(float[] cal_pin_tcp, float cal_pin_offset)
        {
            // Flow: _cal_pin_tcp --> _fixture_tcp
            // Math: T_fixture = T_cal_pin / T_length = T_cal_pin * T_length.Inverse()

            var T_cal_pin = tf.pos2T(ref cal_pin_tcp);

            _cal_pin_length[2] = cal_pin_offset;
            var T_offset = tf.pos2T(ref _cal_pin_length);

            var T_fixture = T_cal_pin * T_offset.Inverse();
           
            _fixture_tcp = tf.T2pos(T_fixture);

            return _fixture_tcp;
        }

        public float[] GetToolAntennaTCP(float[] fixture_tcp, float[] offset)
        {
            // Flow: _fixture_tcp-- > _antenna_tcp
            // Math: T_antenna = T_fixture * T_offset

            var T_fixture = tf.pos2T(ref fixture_tcp);
            var T_offset = tf.pos2T(ref offset);

            var T_antenna = T_fixture * T_offset;

            _antenna_tcp = tf.T2pos(T_antenna);

            return _antenna_tcp;
        }

        // For Rotatory Station
        public float[] GetStationAntennaTCP(float[] station_center_tcp, float[] antenna_offset, float station_offset)
        {
            // Flow: _station_center_tcp-- > _station_antenna_tcp
            // Math: T_station_antenna = T_station_center * T_offset * T_lifter_height

            var T_station_center = tf.pos2T(ref station_center_tcp);
            var T_antenna_offset = tf.pos2T(ref antenna_offset);

            _station_height[2] = station_offset;
            var T_station_height = tf.pos2T(ref _station_height);

            var T_station_antenna = T_station_center * T_antenna_offset * T_station_height;

            _station_antenna_tcp = tf.T2pos(T_station_antenna);

            return _station_antenna_tcp;
        }

        public float[] GetStationCenterZeroTCP(float[] station_cal_pin_tcp_cr7, float station_cal_pin_length)
        {
            // Flow: _cal_pin_tcp --> _fixture_tcp
            // Math: T_fixture = T_cal_pin / T_length = T_cal_pin * T_length.Inverse()
            // Math: T_station_zero = T_cal_pin / T_length = T_cal_pin * T_length.Inverse()

            var T_cal_pin = tf.pos2T(ref station_cal_pin_tcp_cr7);

            _station_pin_length[2] = station_cal_pin_length;
            var T_offset = tf.pos2T(ref _station_pin_length);

            var T_station_zero = T_cal_pin * T_offset.Inverse();

            _station_center_zero_tcp = tf.T2pos(T_station_zero);

            return _station_center_zero_tcp;
        }

        // prerequisite
        // 1. Cal. the Cal. Tools. ---> Set Tool No.1                                                       ---> Robot Base Calibration (part 1)
        // 2. Get CaliBase Data./ Use Rotatory Station Pin to do Calibration --->  Pos_Cr7_CaliBase & Pos_Cr15_CaliBase ---> Robot Base Calibration (part 2)
        // 3. Each Scene Fixture Calibration -> (x,y,z,rx,ry,rz) --->  fixture_tcp_cr7 & fixture_tcp_cr15   ---> Tool Calibration (part 1)
        // 4. Get Rotatory Station Center Zero Position.

        // prerequisite: Robot Base Calibration
        // 1. Cal. the Cal. Tools. (Tool No.1) 
        // 2. Define a co-frame. (Rotatory Station Planar Origin)
        // 3. Move to the same origin respectively.
        // 4. Get the data. (Pos_Cr7_CaliBase & Pos_Cr15_CaliBase)

        // prerequisite: Rotatory Station Calibration(Cr7) (x,y,z,rx,ry,rz)
        // 1. Move to the Cal.Pin Position.
        // 2. Get the Position (x,y,z)
        // 3. Set Orientation the same as co-frame. (rx,ry,rz)
        // 4. Get the final Result.

        // prerequisite: Each Scene Fixture Calibration. (6 points method)
        // 1. Get fixture_tcp_cr7, fixture_tcp_cr15

        // flow: Init
        // 1. Input Robot Base Calibration Data.
        // 2. (optional) Define Station Antenna TCP. 
        // 3. Define Tool Antenna TCP.

        // flow: For Scene1A/2
        // 0. Input Robot Base Calibration Data
        // 1. Get Station Center TCP.             
        // 2. Set/Get Lifer Height. Set Scan R.   
        // 3. Return Station Antenna TCP.         // Define Station Antenna TCP.

        // flow: For Scene1B/1C
        // 1. Define Station Antenna TCP.
        // 2. 

        // flow:
    }
}
