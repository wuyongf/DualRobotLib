using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace DualRobotLib
{
    internal class Calibration
    {
        // Basic Components
        private Transformation tf = new Transformation();

        // Calibration Params
        public Matrix<double> T_Cr15_Cr7 = DenseMatrix.Create(4, 4, 0.0); // T5

        // Scene1A Params
        public Matrix<double> T_Cr15_Scene1A;

        // Scene1B Params
        public Matrix<double> T_Cr15_Pos;

        // Scene2 Params
        public Matrix<double> T_Cr7_PlateCenter = DenseMatrix.Create(4, 4, 0.0);
        public Matrix<double> T_Cr7_Scene2;
        public Matrix<double> T_Cr15_Scene2;

        /// <summary>
        /// prerequisite:
        /// 1. calibrate the tool.
        /// 2. select correct tool no.
        /// </summary>
        /// <param name="Pos_Cr7_CalliBase"> Cr7 Current Pos </param>
        /// <param name="Pos_Cr15_CalliBase"> Cr15 Current Pos </param>
        /// <returns></returns>
        public void SetCalibrationTMat(double[] Pos_Cr7_CalliBase, double[] Pos_Cr15_CalliBase)
        {
            // Cr7
            var T_Cr7_CalliBase = tf.pos2T(ref Pos_Cr7_CalliBase);

            // Cr15
            var T_Cr15_CalliBase = tf.pos2T(ref Pos_Cr15_CalliBase);

            T_Cr15_Cr7 = T_Cr15_CalliBase * T_Cr7_CalliBase.Inverse();  // ori
            // T_Cr15_Cr7 = T_Cr7_CalliBase * T_Cr15_CalliBase.Inverse();  // test1
        }

        /// <summary>
        /// Tool distance between Cr7 and Cr15
        /// </summary>
        /// <param name="Scene"></param>
        /// <param name="model1"></param>
        /// <param name="model2"></param>
        /// <param name="cur_pos_cr7"></param>
        /// <param name="cur_pos_cr15"></param>
        /// <returns></returns>
        public double GetTcpDistance(SceneName Scene, Model model1, Model model2, double[] cur_pos_cr7, double[] cur_pos_cr15)
        {
            // Safety Check - For all Scenes
            if (T_Cr15_Cr7 == null)
            {
                Console.WriteLine("Please set calibration TMat first! Please call the method 'core.RobotBaseCalibrationInit()' first");
                return 0;
            }

            // Safety Check - Scene1A and Scene2
            if (T_Cr7_PlateCenter == null && 
                (Scene == SceneName.Scene1A|| Scene == SceneName.Scene1A_Sim ||
                 Scene == SceneName.Scene2 || Scene == SceneName.Scene2_Sim))
            {
                Console.WriteLine("Please set StationAntennaTCP_Cr7 first!");
                return 0;
            }

            // Cr7 and Cr15
            if ((model1 == Model.CR15 && model2 == Model.CR7) || 
                (model1 == Model.CR7 && model2 == Model.CR15))
            {
                var T_Cr7 = tf.pos2T(ref cur_pos_cr7);
                var T_Cr15 = tf.pos2T(ref cur_pos_cr15);

                // Tcp distance
                var Tr = T_Cr7.Inverse() * T_Cr15_Cr7.Inverse() * T_Cr15;
                return Math.Sqrt(Tr[0, 3] * Tr[0, 3] + Tr[1, 3] * Tr[1, 3] + Tr[2, 3] * Tr[2, 3]);
            }

            // Cr15 and LTTT
            if ((model1 == Model.CR15 && model2 == Model.LiftTable) ||
                (model1 == Model.LiftTable && model2 == Model.CR15))
            {
                var T_Cr15 = tf.pos2T(ref cur_pos_cr15);
                // Rotatory Center
                var T_Cr15_PlateCenter = T_Cr15_Cr7 * T_Cr7_PlateCenter * this.Set_T_rotate2();

                Matrix<double> Tr = new DenseMatrix(4, 4);
                Tr = T_Cr15.Inverse() * T_Cr15_PlateCenter;
                var distance = Math.Sqrt(Tr[0, 3] * Tr[0, 3] + Tr[1, 3] * Tr[1, 3] + Tr[2, 3] * Tr[2, 3]);

                var lt_tcp = tf.T2pos(T_Cr15_PlateCenter);
                Console.WriteLine("cr15 tcp = " + string.Join(",", cur_pos_cr15));
                Console.WriteLine("lt_tcp = " + string.Join(",", lt_tcp));

                return distance;
            }

            // Cr7 and LTTT
            if ((model1 == Model.CR7 && model2 == Model.LiftTable) ||
                (model1 == Model.LiftTable && model2 == Model.CR7))
            {
                var T_Cr7 = tf.pos2T(ref cur_pos_cr7);

                // Rotatory Center
                Matrix<double> Tr = new DenseMatrix(4, 4);
                Tr = T_Cr7.Inverse() * T_Cr7_PlateCenter;
                return Math.Sqrt(Tr[0, 3] * Tr[0, 3] + Tr[1, 3] * Tr[1, 3] + Tr[2, 3] * Tr[2, 3]);
            }

            return 0;
        }


        public double GetTcpDistance_Scene2(Model model, double[] cur_pos_cr7, double[] cur_pos_cr15)
        {
            // Console.WriteLine("cr7:" + cur_pos_cr7[0] + " , " + cur_pos_cr7[1] + " , " + cur_pos_cr7[2] + " , " + cur_pos_cr7[3] + " , " + cur_pos_cr7[4] + " , " + cur_pos_cr7[5]);
            // Console.WriteLine("cr15:" + cur_pos_cr15[0] + " , " + cur_pos_cr15[1] + " , " + cur_pos_cr15[2] + " , " + cur_pos_cr15[3] + " , " + cur_pos_cr15[4] + " , " + cur_pos_cr15[5]);

            // Safety Check

            if (T_Cr15_Cr7 == null)
            {
                Console.WriteLine("Please set calibration TMat first!");
                return 0;
            }

            if (T_Cr7_PlateCenter == null)
            {
                Console.WriteLine("Please set StationAntennaTCP_Cr7 first!");
                return 0;
            }

            // Cr7
            var T_Cr7 = tf.pos2T(ref cur_pos_cr7);

            // Cr15
            var T_Cr15 = tf.pos2T(ref cur_pos_cr15);

            // Rotatory Center
            var T_Cr15_PlateCenter = T_Cr15_Cr7 * T_Cr7_PlateCenter * this.Set_T_rotate2();
            // var T_Cr15_PlateCenter = T_Cr15_Cr7 * T_Cr7_PlateCenter * this.Set_T_rotate2();
            Matrix<double> Tr = new DenseMatrix(4,4);

            switch (model)
            {
                case Model.CR7:
                {
                    Tr = T_Cr7.Inverse() * T_Cr7_PlateCenter;

                    return Math.Sqrt(Tr[0, 3] * Tr[0, 3] + Tr[1, 3] * Tr[1, 3] + Tr[2, 3] * Tr[2, 3]);
                }
                case Model.CR15:
                {
                    Tr = T_Cr15.Inverse() * T_Cr15_PlateCenter;
                    return Math.Sqrt(Tr[0, 3] * Tr[0, 3] + Tr[1, 3] * Tr[1, 3] + Tr[2, 3] * Tr[2, 3]);
                }
            }

            return 0.0;
        }
        

        // Scene1B
        public Matrix<double> SetTr(ref double r)
        {
            Matrix<double> T = DenseMatrix.OfArray(new double[,] {
                {1,0,0,0},
                {0,-1,0,0},
                {0,0,-1,r},
                {0,0,0,1}});

            return T;
        }

        // For Scene2 -- Rotation Plate // Calibration Plate && Calibrate Rotation Center.
        // p.s. Calibrated by Cr7 Cal.Tool
        public void SetCr7PlateCenter(double[] Pos_Cr7_PlateCenter)
        {
            // Cr7
            var R_Cr7 = tf.rpy2R(Pos_Cr7_PlateCenter[3], Pos_Cr7_PlateCenter[4], Pos_Cr7_PlateCenter[5]);

            T_Cr7_PlateCenter = DenseMatrix.OfArray(new double[,] {
                {R_Cr7[0,0],R_Cr7[0,1],R_Cr7[0,2],Pos_Cr7_PlateCenter[0]},
                {R_Cr7[1,0],R_Cr7[1,1],R_Cr7[1,2],Pos_Cr7_PlateCenter[1]},
                {R_Cr7[2,0],R_Cr7[2,1],R_Cr7[2,2],Pos_Cr7_PlateCenter[2]},
                {0,0,0,1}});
        }

        // Scene2
        public Matrix<double> Set_T_Cr15_R(ref double r)
        {
            Matrix<double> T = DenseMatrix.OfArray(new double[,] {
                {1,0,0,0},
                {0,1,0,0},
                {0,0,1,r},
                {0,0,0,1}});

            return T;
        }
        public Matrix<double> Set_T_Cr7_r(ref double r, ref double theta)
        {
            Matrix<double> T = DenseMatrix.OfArray(new double[,] {
                {1,0,0,0},
                {0,1,0,-r*Math.Cos(tf.d2r(theta))},
                {0,0,1,r*Math.Sin(tf.d2r(theta))},
                {0,0,0,1}});

            return T;
        }

        public Matrix<double> Set_T_rotate()
        {
            // Matrix<double> T = DenseMatrix.OfArray(new double[,] {
            //     {1,0,0,0},
            //     {0,1,0,0},
            //     {0,0,1,0},
            //     {0,0,0,1}});

            Matrix<double> T = DenseMatrix.OfArray(new double[,] {
                {0,-1,0,0},
                {0,0,1,0},
                {-1,0,0,0},
                {0,0,0,1}});

            return T;
        }
        public Matrix<double> Set_T_rotate2()
        {
            Matrix<double> T = DenseMatrix.OfArray(new double[,] {
                {0,1,0,0},
                {1,0,0,0},
                {0,0,-1,0},
                {0,0,0,1}});

            return T;
        }
        public Matrix<double> Set_T_rotate3()
        {
            // Matrix<double> T = DenseMatrix.OfArray(new double[,] {
            //     {0,1,0,0},
            //     {1,0,0,0},
            //     {0,0,-1,0},
            //     {0,0,0,1}});

            Matrix<double> T = DenseMatrix.OfArray(new double[,] {
                {1,0,0,0},
                {0,-1,0,0},
                {0,0,-1,0},
                {0,0,0,1}});

            return T;
        }

        public Matrix<double> Set_T_rotate4()
        {
            Matrix<double> T = DenseMatrix.OfArray(new double[,]
            {
                {0,-1, 0, 0},
                {1, 0, 0, 0},
                {0, 0, 1, 0},
                {0, 0, 0, 1}
            });
            return T;
        }

        public Matrix<double> Set_T_rotate_default()
        {
            Matrix<double> T = DenseMatrix.OfArray(new double[,] {
                {1,0,0,0},
                {0,1,0,0},
                {0,0,1,0},
                {0,0,0,1}});

            return T;
        }

        public Matrix<double> Set_T_rot_x(double theta)
        {
            Matrix<double> T = DenseMatrix.OfArray(new double[,] {
                {1,0,0,0},
                {0,Math.Cos(tf.d2r(theta)),-Math.Sin(tf.d2r(theta)),0},
                {0,Math.Sin(tf.d2r(theta)),Math.Cos(tf.d2r(theta)),0},
                {0,0,0,1}});

            return T;
        }

        public Matrix<double> Set_T_rot_y(double theta)
        {
            Matrix<double> T = DenseMatrix.OfArray(new double[,] {
                {Math.Cos(tf.d2r(theta)),0,Math.Sin(tf.d2r(theta)),0},
                {0,1,0,0},
                {-Math.Sin(tf.d2r(theta)),0,Math.Cos(tf.d2r(theta)),0},
                {0,0,0,1}});

            return T;
        }

        public Matrix<double> Set_T_diff(ref double diff)
        {
            Matrix<double> T = DenseMatrix.OfArray(new double[,] {
                {1,0,0,0},
                {0,1,0,0},
                {0,0,1,diff},
                {0,0,0,1}});

            return T;
        }



    }

    public static class MatrixExtension
    {
        public static Matrix<double> Set_T_R_Scene1B1C()
        {
            Matrix<double> T = DenseMatrix.OfArray(new double[,] {
                {1,0,0,0},
                {0,-1,0,0},
                {0,0,-1,0},
                {0,0,0,1}});

            return T;
        }
    }
}
