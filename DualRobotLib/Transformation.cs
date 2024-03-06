using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.ExceptionServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;
using FRRJIf;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace DualRobotLib
{
    public static class ArrayExtension
    {
        public static double[] ToDouble(this float[] arr) =>
            Array.ConvertAll(arr, x => (double)x);
    }

    internal class Transformation
    {
        // x, y, z, rx, ry, rz, line_no, point_no, vdi_angle

        // general-0
        public double[] point_location = new double[3];

        // general
        public List<double[]> via_points = new List<double[]>();

        public List<double[]> via_orbit_points_part1 = new List<double[]>();
        public List<double[]> via_orbit_points_part2 = new List<double[]>();

        /// Scene1A Relate Params
        public List<double> motor_points = new List<double>(); // Motor: rotate points

        // Scene2 Relate Params
        public List<double[]> motion1_points = new List<double[]>(); // Cr15: scan points
        public List<double[]> motion2_points = new List<double[]>(); // Cr15: rotate points
        public List<double[]> motion3_points = new List<double[]>(); // Cr7: approach/one point

        /// rb: single circle
        public double[] rb_center_point = new double[6];
        public double rb_measure_radius = 150;
        public double rb_measure_step_radian = 0.1;
        public Matrix<double> RMat;
        public List<double[]> rb_measure_points = new List<double[]>();

        /// rb: multi circle
        public double rb_orbit_radius = 150;
        public double rb_orbit_step_angle = Math.PI / 2;
        public List<double[]> rb_orbit_points = new List<double[]>();

        /// uf: orbit circle
        /// // for cr7
        public double uf_orbit_radius = 150;
        public double uf_orbit_step_angle = 1;
        public List<double[]> uf_orbit_points = new List<double[]>();
        public int uf_orbit_points_no;

        public List<double[]> status_orbit_points = new List<double[]>();

        /// uf: measure circle for 1A
        /// // for cr15
        public double[] uf_center_point = new double[6];
        public double uf_measure_radius = 150;
        public double uf_measure_arc;
        public double uf_measure_step_angle = 0.1;

        public int uf_measure_points_sub_no;
        public List<List<double[]>> uf_measure_points = new List<List<double[]>>();


        public double d2r(double degree)
        {
            double radian = Math.PI / 180.0 * degree;
            return radian;
        }

        public double r2d(double radian)
        {
            var degree = radian * 180.0 / Math.PI;
            return degree;
        }
        public Matrix<double> R2rpy(Matrix<double> R)
        {
            double alpha = 0;
            double gamma = 0;
            double beta = 0;


            var r11 = R[0, 0];
            var r12 = R[0, 1];
            var r13 = R[0, 2];
            var r21 = R[1, 0];
            var r22 = R[1, 1];
            var r23 = R[1, 2];
            var r31 = R[2, 0];
            var r32 = R[2, 1];
            var r33 = R[2, 2];

            beta = Math.Atan2(-r31, Math.Sqrt(r11 * r11 + r21 * r21));

            if (beta > d2r(89.99))
            {
                beta = d2r(89.99);
                alpha = 0;
                gamma = Math.Atan2(r12, r22);
            }
            else if (beta < -d2r(89.99))
            {
                beta = -d2r(89.99);
                alpha = 0;
                gamma = -Math.Atan2(r12, r22);
            }
            else
            {
                var cb = Math.Cos(beta);
                alpha = Math.Atan2(r21 / cb, r11 / cb);
                gamma = Math.Atan2(r32 / cb, r33 / cb);
            }

            Matrix<double> rpy = DenseMatrix.OfArray(new double[,] {
                {r2d(gamma)},
                {r2d(beta)},
                {r2d(alpha)}});

            return rpy;
        }

        public Matrix<double> rpy2R(double roll, double pitch, double yaw)
        {
            var alpha = d2r(yaw);
            var beta = d2r(pitch);
            var gamma = d2r(roll);

            var ca = Math.Cos(alpha);
            var cb = Math.Cos(beta);
            var cg = Math.Cos(gamma);
            var sa = Math.Sin(alpha);
            var sb = Math.Sin(beta);
            var sg = Math.Sin(gamma);

            Matrix<double> Ra = DenseMatrix.OfArray(new double[,] {
                {ca,-sa,0},
                {sa,ca,0},
                {0,0,1}});

            Matrix<double> Rb = DenseMatrix.OfArray(new double[,] {
                {cb,0,sb},
                {0,1,0},
                {-sb,0,cb}});

            Matrix<double> Rg = DenseMatrix.OfArray(new double[,] {
                {1,0,0},
                {0,cg,-sg},
                {0,sg,cg}});

            var R = Ra * Rb * Rg;

            return R;
        }

        public Matrix<double> pos2T(ref float[] pos)
        {
            double[] init_pos = new double[6];
            init_pos[0] = pos[0]; init_pos[1] = pos[1]; init_pos[2] = pos[2];
            init_pos[3] = pos[3]; init_pos[4] = pos[4]; init_pos[5] = pos[5];

            var R = this.rpy2R(pos[3], pos[4], pos[5]);

            Matrix<double> T = DenseMatrix.OfArray(new double[,] {
                {R[0,0],R[0,1],R[0,2],pos[0]},
                {R[1,0],R[1,1],R[1,2],pos[1]},
                {R[2,0],R[2,1],R[2,2],pos[2]},
                {0,0,0,1}});
            return T;
        }

        public Matrix<double> pos2T(ref double[] pos)
        {
            var R = this.rpy2R(pos[3], pos[4], pos[5]);

            Matrix<double> T = DenseMatrix.OfArray(new double[,] {
                {R[0,0],R[0,1],R[0,2],pos[0]},
                {R[1,0],R[1,1],R[1,2],pos[1]},
                {R[2,0],R[2,1],R[2,2],pos[2]},
                {0,0,0,1}});
            return T;
        }

        public float[] T2pos(Matrix<double> T)
        {
            float[] pos = new float[6];

            // translation
            pos[0] = (float)T[0, 3]; pos[1] = (float)T[1, 3]; pos[2] = (float)T[2, 3];

            // rotation
            var rpy = this.R2rpy(T);
            pos[3] = (float)rpy[0, 0]; pos[4] = (float)rpy[1, 0]; pos[5] = (float)rpy[2, 0];

            return pos;
        }

        // 1. get Rotation Matrix
        // 2. get rb_measure_radius
        // 3. get x,y,z
        // 4. get the via points.

        public List<double[]> get_rb_measure_points(double[] rb_center_point, Matrix<double> RMat, double radius, double step_radian)
        {
            rb_measure_points.Clear();

            for (double r = 0; r <= Math.PI / 2; r = r + step_radian)
            {
                double[] measure_point = new double[6];

                measure_point[0] = rb_center_point[0] + Math.Sin(r) * radius * RMat[0, 1] + Math.Cos(r) * radius * RMat[0, 2];
                measure_point[1] = rb_center_point[1] + Math.Sin(r) * radius * RMat[1, 1] + Math.Cos(r) * radius * RMat[1, 2];
                measure_point[2] = rb_center_point[2] + Math.Sin(r) * radius * RMat[2, 1] + Math.Cos(r) * radius * RMat[2, 2];
                measure_point[3] = rb_center_point[3] - r2d(r);
                measure_point[4] = rb_center_point[4];
                measure_point[5] = rb_center_point[5];

                rb_measure_points.Add(measure_point);
            }

            return rb_measure_points;
        }

        public List<double[]> get_rb_orbit_points(double[] rb_center_point, Matrix<double> RMat, double radius, double step_radian)
        {
            rb_orbit_points.Clear();

            for (double r = 0; r <= 2 * Math.PI; r = r + step_radian)
            {
                double[] orbit_point = new double[6];

                orbit_point[0] = rb_center_point[0] + Math.Sin(r) * radius * RMat[0, 0] + Math.Cos(r) * radius * RMat[0, 1];
                orbit_point[1] = rb_center_point[1] + Math.Sin(r) * radius * RMat[1, 0] + Math.Cos(r) * radius * RMat[1, 1];
                orbit_point[2] = rb_center_point[2] + Math.Sin(r) * radius * RMat[2, 0] + Math.Cos(r) * radius * RMat[2, 1];
                orbit_point[3] = rb_center_point[3];
                orbit_point[4] = rb_center_point[4];
                orbit_point[5] = rb_center_point[5];

                rb_orbit_points.Add(orbit_point);
            }

            return rb_orbit_points;
        }

        public List<double[]> get_uf_orbit_points(double radius, double step_degree)
        {
            // init
            uf_orbit_points.Clear();

            var n = 360.0 / step_degree;

            for (double m = 0; m <= n; m = m + step_degree)
            {
                var r = d2r(m);

                double[] orbit_point = new double[6];

                orbit_point[0] = Math.Cos(r) * radius;
                orbit_point[1] = Math.Sin(r) * radius;
                orbit_point[2] = 0;
                orbit_point[3] = 0;
                orbit_point[4] = 0;
                orbit_point[5] = 0;

                uf_orbit_points.Add(orbit_point);
            }

            return uf_orbit_points;
        }

        public List<double[]> get_uf_measure_points_test(double radius, double arc, double step_angle)
        {
            List<double[]> uf_measure_points_sub = new List<double[]>();

            List<double[]> uf_measure_points_sub1 = new List<double[]>();
            List<double[]> uf_measure_points_sub2 = new List<double[]>();

            var sub_arc = arc / 2;

            var n = sub_arc / step_angle;

            // sub_1
            //  1. list out the points
            for (double m = 0; m <= n; m = m + step_angle)
            {
                var r = d2r(m);

                double[] measure_point = new double[6];

                measure_point[0] = Math.Sin(r) * radius;
                measure_point[1] = 0;
                measure_point[2] = Math.Cos(r) * radius;
                measure_point[3] = 0;
                measure_point[4] = m;
                measure_point[5] = 0;

                uf_measure_points_sub1.Add(measure_point);
            }
            // 2. rearrange
            uf_measure_points_sub1.Reverse();

            // half_2
            for (double m = 0; m <= n; m = m + step_angle)
            {
                var r = d2r(m);

                double[] measure_point = new double[6];

                // measure_point[0] = 0;
                // measure_point[1] = -Math.Sin(r) * radius;
                measure_point[0] = -Math.Sin(r) * radius;
                measure_point[1] = 0;
                measure_point[2] = Math.Cos(r) * radius;
                measure_point[3] = 0;
                measure_point[4] = -m;
                measure_point[5] = 0;

                uf_measure_points_sub2.Add(measure_point);
            }

            // uf_measure_points
            uf_measure_points_sub.AddRange(uf_measure_points_sub1);
            uf_measure_points_sub.AddRange(uf_measure_points_sub2);

            return uf_measure_points_sub;
        }

        public List<double[]> get_uf_measure_points_v02(double radius, double arc, double step_angle)
        {
            List<double[]> uf_measure_points_sub = new List<double[]>();

            List<double[]> uf_measure_points_sub1 = new List<double[]>();
            List<double[]> uf_measure_points_sub2 = new List<double[]>();

            var sub_arc = arc / 2;

            var n = sub_arc / step_angle;

            Console.WriteLine("radius: " + radius);
            Console.WriteLine("arc: " + arc);
            Console.WriteLine("step_angle: " + step_angle);

            // sub_1
            //  1. list out the points
            for (double m = 0; m <= n; m = m + step_angle)
            {
                var r = d2r(m);

                double[] measure_point = new double[6];

                measure_point[0] = 0;
                measure_point[1] = Math.Cos(r) * radius;
                measure_point[2] = -Math.Sin(r) * radius;
                measure_point[3] = m;
                measure_point[4] = 0;
                measure_point[5] = 0;

                uf_measure_points_sub1.Add(measure_point);
            }
            // 2. rearrange
            uf_measure_points_sub1.Reverse();

            // half_2
            for (double m = 0; m <= n; m = m + step_angle)
            {
                var r = d2r(m);

                double[] measure_point = new double[6];

                measure_point[0] = 0;
                measure_point[1] = -Math.Sin(r) * radius;
                measure_point[2] = -Math.Cos(r) * radius;
                measure_point[3] = -m;
                measure_point[4] = 0;
                measure_point[5] = 0;

                uf_measure_points_sub2.Add(measure_point);
            }

            double[] final_point = new double[6];

            final_point[0] = 0;
            final_point[1] = 0;
            final_point[2] = 0;
            final_point[3] = 0;
            final_point[4] = 0;
            final_point[5] = 0;

            uf_measure_points_sub2.Add(final_point);

            // uf_measure_points
            uf_measure_points_sub.AddRange(uf_measure_points_sub1);
            uf_measure_points_sub.AddRange(uf_measure_points_sub2);

            return uf_measure_points_sub;
        }

        public List<double[]> get_uf_measure_points_v03(double radius, double arc, double step_angle)
        {
            List<double[]> uf_measure_points_sub = new List<double[]>();

            List<double[]> uf_measure_points_sub1 = new List<double[]>();
            List<double[]> uf_measure_points_sub2 = new List<double[]>();

            if (arc <= 180)
            {
                var start_angle = (180 - arc) / 2;
                var end_angle = 180 - start_angle;

                for (double ang = start_angle; ang <= end_angle; ang = ang + step_angle)
                {
                    var r = d2r(ang);

                    double[] measure_point = new double[6];

                    measure_point[0] = measure_point[4] = measure_point[5] = 0;

                    measure_point[1] = Math.Cos(r) * radius;
                    measure_point[2] = -Math.Sin(r) * radius;

                    if (ang < end_angle / 2)
                    {
                        measure_point[3] = 90 - ang;
                    }
                    else
                    {
                        measure_point[3] = -(ang - 90);
                    }

                    uf_measure_points_sub1.Add(measure_point);
                }
            }

            // uf_measure_points
            uf_measure_points_sub.AddRange(uf_measure_points_sub1);

            return uf_measure_points_sub;
        }

        public List<double[]> get_uf_measure_points_v04(double radius, double arc, double step_angle, double tcp_distance)
        {
            List<double[]> uf_measure_points_sub = new List<double[]>();

            List<double[]> uf_measure_points_sub1 = new List<double[]>();
            List<double[]> uf_measure_points_sub2 = new List<double[]>();

            if (arc <= 180)
            {
                var start_angle = (180 - arc) / 2;
                var end_angle = 180 - start_angle;

                for (double ang = start_angle; ang <= end_angle; ang = ang + step_angle)
                {
                    // init
                    double[] measure_point = new double[9];
                    var r = d2r(ang);

                    // point data
                    measure_point[0] = measure_point[4] = measure_point[5] = 0;
                    measure_point[1] =  Math.Cos(r) * radius + Math.Cos(r) * tcp_distance;
                    measure_point[2] = -Math.Sin(r) * radius - Math.Sin(r) * tcp_distance + tcp_distance;
                    if (ang < end_angle / 2)
                    {
                        measure_point[3] = 90 - ang;
                    }
                    else
                    {
                        measure_point[3] = -(ang - 90);
                    }

                    // push_back
                    uf_measure_points_sub1.Add(measure_point);
                }
            }

            // uf_measure_points
            uf_measure_points_sub.AddRange(uf_measure_points_sub1);

            return uf_measure_points_sub;
        }

        public List<double[]> get_uf_measure_points(double[] origin, double radius, double arc, double step_angle)
        {
            List<double[]> uf_measure_points_sub = new List<double[]>();

            List<double[]> uf_measure_points_sub1 = new List<double[]>();
            List<double[]> uf_measure_points_sub2 = new List<double[]>();

            var sub_arc = arc / 2;

            var n = sub_arc / step_angle;

            // sub_1
            //  1. list out the points
            for (double m = 0; m <= n; m = m + step_angle)
            {
                var r = d2r(m);

                double[] measure_point = new double[6];

                // measure_point[0] = -origin[0] + Math.Sin(r) * radius;
                // measure_point[1] = origin[1];
                measure_point[0] = origin[0];
                measure_point[1] = -origin[1] + Math.Sin(r) * radius;
                measure_point[2] = -Math.Cos(r) * radius;
                measure_point[3] = m;
                measure_point[4] = 0;
                measure_point[5] = 0;

                uf_measure_points_sub1.Add(measure_point);
            }
            // 2. rearrange
            uf_measure_points_sub1.Reverse();

            // half_2
            for (double m = 0; m <= n; m = m + step_angle)
            {
                var r = d2r(m);

                double[] measure_point = new double[6];

                measure_point[0] = -origin[0] - Math.Sin(r) * radius;
                measure_point[1] = origin[1];
                // measure_point[0] = origin[0];
                // measure_point[1] = -origin[1] - Math.Sin(r) * radius;
                measure_point[2] = -Math.Cos(r) * radius;
                measure_point[3] = -m;
                measure_point[4] = 0;
                measure_point[5] = 0;

                uf_measure_points_sub2.Add(measure_point);
            }

            // uf_measure_points
            uf_measure_points_sub.AddRange(uf_measure_points_sub1);
            uf_measure_points_sub.AddRange(uf_measure_points_sub2);

            return uf_measure_points_sub;
        }

        // uf orbit point part1 --- swing
        //
        // uf orbit point part1 v01
        public List<double[]> get_uf_orbit_points_part1_v01(double arc, double step_angle)
        {
            // init
            List<double[]> uf_orbit_points_part1 = new List<double[]>();

            if (arc <= 180)
            {
                var start_angle = (180 - arc) / 2;
                var end_angle = 180 - start_angle;

                for (double ang = start_angle; ang <= end_angle; ang = ang + step_angle)
                {
                    var r = d2r(ang);

                    double[] measure_point = new double[6];

                    measure_point[0] = measure_point[1] = measure_point[2] = measure_point[4] = measure_point[5] = 0;

                    if (ang < end_angle / 2)
                    {
                        measure_point[3] = 90 - ang;
                    }
                    else
                    {
                        measure_point[3] = -(ang - 90);
                    }

                    uf_orbit_points_part1.Add(measure_point);
                }
            }

            return uf_orbit_points_part1;
        }

        // uf orbit point part1 v02 -- merge with part2
        public List<double[]> get_uf_orbit_points_part1_v02(double arc, double step_angle, float[] cur_pos)
        {
            // init
            List<double[]> uf_orbit_points_part1 = new List<double[]>();

            if (arc <= 180)
            {
                var start_angle = (180 - arc) / 2;
                var end_angle = 180 - start_angle;

                for (double ang = start_angle; ang <= end_angle; ang = ang + step_angle)
                {
                    var r = d2r(ang);

                    double[] measure_point = new double[6];

                    measure_point[0] = cur_pos[0];
                    measure_point[1] = cur_pos[1];
                    measure_point[2] = cur_pos[2];
                    measure_point[5] = cur_pos[5];

                    if (ang < end_angle / 2)
                    {
                        var theta = d2r(cur_pos[5]);

                        // measure_point[3] = (90 - ang);
                        // measure_point[4] = 0;

                        measure_point[3] = Math.Cos(theta) * (90 - ang);
                        measure_point[4] = -Math.Sin(theta) * (90 - ang);
                    }
                    else
                    {
                        var theta = d2r(cur_pos[5]);

                        // measure_point[3] = -(ang - 90) ;
                        // measure_point[4] = 0;

                        measure_point[3] = -(ang - 90) * Math.Cos(theta);
                        measure_point[4] = -(ang - 90) * -Math.Sin(theta);
                    }


                    uf_orbit_points_part1.Add(measure_point);
                }
            }

            return uf_orbit_points_part1;
        }
        public List<double[]> get_uf_orbit_points_part1_v03(double arc, double step_angle, double[] cur_pos, Matrix<double> rpy)
        {
            // init
            List<double[]> uf_orbit_points_1 = new List<double[]>();

            if (arc <= 180)
            {
                var start_angle = (180 - arc) / 2;
                var end_angle = 180 - start_angle;

                for (double ang = start_angle; ang <= end_angle; ang = ang + step_angle)
                {
                    var r = d2r(ang);

                    double[] measure_point = new double[6];

                    measure_point[0] = cur_pos[0];
                    measure_point[1] = cur_pos[1];
                    measure_point[2] = cur_pos[2];
                    measure_point[4] = 0;
                    measure_point[5] = 0; // todo: cur_pos[5]?

                    if (ang < end_angle / 2)
                    {
                        measure_point[3] = 0 + (90 - ang);

                        // measure_point[3] = Math.Cos(theta) * (90 - ang);
                        // measure_point[4] = -Math.Sin(theta) * (90 - ang);
                    }
                    else
                    {
                        measure_point[3] = 0 - (ang - 90);

                        // measure_point[3] = -(ang - 90) * Math.Cos(theta);
                        // measure_point[4] = -(ang - 90) * -Math.Sin(theta);
                    }

                    uf_orbit_points_1.Add(measure_point);
                }
            }

            // // filter
            // List<double[]> uf_orbit_points_2 = new List<double[]>();
            //
            // for (int i = 0; i < uf_orbit_points_1.Count; i++)
            // {
            //     double[] point = new double[6];
            //
            //     point[0] = uf_orbit_points_1[i][0];
            //     point[1] = uf_orbit_points_1[i][1];
            //     point[2] = uf_orbit_points_1[i][2];
            //
            //     var R1 = rpy2R(cur_pos[3], cur_pos[4], cur_pos[5]);
            //
            //     var theta  = d2r(cur_pos[5]);
            //     var ctheta = Math.Cos(theta);
            //     var stheta = Math.Sin(theta);
            //
            //     Matrix<double> Rz = DenseMatrix.OfArray(new double[,] {
            //         {ctheta,-stheta,0},
            //         {stheta,ctheta,0},
            //         {0,0,1}});
            //
            //     var Ro = Rz * R1;
            //
            //     var rpy = R2rpy(Ro);
            //
            //     point[3] = rpy[0,0];
            //     point[4] = rpy[1,0];
            //     point[5] = rpy[2,0];
            //
            //     uf_orbit_points_2.Add(point);
            // }

            return uf_orbit_points_1;
        }

        public List<double[]> get_uf_orbit_points_part1_v04(double arc, double step_angle, float[] cur_pos, float[] offset_tcp)
        {
            // v04: do transformation for each point
            // init
            List<double[]> uf_orbit_points_part1 = new List<double[]>();

            if (arc <= 180)
            {
                var start_angle = (180 - arc) / 2;
                var end_angle = 180 - start_angle;

                for (double ang = start_angle; ang <= end_angle; ang = ang + step_angle)
                {
                    var r = d2r(ang);

                    double[] measure_point = new double[6];

                    measure_point[0] = cur_pos[0];
                    measure_point[1] = cur_pos[1];
                    measure_point[2] = cur_pos[2];

                    measure_point[4] = 0;
                    measure_point[5] = 0;

                    if (ang < end_angle / 2)
                    {
                        measure_point[3] = (90 - ang);
                    }
                    else
                    {
                        measure_point[3] = -(ang - 90);
                    }

                    // P_via_real = R_temp * P_via_ref
                    var R_via_ref = rpy2R(measure_point[3], measure_point[4], measure_point[5]);

                    var alpha = cur_pos[5];
                    var rpy_temp = GetTempRPY(offset_tcp, alpha);
                    var R_temp = rpy2R(rpy_temp[0, 0], rpy_temp[1, 0], rpy_temp[2, 0]);

                    var R_via_real = R_temp.Inverse() * R_via_ref;

                    var rpy_via_real = R2rpy(R_via_real);

                    measure_point[3] = rpy_via_real[0, 0];
                    measure_point[4] = rpy_via_real[1, 0];
                    measure_point[5] = rpy_via_real[2, 0];

                    uf_orbit_points_part1.Add(measure_point);
                }
            }

            return uf_orbit_points_part1;
        }

        public List<double[]> get_uf_orbit_points_part1_v03_no(double arc, double step_angle)
        {
            // init
            List<double[]> uf_orbit_points_1 = new List<double[]>();

            if (arc <= 180)
            {
                var start_angle = (180 - arc) / 2;
                var end_angle = 180 - start_angle;

                for (double ang = start_angle; ang <= end_angle; ang = ang + step_angle)
                {
                    var r = d2r(ang);

                    double[] measure_point = new double[6];

                    measure_point[0] = 0;
                    measure_point[1] = 0;
                    measure_point[2] = 0;
                    measure_point[4] = 0;
                    measure_point[5] = 0; // todo: cur_pos[5]?

                    if (ang < end_angle / 2)
                    {
                        measure_point[3] = 0 + (90 - ang);

                        // measure_point[3] = Math.Cos(theta) * (90 - ang);
                        // measure_point[4] = -Math.Sin(theta) * (90 - ang);
                    }
                    else
                    {
                        measure_point[3] = 0 - (ang - 90);

                        // measure_point[3] = -(ang - 90) * Math.Cos(theta);
                        // measure_point[4] = -(ang - 90) * -Math.Sin(theta);
                    }

                    uf_orbit_points_1.Add(measure_point);
                }
            }

            

            return uf_orbit_points_1;
        }

        // uf orbit point part2 v01
        public List<double[]> get_uf_orbit_points_part2_v01(double radius, double arc, double step_angle)
        {
            // init
            List<double[]> uf_orbit_points_part2 = new List<double[]>();

            if (arc <= 180)
            {
                var start_angle = (180 - arc) / 2;
                var end_angle = 180 - start_angle;

                for (double ang = start_angle; ang <= end_angle; ang = ang + step_angle)
                {
                    var r = d2r(ang);

                    double[] measure_point = new double[6];

                    /// x
                    measure_point[0] = radius * Math.Sin(r);
                    /// y
                    if (ang < end_angle / 2)
                    {
                        measure_point[1] = -radius * Math.Cos(r);
                    }
                    else
                    {
                        measure_point[1] = radius * Math.Cos(d2r(180 - ang));
                    }
                    /// z. rx, ry
                    measure_point[2] = measure_point[3] = measure_point[4] = 0;

                    /// rz
                    measure_point[5] = ang - 90;

                    uf_orbit_points_part2.Add(measure_point);
                }
            }

            return uf_orbit_points_part2;
        }


        // get temp_rpy
        public Matrix<double> GetTempRPY(float[] offset_tcp, float alpha)
        {
            // Rotation Matrix: offset_tcp
            var R_t = rpy2R(offset_tcp[3], offset_tcp[4], offset_tcp[5]);

            // Rotation Matrix: rotate around new z-axis
            var ca = Math.Cos(d2r(alpha));
            var sa = Math.Sin(d2r(alpha));

            Matrix<double> R_a = DenseMatrix.OfArray(new double[,] {
                {ca,-sa,0},
                {sa,ca,0},
                {0,0,1}});

            var R_d = R_t * R_a;

            var rpy = R2rpy(R_d);

            // Console.WriteLine("alpha: " + alpha);
            // Console.WriteLine("rpy: " + rpy[0,0] + ", " + rpy[1, 0] + ", " + rpy[2, 0]);

            return rpy;
        }

        // CR15: get 2A motion measurement points
        public List<double[]> get_2A_motion_measurement_points(double radius, double arc, double step_angle)
        {
            List<double[]> uf_measure_points_sub = new List<double[]>();

            List<double[]> uf_measure_points_sub1 = new List<double[]>();

            if (arc > 90)
            {
                var start_angle = 90 - arc;
                var end_angle = arc;

                for (double ang = start_angle; ang <= end_angle; ang = ang + step_angle)
                {
                    var r = d2r(ang);

                    double[] measure_point = new double[6];

                    measure_point[1] = measure_point[3] = measure_point[5] = 0;

                    measure_point[0] = Math.Sin(r) * radius;
                    measure_point[2] = -Math.Cos(r) * radius;

                    measure_point[4] = -ang;

                    uf_measure_points_sub1.Add(measure_point);
                }
            }


            if (arc <= 90)
            {
                var start_angle = 0;
                var end_angle = arc;

                for (double ang = start_angle; ang <= end_angle; ang = ang + step_angle)
                {
                    var r = d2r(ang);

                    double[] measure_point = new double[6];

                    measure_point[1] = measure_point[3] = measure_point[5] = 0;

                    measure_point[0] = Math.Sin(r) * radius;
                    measure_point[2] = -Math.Cos(r) * radius;

                    measure_point[4] = -ang;

                    uf_measure_points_sub1.Add(measure_point);
                }
            }

            // uf_measure_points
            uf_measure_points_sub.AddRange(uf_measure_points_sub1);

            return uf_measure_points_sub;
        }

        // CR7: get 2A motion
        public List<double[]> get_2A_motion_points(double radius, double arc, double step_angle)
        {
            List<double[]> uf_measure_points_sub = new List<double[]>();

            List<double[]> uf_measure_points_sub1 = new List<double[]>();

            // if (arc > 90)
            // {
            //     var start_angle = 90 - arc;
            //     var end_angle = arc;
            //
            //     for (double ang = start_angle; ang <= end_angle; ang = ang + step_angle)
            //     {
            //         var r = d2r(ang);
            //
            //         double[] measure_point = new double[6];
            //
            //         measure_point[1] = measure_point[3] = measure_point[5] = 0;
            //
            //         measure_point[0] = Math.Sin(r) * radius;
            //         measure_point[2] = -Math.Cos(r) * radius;
            //
            //         measure_point[4] = -ang;
            //
            //         uf_measure_points_sub1.Add(measure_point);
            //     }
            // }


            if (arc <= 90)
            {
                var start_angle = 0;
                var end_angle = arc;

                for (double ang = start_angle; ang <= end_angle; ang = ang + step_angle)
                {
                    var r = d2r(ang);

                    double[] measure_point = new double[6];

                    measure_point[1] = measure_point[3] = measure_point[5] = 0;

                    measure_point[0] = -Math.Sin(r) * radius;
                    measure_point[2] = -Math.Cos(r) * radius;

                    measure_point[4] = ang;

                    uf_measure_points_sub1.Add(measure_point);
                }
            }

            // uf_measure_points
            uf_measure_points_sub.AddRange(uf_measure_points_sub1);

            return uf_measure_points_sub;
        }

        // Scene2
        public List<double[]> get_2_points_Cr7(double R, double arc, double step_angle)
        {
            List<double[]> uf_measure_points_sub = new List<double[]>();

            List<double[]> uf_measure_points_sub1 = new List<double[]>();

            if (arc <= 90)
            {
                var start_angle = 0;
                var end_angle = arc;

                for (double ang = start_angle; ang <= end_angle; ang = ang + step_angle)
                {
                    var r = d2r(ang);

                    double[] measure_point = new double[6];

                    measure_point[0] = measure_point[4] = measure_point[5] = 0;

                    measure_point[1] = -Math.Cos(r) * R + R;
                    measure_point[2] = Math.Sin(r) * R;

                    measure_point[3] = -ang;

                    uf_measure_points_sub1.Add(measure_point);
                }
            }

            // uf_measure_points
            uf_measure_points_sub.AddRange(uf_measure_points_sub1);

            return uf_measure_points_sub;
        }

        public List<double[]> get_2_points_Cr15_motion1(double R, double arc, double step_angle)
        {
            List<double[]> uf_measure_points_sub = new List<double[]>();

            List<double[]> uf_measure_points_sub1 = new List<double[]>();

            if (arc > 90)
            {
                var start_angle = 90 - arc;
                var end_angle = 90;

                for (double ang = start_angle; ang <= end_angle; ang = ang + step_angle)
                {
                    var r = d2r(ang);

                    double[] measure_point = new double[6];

                    measure_point[0] =  measure_point[4] = measure_point[5] = 0;

                    measure_point[1] = -Math.Sin(r) * R;
                    measure_point[2] = -Math.Cos(r) * R + R;

                    measure_point[3] = -ang;

                    uf_measure_points_sub1.Add(measure_point);
                }
            }

            if (arc <= 90)
            {
                var start_angle = 0;
                var end_angle = arc;

                for (double ang = start_angle; ang <= end_angle; ang = ang + step_angle)
                {
                    var r = d2r(ang);

                    double[] measure_point = new double[6];

                    measure_point[0] = measure_point[4] = measure_point[5] = 0;

                    measure_point[1] = -Math.Sin(r) * R;
                    measure_point[2] = -Math.Cos(r) * R + R;

                    measure_point[3] = -ang;

                    uf_measure_points_sub1.Add(measure_point);
                }
            }

            // uf_measure_points
            uf_measure_points_sub.AddRange(uf_measure_points_sub1);

            return uf_measure_points_sub;
        }

        public List<double[]> get_2_points_Cr7_motion3(double R_max, double R, double arc)
        {
            double step_angle = arc/10;

            // init
            List<double[]> approach_points = new List<double[]>();

            if (arc <= 90)
            {
                var start_angle = 0;
                var end_angle = arc;

                for (double ang = start_angle; ang <= end_angle; ang = ang + step_angle)
                {
                    var r = d2r(ang);

                    double[] measure_point = new double[6];

                    // measure_point[0] = measure_point[4] = measure_point[5] = 0;
                    // measure_point[1] = -Math.Cos(r) * R_max + R_max;
                    // measure_point[2] = Math.Sin(r) * R_max;
                    // measure_point[3] = -ang;

                    measure_point[1] = measure_point[3] = measure_point[5] = 0;
                    measure_point[0] = -Math.Sin(r) * R_max;
                    measure_point[2] = -Math.Cos(r) * R_max + R_max;
                    measure_point[4] = ang;

                    approach_points.Add(measure_point);
                }

                // final point
                double[] final_point = new double[6];
                // final_point[0] = final_point[4] = final_point[5] = 0;
                // final_point[1] = -Math.Cos(d2r(arc)) * R + R_max;
                // final_point[2] = Math.Sin(d2r(arc)) * R;
                // final_point[3] = -arc;

                final_point[1] = final_point[3] = final_point[5] = 0;
                final_point[0] = -Math.Sin(d2r(arc)) * R;
                final_point[2] = -Math.Cos(d2r(arc)) * R + R_max;
                final_point[4] = arc;

                approach_points.Add(final_point);
                approach_points.Add(final_point);
            }

            return approach_points;
        }

        public List<double[]> get_2_points_Cr15_motion1a(double R, double arc, double step_angle, int line_no)
        {
            List<double[]> uf_measure_points_sub = new List<double[]>();

            List<double[]> uf_measure_points_sub1 = new List<double[]>();

            double start_angle = 0, end_angle = 0;

            if (arc > 90)
            {
                start_angle = 90 - arc;
                end_angle = 90;
            }

            if (arc <= 90)
            {
                start_angle = 0;
                end_angle = arc;
            }

            int point_no = 0;

            for (double ang = start_angle; ang <= end_angle; ang = ang + step_angle)
            {
                // init
                double[] measure_point = new double[9];
                var r = d2r(ang);
                point_no++;

                // point data
                measure_point[0] = Math.Sin(r) * R;
                measure_point[1] = 0;
                measure_point[2] = -Math.Cos(r) * R + R;
                measure_point[3] = 0;
                measure_point[4] = -ang;
                measure_point[5] = 0;

                // point location retrieving
                measure_point[6] = 0;
                measure_point[7] = line_no;
                measure_point[8] = point_no;

                // push_back
                uf_measure_points_sub1.Add(measure_point);
            }

            // uf_measure_points
            uf_measure_points_sub.AddRange(uf_measure_points_sub1);

            return uf_measure_points_sub;
        }

        public List<double[]> get_2_points_Cr15_motion1b(double R, double arc, double step_angle, int line_no)
        {
            List<double[]> uf_measure_points_sub = new List<double[]>();

            List<double[]> uf_measure_points_sub1 = new List<double[]>();

            double start_angle = 0, end_angle = 0;

            if (arc > 90)
            {
                start_angle = 90 - arc;
                end_angle = 90;
            }

            if (arc <= 90)
            {
                start_angle = 0;
                end_angle = arc;
            }

            int point_no = 0;

            for (double ang = start_angle; ang <= end_angle; ang = ang + step_angle)
            {
                // init
                double[] measure_point = new double[9];
                var r = d2r(ang);
                point_no++;

                // point data
                measure_point[0] = Math.Sin(r) * R;
                measure_point[1] = 0;
                measure_point[2] = -Math.Cos(r) * R + R;
                measure_point[3] = ang;
                measure_point[4] = 0;
                measure_point[5] = -90;

                // point location retrieving
                measure_point[6] = 90;
                measure_point[7] = line_no;
                measure_point[8] = point_no;

                // push_back
                uf_measure_points_sub1.Add(measure_point);
            }

            // uf_measure_points
            uf_measure_points_sub.AddRange(uf_measure_points_sub1);

            return uf_measure_points_sub;
        }

        public List<double[]> get_2_points_Cr15_motion1c(double R, double arc, double step_angle, int line_no)
        {
            List<double[]> uf_measure_points_sub = new List<double[]>();

            List<double[]> uf_measure_points_sub1 = new List<double[]>();

            double start_angle = 0, end_angle = 0;

            start_angle = -arc/2;
            end_angle = arc / 2;

            int point_no = 0;

            for (double ang = start_angle; ang <= end_angle; ang = ang + step_angle)
            {
                // init
                double[] measure_point = new double[9];
                var r = d2r(ang);
                point_no++;

                // point data
                measure_point[0] = Math.Sin(r) * R;
                measure_point[1] = 0;
                measure_point[2] = -Math.Cos(r) * R + R;
                measure_point[3] = 0;
                measure_point[4] = -ang;
                measure_point[5] = 0;

                // point location retrieving
                measure_point[6] = 0;
                measure_point[7] = line_no;
                measure_point[8] = point_no;

                // push_back
                uf_measure_points_sub1.Add(measure_point);
            }

            // uf_measure_points
            uf_measure_points_sub.AddRange(uf_measure_points_sub1);

            return uf_measure_points_sub;
        }

        public List<double[]> get_2_points_Cr15_motion1d(double R, double arc, double step_angle, int line_no)
        {
            List<double[]> uf_measure_points_sub = new List<double[]>();

            List<double[]> uf_measure_points_sub1 = new List<double[]>();

            double start_angle = 0, end_angle = 0;

            start_angle = -arc / 2;
            end_angle = arc / 2;

            int point_no = 0;

            for (double ang = start_angle; ang <= end_angle; ang = ang + step_angle)
            {
                // init
                double[] measure_point = new double[9];
                var r = d2r(ang);
                point_no++;

                // point data
                measure_point[0] = Math.Sin(r) * R;
                measure_point[1] = 0;
                measure_point[2] = -Math.Cos(r) * R + R;
                measure_point[3] = ang;
                measure_point[4] = 0;
                measure_point[5] = -90;

                // point location retrieving
                measure_point[6] = 90;
                measure_point[7] = line_no;
                measure_point[8] = point_no;

                // push_back
                uf_measure_points_sub1.Add(measure_point);
            }

            // uf_measure_points
            uf_measure_points_sub.AddRange(uf_measure_points_sub1);

            return uf_measure_points_sub;
        }


        public List<double[]> get_2_points_Cr15_motion2(double arc, double step_angle)
        {
            // init
            List<double[]> rotate_points = new List<double[]>();

            if (arc <= 180)
            {
                var start_angle = -arc/2;
                var end_angle = arc/2;

                for (double ang = start_angle; ang <= end_angle; ang = ang + step_angle)
                {
                    var r = d2r(ang);

                    double[] measure_point = new double[6];

                    /// x,y,z
                    measure_point[0] = measure_point[1] = measure_point[2] = 0;
                    /// rx, ry
                    measure_point[3] = measure_point[4] = 0;

                    /// rz
                    measure_point[5] = ang;

                    rotate_points.Add(measure_point);
                }
            }

            return rotate_points;
        }

        // Scene1A
        public List<double[]> get_1a_points_Cr15_motion1a(double R, double arc, double step_angle, int line_no)
        {
            List<double[]> uf_measure_points_sub = new List<double[]>();

            List<double[]> uf_measure_points_sub1 = new List<double[]>();

            double start_angle = 0, end_angle = 0;

            if (arc > 90)
            {
                start_angle = 90 - arc;
                end_angle = 90;
            }

            if (arc <= 90)
            {
                start_angle = 0;
                end_angle = arc;
            }

            int point_no = 0;

            for (double ang = start_angle; ang <= end_angle; ang = ang + step_angle)
            {
                // init
                double[] measure_point = new double[9];
                var r = d2r(ang);
                point_no++;

                // point data
                measure_point[0] = 0;
                measure_point[1] = -Math.Sin(r) * R;
                measure_point[2] = -Math.Cos(r) * R + R;
                measure_point[3] = -ang;
                measure_point[4] = 0;
                measure_point[5] = 0;

                // point location retrieving
                measure_point[6] = 0;
                measure_point[7] = line_no;
                measure_point[8] = point_no;

                // push_back
                uf_measure_points_sub1.Add(measure_point);
            }

            // uf_measure_points
            uf_measure_points_sub.AddRange(uf_measure_points_sub1);

            return uf_measure_points_sub;
        }

        public List<double[]> get_1a_points_Cr15_motion1b(double R, double arc, double step_angle, int line_no)
        {
            List<double[]> uf_measure_points_sub = new List<double[]>();

            List<double[]> uf_measure_points_sub1 = new List<double[]>();

            double start_angle = 0, end_angle = 0;

            if (arc > 90)
            {
                start_angle = 90 - arc;
                end_angle = 90;
            }

            if (arc <= 90)
            {
                start_angle = 0;
                end_angle = arc;
            }

            int point_no = 0;

            for (double ang = start_angle; ang <= end_angle; ang = ang + step_angle)
            {
                // init
                double[] measure_point = new double[9];
                var r = d2r(ang);
                point_no++;

                // point data
                measure_point[0] = 0;
                measure_point[1] = -Math.Sin(r) * R;
                measure_point[2] = -Math.Cos(r) * R + R;
                measure_point[3] = 0;
                measure_point[4] = -ang;
                measure_point[5] = -90;

                // point location retrieving
                measure_point[6] = 90;
                measure_point[7] = line_no;
                measure_point[8] = point_no;

                // push_back
                uf_measure_points_sub1.Add(measure_point);
            }

            // uf_measure_points
            uf_measure_points_sub.AddRange(uf_measure_points_sub1);

            return uf_measure_points_sub;
        }

        public List<double> get_1a_points_motor_motion(double arc, double step_angle)
        {
            // init
            List<double> rotate_points = new List<double>();

            double offset_arc = 0;

            if (arc <= 180)
            {
                var start_angle = 0 + offset_arc;
                var end_angle = arc + offset_arc;

                for (double ang = start_angle; ang <= end_angle; ang = ang + step_angle)
                {
                    rotate_points.Add(ang);
                }
            }

            return rotate_points;
        }

        // Scene1B
        public List<double[]> get_1B_points_Cr15_motion1(double radius, double arc, double step_angle, double tcp_distance, double angle, int line_no)
        {
            List<double[]> uf_measure_points_sub = new List<double[]>();

            List<double[]> uf_measure_points_sub1 = new List<double[]>();
            List<double[]> uf_measure_points_sub2 = new List<double[]>();

            if (arc <= 180)
            {
                var start_angle = (180 - arc) / 2;
                var end_angle = 180 - start_angle;
                int point_no = 0;

                for (double ang = start_angle; ang <= end_angle; ang = ang + step_angle)
                {
                    // init
                    double[] measure_point = new double[9];
                    var r = d2r(ang);
                    point_no++;

                    // point data
                    measure_point[0] = measure_point[4] = measure_point[5] = 0;
                    measure_point[1] = Math.Cos(r) * radius + Math.Cos(r) * tcp_distance;
                    measure_point[2] = -Math.Sin(r) * radius - Math.Sin(r) * tcp_distance + tcp_distance;
                    if (ang < end_angle / 2)
                    {
                        measure_point[3] = 90 - ang;
                    }
                    else
                    {
                        measure_point[3] = -(ang - 90);
                    }

                    // point location retrieving
                    measure_point[6] = angle;
                    measure_point[7] = line_no;
                    measure_point[8] = point_no;

                    // push_back
                    uf_measure_points_sub1.Add(measure_point);
                }
            }

            // uf_measure_points
            uf_measure_points_sub.AddRange(uf_measure_points_sub1);

            return uf_measure_points_sub;
        }

        // Scene1C
        public List<double[]> get_1C_points_Cr15_motion1(double width, double height, double step_w, double step_h)
        {
            List<double[]> uf_measure_points = new List<double[]>();

            List<double[]> uf_measure_points_sub1 = new List<double[]>();

            for (double w = -width/2; w <= width/2; w = w + step_w)
            {
                for (double h = -height/2; h <= height/2; h = h + step_h)
                {
                    double[] measure_point = new double[6];

                    measure_point[2] = measure_point[3] = measure_point[4] = measure_point[5] = 0;

                    measure_point[0] = h; // x
                    measure_point[1] = w; // y

                    uf_measure_points_sub1.Add(measure_point);
                }
            }

            // uf_measure_points
            uf_measure_points.AddRange(uf_measure_points_sub1);

            return uf_measure_points;
        }

        public List<double[]> get_1C_points_Cr15_motion2(double width, double height, double step_w, double step_h)
        {
            List<double[]> uf_measure_points = new List<double[]>();

            List<double[]> uf_measure_points_sub1 = new List<double[]>();

            for (double w = -width / 2; w <= width / 2; w = w + step_w)
            {
                for (double h = -height / 2; h <= height / 2; h = h + step_h)
                {
                    double[] measure_point = new double[6];

                    measure_point[2] = measure_point[3] = measure_point[4] = 0;

                    measure_point[0] = h; // x
                    measure_point[1] = w; // y
                    measure_point[5] = -90;

                    uf_measure_points_sub1.Add(measure_point);
                }
            }

            // uf_measure_points
            uf_measure_points.AddRange(uf_measure_points_sub1);

            return uf_measure_points;
        }

        public List<double[]> get_1C_points_Cr15_motion1_test(double width, double height, double no_step_w, double no_step_h)
        {
            double step_w = width / (no_step_w - 1);
            double step_h = height / (no_step_h - 1);


            List<double[]> uf_measure_points = new List<double[]>();

            List<double[]> uf_measure_points_sub1 = new List<double[]>();

            for (double no_w = 1; no_w <= no_step_w; no_w++)
            {
                var w = -width / 2 + step_w * (no_w - 1);

                for (double no_h = 1; no_h <= no_step_h; no_h++)
                {
                    // init
                    double[] measure_point = new double[9];

                    // point data
                    measure_point[2] = measure_point[3] = measure_point[4] = measure_point[5] = 0;
                    measure_point[0] = -height / 2 + step_h * (no_h - 1); // x
                    measure_point[1] = w; // y

                    // point location retrieving
                    measure_point[6] = 0;
                    measure_point[7] = no_w;
                    measure_point[8] = no_h;

                    // push_back
                    uf_measure_points_sub1.Add(measure_point);
                }
            }

            // uf_measure_points
            uf_measure_points.AddRange(uf_measure_points_sub1);

            return uf_measure_points;
        }

        public List<double[]> get_1C_points_Cr15_motion2_test(double width, double height, double no_step_w, double no_step_h)
        {
            double step_w = width / (no_step_w -1);
            double step_h = height / (no_step_h -1);

            List<double[]> uf_measure_points = new List<double[]>();

            List<double[]> uf_measure_points_sub1 = new List<double[]>();

            for (double no_w = 1; no_w <= no_step_w; no_w++)
            {
                var w = -width / 2 + step_w * (no_w - 1);

                for (double no_h = 1; no_h <= no_step_h; no_h++)
                {
                    // init
                    double[] measure_point = new double[9];

                    // point data
                    measure_point[2] = measure_point[3] = measure_point[4] = 0;
                    measure_point[0] = -height / 2 + step_h * (no_h - 1); // x
                    measure_point[1] = w; // y
                    measure_point[5] = -90;

                    // point location retrieving
                    measure_point[6] = 90;
                    measure_point[7] = no_w;
                    measure_point[8] = no_h;

                    // push_back
                    uf_measure_points_sub1.Add(measure_point);
                }
            }

            // uf_measure_points
            uf_measure_points.AddRange(uf_measure_points_sub1);

            return uf_measure_points;
        }

        public List<double[]> get_1C_points_Cr7(double tilt_angle)
        {
            List<double[]> uf_measure_points = new List<double[]>();

            List<double[]> uf_measure_points_sub1 = new List<double[]>();

            // step_angle: default: 0

            double[] measure_point = new double[6];

            if (tilt_angle == 0)
            {
                measure_point[0] = measure_point[1] = measure_point[2] = 0;
                measure_point[3] = measure_point[4] = measure_point[5] = 0;

                uf_measure_points_sub1.Add(measure_point);
            }
            else if (tilt_angle > 0 && tilt_angle <= 90)
            {
                for (double n = 0; n <= tilt_angle; n = n + tilt_angle / 10)
                {
                    measure_point[0] = measure_point[1] = measure_point[2] = 0;
                    measure_point[3] = measure_point[5] = 0;

                    measure_point[4] = n;

                    uf_measure_points_sub1.Add(measure_point);
                }
            }
            else if ( tilt_angle < 0 && tilt_angle >= -90)
            {
                for (double n = 0; n >= tilt_angle; n = n + tilt_angle / 10)
                {
                    measure_point[0] = measure_point[1] = measure_point[2] = 0;
                    measure_point[3] = measure_point[5] = 0;

                    measure_point[4] = n;

                    uf_measure_points_sub1.Add(measure_point);
                }
            }
            else
            {
                Console.WriteLine("Please input correct tilt_angle.");
            }

            // uf_measure_points
            uf_measure_points.AddRange(uf_measure_points_sub1);

            return uf_measure_points;
        }

        public List<double[]> get_1A_points_Cr15_motion1(double R, double arc, double step_angle)
        {
            List<double[]> uf_measure_points_sub = new List<double[]>();

            List<double[]> uf_measure_points_sub1 = new List<double[]>();

            var start_angle = 90 - arc / 2;
            var end_angle = 90 + arc / 2;

            for (double ang = start_angle; ang <= end_angle; ang = ang + step_angle)
            {
                var r = d2r(ang);

                double[] measure_point = new double[6];

                measure_point[1] = measure_point[3] = measure_point[5] = 0;

                measure_point[0] = Math.Cos(r) * R;
                measure_point[2] = -Math.Sin(r) * R + R;

                measure_point[4] = ang-90;

                uf_measure_points_sub1.Add(measure_point);
            }

            // uf_measure_points
            uf_measure_points_sub.AddRange(uf_measure_points_sub1);

            return uf_measure_points_sub;
        }

        public List<double[]> get_1A_points_Cr15_motion2(double R, double arc, double step_angle)
        {
            List<double[]> uf_measure_points_sub = new List<double[]>();

            List<double[]> uf_measure_points_sub1 = new List<double[]>();

            var start_angle = 90 - arc / 2;
            var end_angle = 90 + arc / 2;

            for (double ang = start_angle; ang <= end_angle; ang = ang + step_angle)
            {
                var r = d2r(ang);

                double[] measure_point = new double[6];

                measure_point[1] = measure_point[4] = 0;

                measure_point[0] = Math.Cos(r) * R;
                measure_point[2] = -Math.Sin(r) * R + R;

                measure_point[3] = ang-90;

                measure_point[5] = 90;

                uf_measure_points_sub1.Add(measure_point);
            }

            // uf_measure_points
            uf_measure_points_sub.AddRange(uf_measure_points_sub1);

            return uf_measure_points_sub;
        }

        // Scene3
        public List<double[]> get_3_points_Cr15_motion1(double width, double height, double no_step_w, double no_step_h)
        {
            double step_w = width / (no_step_w - 1);
            double step_h = height / (no_step_h - 1);


            List<double[]> uf_measure_points = new List<double[]>();

            List<double[]> uf_measure_points_sub1 = new List<double[]>();

            for (double no_w = 1; no_w <= no_step_w; no_w++)
            {
                var w = -width / 2 + step_w * (no_w - 1);

                for (double no_h = 1; no_h <= no_step_h; no_h++)
                {
                    // init
                    double[] measure_point = new double[9];

                    // point data
                    measure_point[2] = measure_point[3] = measure_point[4] = 0;
                    measure_point[0] = -height / 2 + step_h * (no_h - 1); // x
                    measure_point[1] = w; // y
                    measure_point[5] = 0;

                    // point location retrieving
                    measure_point[6] = 0;
                    measure_point[7] = no_w;
                    measure_point[8] = no_h;

                    // push_back
                    uf_measure_points_sub1.Add(measure_point);
                }
            }

            // uf_measure_points
            uf_measure_points.AddRange(uf_measure_points_sub1);

            return uf_measure_points;
        }

        public List<double[]> get_3_points_Cr15_motion2(double width, double height, double no_step_w, double no_step_h)
        {
            double step_w = width / (no_step_w - 1);
            double step_h = height / (no_step_h - 1);

            List<double[]> uf_measure_points = new List<double[]>();

            List<double[]> uf_measure_points_sub1 = new List<double[]>();

            for (double no_w = 1; no_w <= no_step_w; no_w++)
            {
                var w = -width / 2 + step_w * (no_w - 1);

                for (double no_h = 1; no_h <= no_step_h; no_h++)
                {
                    // init
                    double[] measure_point = new double[9];

                    // point data
                    measure_point[2] = measure_point[3] = measure_point[4] = 0;
                    measure_point[0] = -height / 2 + step_h * (no_h - 1); // x
                    measure_point[1] = w; // y
                    measure_point[5] = 90;

                    // point location retrieving
                    measure_point[6] = 90;
                    measure_point[7] = no_w;
                    measure_point[8] = no_h;

                    // push_back
                    uf_measure_points_sub1.Add(measure_point);
                }
            }

            // uf_measure_points
            uf_measure_points.AddRange(uf_measure_points_sub1);

            return uf_measure_points;
        }

        // Scene4
        public List<double[]> get_4_points_Cr15_motion1(double length, double width, double height, double no_step_l, double no_step_w, double no_step_h)
        {
            double step_l = length / (no_step_l - 1);
            double step_w = width  / (no_step_w - 1);
            double step_h = height / (no_step_h - 1);


            List<double[]> measurePoints = new List<double[]>();
            List<double[]> eachLayerPoints = new List<double[]>();

            for (double no_h = 1; no_h <= no_step_h; no_h++)
            {
                var h = - step_h * (no_h - 1);

                for (double no_l = 1; no_l <= no_step_l; no_l++)
                {
                    var l = -length / 2 + step_l * (no_l - 1);

                    for (double no_w = 1; no_w <= no_step_w; no_w++)
                    {
                        // init
                        double[] measurePoint = new double[10];

                        // point data
                        
                        measurePoint[0] = -width / 2 + step_w * (no_w - 1); // x
                        measurePoint[1] = l;                                // y
                        measurePoint[2] = h;                                // z
                        measurePoint[3] = 0;
                        measurePoint[4] = 0;
                        measurePoint[5] = 0;

                        // point location retrieving
                        measurePoint[6] = 0;
                        measurePoint[7] = no_l;
                        measurePoint[8] = no_w;
                        measurePoint[9] = no_h;

                        // push_back
                        eachLayerPoints.Add(measurePoint);
                    }
                }
            }

            // uf_measure_points
            measurePoints.AddRange(eachLayerPoints);

            return measurePoints;
        }

        public List<double[]> get_4_points_Cr15_motion2(double length, double width, double height, double no_step_l, double no_step_w, double no_step_h)
        {
            double step_l = length / (no_step_l - 1);
            double step_w = width / (no_step_w - 1);
            double step_h = height / (no_step_h - 1);


            List<double[]> measurePoints = new List<double[]>();
            List<double[]> eachLayerPoints = new List<double[]>();

            for (double no_h = 1; no_h <= no_step_h; no_h++)
            {
                var h = -step_h * (no_h - 1);

                for (double no_l = 1; no_l <= no_step_l; no_l++)
                {
                    var l = -length / 2 + step_l * (no_l - 1);

                    for (double no_w = 1; no_w <= no_step_w; no_w++)
                    {
                        // init
                        double[] measurePoint = new double[10];

                        // point data

                        measurePoint[0] = -width / 2 + step_w * (no_w - 1); // x
                        measurePoint[1] = l;                                // y
                        measurePoint[2] = h;                                // z
                        measurePoint[3] = 0;
                        measurePoint[4] = 0;
                        measurePoint[5] = -90;

                        // point location retrieving
                        measurePoint[6] = -90;
                        measurePoint[7] = no_l;
                        measurePoint[8] = no_w;
                        measurePoint[9] = no_h;

                        // push_back
                        eachLayerPoints.Add(measurePoint);
                    }
                }
            }

            // uf_measure_points
            measurePoints.AddRange(eachLayerPoints);

            return measurePoints;
        }
    }

    public class Trans
    {
        public double d2r(double degree)
        {
            double radian = Math.PI / 180.0 * degree;
            return radian;
        }

        public double r2d(double radian)
        {
            var degree = radian * 180.0 / Math.PI;
            return degree;
        }
        public Matrix<double> R2rpy(Matrix<double> R)
        {
            double alpha = 0;
            double gamma = 0;
            double beta = 0;


            var r11 = R[0, 0];
            var r12 = R[0, 1];
            var r13 = R[0, 2];
            var r21 = R[1, 0];
            var r22 = R[1, 1];
            var r23 = R[1, 2];
            var r31 = R[2, 0];
            var r32 = R[2, 1];
            var r33 = R[2, 2];

            beta = Math.Atan2(-r31, Math.Sqrt(r11 * r11 + r21 * r21));

            if (beta > d2r(89.99))
            {
                beta = d2r(89.99);
                alpha = 0;
                gamma = Math.Atan2(r12, r22);
            }
            else if (beta < -d2r(89.99))
            {
                beta = -d2r(89.99);
                alpha = 0;
                gamma = -Math.Atan2(r12, r22);
            }
            else
            {
                var cb = Math.Cos(beta);
                alpha = Math.Atan2(r21 / cb, r11 / cb);
                gamma = Math.Atan2(r32 / cb, r33 / cb);
            }

            Matrix<double> rpy = DenseMatrix.OfArray(new double[,] {
                {r2d(gamma)},
                {r2d(beta)},
                {r2d(alpha)}});

            return rpy;
        }

        public Matrix<double> rpy2R(double roll, double pitch, double yaw)
        {
            var alpha = d2r(yaw);
            var beta = d2r(pitch);
            var gamma = d2r(roll);

            var ca = Math.Cos(alpha);
            var cb = Math.Cos(beta);
            var cg = Math.Cos(gamma);
            var sa = Math.Sin(alpha);
            var sb = Math.Sin(beta);
            var sg = Math.Sin(gamma);

            Matrix<double> Ra = DenseMatrix.OfArray(new double[,] {
                {ca,-sa,0},
                {sa,ca,0},
                {0,0,1}});

            Matrix<double> Rb = DenseMatrix.OfArray(new double[,] {
                {cb,0,sb},
                {0,1,0},
                {-sb,0,cb}});

            Matrix<double> Rg = DenseMatrix.OfArray(new double[,] {
                {1,0,0},
                {0,cg,-sg},
                {0,sg,cg}});

            var R = Ra * Rb * Rg;

            return R;
        }

        public Matrix<double> pos2T(ref float[] pos)
        {
            double[] init_pos = new double[6];
            init_pos[0] = pos[0]; init_pos[1] = pos[1]; init_pos[2] = pos[2];
            init_pos[3] = pos[3]; init_pos[4] = pos[4]; init_pos[5] = pos[5];

            var R = this.rpy2R(pos[3], pos[4], pos[5]);

            Matrix<double> T = DenseMatrix.OfArray(new double[,] {
                {R[0,0],R[0,1],R[0,2],pos[0]},
                {R[1,0],R[1,1],R[1,2],pos[1]},
                {R[2,0],R[2,1],R[2,2],pos[2]},
                {0,0,0,1}});
            return T;
        }

        public Matrix<double> pos2T(ref double[] pos)
        {
            var R = this.rpy2R(pos[3], pos[4], pos[5]);

            Matrix<double> T = DenseMatrix.OfArray(new double[,] {
                {R[0,0],R[0,1],R[0,2],pos[0]},
                {R[1,0],R[1,1],R[1,2],pos[1]},
                {R[2,0],R[2,1],R[2,2],pos[2]},
                {0,0,0,1}});
            return T;
        }

        public float[] T2pos(Matrix<double> T)
        {
            float[] pos = new float[6];

            // translation
            pos[0] = (float)T[0, 3]; pos[1] = (float)T[1, 3]; pos[2] = (float)T[2, 3];

            // rotation
            var rpy = this.R2rpy(T);
            pos[3] = (float)rpy[0, 0]; pos[4] = (float)rpy[1, 0]; pos[5] = (float)rpy[2, 0];

            return pos;
        }

        public double[] T2posd(Matrix<double> T)
        {
            double[] pos = new double[6];

            // translation
            pos[0] = T[0, 3]; pos[1] = T[1, 3]; pos[2] = T[2, 3];

            // rotation
            var rpy = this.R2rpy(T);
            pos[3] = rpy[0, 0]; pos[4] = rpy[1, 0]; pos[5] = rpy[2, 0];

            return pos;
        }
    }
}
