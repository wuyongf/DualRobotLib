using NLog;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Configuration;
using System.IO.Packaging;
using System.Linq;
using System.Security.RightsManagement;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Forms;
using System.Windows.Threading;

namespace DualRobotLib
{
    class Motor
    {
        SerialPortManager serialPortManager = new SerialPortManager(callback);

        static TextCallback callback = new TextCallback(callbackMethod);

        public static AutoResetEvent stopWaitHandle = new AutoResetEvent(false);

        private static string serialNo;
        private static bool isMoving;
        private static bool isStalled;
        private static int currentPositionCount;

        // motor properties
        private const int gearRatio_RM8 = 80;
        private const int countPerRev = 2048;
        public  const double homePos = 0; //180
        public const double initPos = -180;

        public static void callbackMethod(string command, string data)
        {
            switch (command)
            {
                case "PR SN":
                {
                    serialNo = data;
                    break;
                }
                case "PR MV":
                {
                    isMoving = data == "1" ? true : false;
                    break;
                }
                case "PR ST":
                {
                    isStalled = data == "1" ? true : false;
                    break;
                }
                case "PR P":
                {
                    currentPositionCount = int.Parse(data);
                    break;
                }

            }

            stopWaitHandle.Set();
        }

        public void SendCommand(string command)
        {
            serialPortManager.EnQueue(command);
        }

        // Get Methods
        
        public string Get_SN()
        {
            SendCommand("PR SN");

            stopWaitHandle.WaitOne();

            return serialNo;
        }
        public bool Get_IsMoving()
        {
            SendCommand("PR MV");

            stopWaitHandle.WaitOne();

            return isMoving;
        }
        public bool Get_IsStalled()
        {
            SendCommand("PR ST");

            stopWaitHandle.WaitOne();

            return isStalled;
        }
        public int Get_CurrentPosition()
        {
            SendCommand("PR P");

            stopWaitHandle.WaitOne();

            return currentPositionCount;
        }
        public double Get_Degree()
        {
            return CountToMmDegree(Get_CurrentPosition());
        }

        public double Get_homePos()
        {
            return homePos;
        }

        public double Get_initPos()
        {
            return initPos;
        }



        // Connection/Init
        public void Connect(string portName)
        {
            serialPortManager.Connect(portName);
        }
        public bool IsConnected()
        {
            return serialPortManager.IsConnected();
        }

        public void Disconnect()
        {
            serialPortManager.Disconnect();
        }

        public void Init()
        {
            // motor initial, set all paras in desire values
            serialPortManager.EnQueue("PR SN");
            serialPortManager.EnQueue("PR PN"); // to recognize that the motor is connected.
            serialPortManager.EnQueue("PG"); // exits program mode if needed
            serialPortManager.EnQueue("E"); // ends any running programs
            serialPortManager.EnQueue("EE=1"); // enables the encoder
            serialPortManager.EnQueue("DB=3"); // encoder deadband, default = 1
            serialPortManager.EnQueue("SL 0"); // stops movement
            serialPortManager.EnQueue("ER=0"); // clears errors
            serialPortManager.EnQueue("ST=0"); // clears stall
            serialPortManager.EnQueue("MT=50"); // Encoder need to set 50-100ms
            //serialPortManager.EnQueue("PM=1"); // Encoder need??
            //serialPortManager.EnQueue("SF=???"); // Encoder need??
            //serialPortManager.EnQueue("PR AL"); // get all parameters
            //serialPortManager.EnQueue("L"); // get program listing

            var initVelocity = 2500; // #RM-8 #TestingSet
            var maxVelocity = 8000; // #RM-8 #TestingSet, MOV speed
            var acceleration = 2000; // lower down these value if the loading (DUT) is heavy
            var deceleration = 2000;

            serialPortManager.EnQueue("A=" + acceleration); // A=1000000, E41000
            serialPortManager.EnQueue("D=" + deceleration); // D=1000000, E41000
            serialPortManager.EnQueue("HC=10"); // holding current, change from 100 to 50 because the motor is hot even idle
            serialPortManager.EnQueue("LM=1");
            serialPortManager.EnQueue("MS=250"); // micro step setting
            serialPortManager.EnQueue("RC=75"); // Running current, higher value with higher force also
            serialPortManager.EnQueue("S1=1,0,0"); // for homing setting
            serialPortManager.EnQueue("S2=0,0,0"); // for homing setting
            serialPortManager.EnQueue("S3=0,0,0"); // for homing setting
            serialPortManager.EnQueue("S4=0,0,0"); // for homing setting
            serialPortManager.EnQueue("VI=" + initVelocity); // VI=1000, E41
            serialPortManager.EnQueue("VM=" + maxVelocity); // VM=200000, E8196

            serialPortManager.EnQueue("S"); // save the parameters each time initial
            serialPortManager.EnQueue("PR AL"); // get all parameters, don't remove because use it to indicate initial complete
        }

        // Set Methods

        public void  SetAbsDegree(double degree)
        {
            var count = MmDegreeToCount(degree);

            SendCommand("MA " + count);

            // while (Get_IsMoving())
            // {
            //     // moving
            //     // Console.WriteLine("Moving!");
            //     Thread.Sleep(100);
            // }

            // check if it is arrived.
            // Console.WriteLine("Arrived!");
        }

        public void SetRelDegree(double step_degree)
        {
            var count = MmDegreeToCount(step_degree);

            SendCommand("MR " + count);

            // while (Get_IsMoving())
            // {
            //     // moving
            //     Thread.Sleep(100);
            // }

            // check if it is arrived.

        }

        public void Stop()
        {
            SendCommand("sl 0");

            while (Get_IsMoving())
            {
                Thread.Sleep(100);
            }
        }

        // useful method1
        public static int MmDegreeToCount(double mmDegree)
        {
            int result = 0;

            //float factor;
            //int microStep; // ms = 125 or 250 now

            //factor = microStep * motorStep / countPerRev;
            //count = step / factor;

            /*

            Rotatory RM5:
            Gear Ratio: 72 for RM-5
            Gear Ratio: 36 for RM-5D
            Encoder enabled:
            2048*36 counts = 1 rev (360degree)
            where 250MS rotate slower than 125MS
            Encoder disable:
            MS 250: 1800000 = 1 rev
            MS 125: 1800000 = 2 rev

            ***We will use RM8***
            Rotatory RM8:
            2048*80 counts = 1 rev (360degree)

            */

            result = (int) (mmDegree* countPerRev * gearRatio_RM8 / 360);

            return result;
        }

        public static double CountToMmDegree(int count)
        {
            double result = 0d;

            result = 1.0d * count* 360 / countPerRev / gearRatio_RM8;
		    	
            return result;
        }

        // useful methods 2
        void timer_loop()
        {
            DispatcherTimer _timer = new DispatcherTimer();
            
            //設定呼叫間隔時間為30ms
            _timer.Interval = TimeSpan.FromMilliseconds(30);
            
            //加入callback function
            _timer.Tick += _timer_Tick;
            
            //開始
            _timer.Start();
        }
        void _timer_Tick(object sender, EventArgs e)
        {
            // lock (serialPortManager.monitor)
            // {
            //     if (serialPortManager.serialNo != null)
            //     {
            //         serialNo = serialPortManager.serialNo; 
            //         serialPortManager.serialNo = null;
            //     }
            // }
        }

        public void Test()
        {
            // connect the serial port.
            serialPortManager.Connect("COM2");

            // // disconnect the serial port.
            // serialPortManager.Disconnect();
            // this.Get_SN();
            // SendCommand("PR SN");
            Console.WriteLine("SN = " + this.Get_SN());

            // SendCommand("PR MV");
            Console.WriteLine("IsMoving = " + this.Get_IsMoving());
            // serialPortManager.EnQueue("PR SN");

            this.SetAbsDegree(0);

            Console.WriteLine("Degree = " + this.Get_Degree());

            // this.Init();
        }

        public void Test2()
        {
            // motor
            // -1. Connection
            // 0. Init()
            // 1. Get Current Position Count
            // 2. Return Zero Position
            // 3. Set Degree...

            // 4. What about error handling?

            // optical grating transducer

            // connect the serial port.
            serialPortManager.Connect("COM2");

            this.Init();

            this.SetAbsDegree(0);

        }
    }
}
