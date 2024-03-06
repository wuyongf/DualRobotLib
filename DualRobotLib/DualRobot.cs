using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Annotations;
using MathNet.Numerics.LinearAlgebra;

namespace DualRobotLib
{
    internal class DualRobot
    {
        // Basic info about the robotic arm
        public string IP = "127.0.0.1";
        public int Port = 9021;
        public string ModelName = "Fanuc Robot";

        // Basic info about connection status
        public bool IsConnectCalled = false;

        // Basic info about Fanuc robotic arm program
        public string ProgramName = "UF_TEST";
        public short LineNumber;
        public short State;
        public string ParentProgName;

        // Basic info about scene status
        public int SceneStatus;

        // Params
        //
        // 1. offset_tcp
        public float[] offset_tcp = new float[6];
        // 2.  

        // for lock/mutex lock
        // reference: https://stackoverflow.com/questions/5754879/usage-of-mutex-in-c-sharp
        private readonly object syncLock_R = new object();
        private readonly object syncLock_S = new object();
        private static Mutex mutex = new Mutex();
        private readonly object syncLock_RP = new object();


        public FRRJIf.Core mobjCore;
        public FRRJIf.DataTable mobjDataTable;
        public FRRJIf.DataTable mobjDataTable2;
        public FRRJIf.DataCurPos mobjCurPos;
        public FRRJIf.DataCurPos mobjCurPosUF;
        public FRRJIf.DataCurPos mobjCurPos2;
        public FRRJIf.DataTask mobjTask;
        public FRRJIf.DataTask mobjTaskIgnoreMacro;
        public FRRJIf.DataTask mobjTaskIgnoreKarel;
        public FRRJIf.DataTask mobjTaskIgnoreMacroKarel;
        public FRRJIf.DataPosReg mobjPosReg;
        public FRRJIf.DataPosReg mobjPosReg2;
        public FRRJIf.DataPosRegXyzwpr mobjPosRegXyzwpr;
        public FRRJIf.DataSysVar mobjSysVarInt;
        public FRRJIf.DataSysVar mobjSysVarInt2;
        public FRRJIf.DataSysVar mobjSysVarReal;
        public FRRJIf.DataSysVar mobjSysVarReal2;
        public FRRJIf.DataSysVar mobjSysVarString;
        public FRRJIf.DataSysVarPos mobjSysVarPos;
        public FRRJIf.DataSysVar[] mobjSysVarIntArray;
        public FRRJIf.DataNumReg mobjNumReg;
        public FRRJIf.DataNumReg mobjNumReg2;
        public FRRJIf.DataNumReg mobjNumReg3;
        public FRRJIf.DataAlarm mobjAlarm;
        public FRRJIf.DataAlarm mobjAlarmCurrent;
        public FRRJIf.DataSysVar mobjVarString;
        public FRRJIf.DataString mobjStrReg;
        public FRRJIf.DataString mobjStrRegComment;

        public Array xyzwpr = new float[9];
        public Array config = new short[7];
        public Array joint = new float[9];
        public Array intSDO = new short[100];
        public Array intSDI = new short[10];

        public string strRBCurPosX, strRBCurPosY, strRBCurPosZ, strRBCurPosW, strRBCurPosP, strRBCurPosR;
        public string strUFCurPosX, strUFCurPosY, strUFCurPosZ, strUFCurPosW, strUFCurPosP, strUFCurPosR;
        

        public float[] arrayUFOrigin = new float[6];
        public float[] arrayUFTempOrigin = new float[6];

        /// register pos
        /// 1. get method
        public Array RpParamXyzwpr = new float[9];
        public Array RpParamConfig = new short[7];
        public Array RpParamJoint = new float[9];
        public short RpParamUF, RpParamUT, RpParamValidC, RpParamValidJ;


        // Connection
        public bool Init()
        {
            bool blnRes = false;
            string strHost = null;
            int lngTmp = 0;

            try
            {
                mobjCore = new FRRJIf.Core();

                // You need to set data table before connecting.
                mobjDataTable = mobjCore.DataTable;

                {
                    mobjAlarm = mobjDataTable.AddAlarm(FRRJIf.FRIF_DATA_TYPE.ALARM_LIST, 5, 0);
                    mobjAlarmCurrent = mobjDataTable.AddAlarm(FRRJIf.FRIF_DATA_TYPE.ALARM_CURRENT, 1, 0);
                    mobjCurPos = mobjDataTable.AddCurPos(FRRJIf.FRIF_DATA_TYPE.CURPOS, 1);
                    mobjCurPosUF = mobjDataTable.AddCurPosUF(FRRJIf.FRIF_DATA_TYPE.CURPOS, 1, 15);
                    mobjCurPos2 = mobjDataTable.AddCurPos(FRRJIf.FRIF_DATA_TYPE.CURPOS, 2);
                    mobjTask = mobjDataTable.AddTask(FRRJIf.FRIF_DATA_TYPE.TASK, 1);
                    mobjTaskIgnoreMacro = mobjDataTable.AddTask(FRRJIf.FRIF_DATA_TYPE.TASK_IGNORE_MACRO, 1);
                    mobjTaskIgnoreKarel = mobjDataTable.AddTask(FRRJIf.FRIF_DATA_TYPE.TASK_IGNORE_KAREL, 1);
                    mobjTaskIgnoreMacroKarel = mobjDataTable.AddTask(FRRJIf.FRIF_DATA_TYPE.TASK_IGNORE_MACRO_KAREL, 1);
                    mobjPosReg = mobjDataTable.AddPosReg(FRRJIf.FRIF_DATA_TYPE.POSREG, 1, 1, 100);
                    mobjPosReg2 = mobjDataTable.AddPosReg(FRRJIf.FRIF_DATA_TYPE.POSREG, 2, 1, 4);
                    mobjSysVarInt = mobjDataTable.AddSysVar(FRRJIf.FRIF_DATA_TYPE.SYSVAR_INT, "$FAST_CLOCK");
                    mobjSysVarInt2 = mobjDataTable.AddSysVar(FRRJIf.FRIF_DATA_TYPE.SYSVAR_INT, "$TIMER[10].$TIMER_VAL");
                    mobjSysVarReal = mobjDataTable.AddSysVar(FRRJIf.FRIF_DATA_TYPE.SYSVAR_REAL, "$MOR_GRP[1].$CURRENT_ANG[1]");
                    mobjSysVarReal2 = mobjDataTable.AddSysVar(FRRJIf.FRIF_DATA_TYPE.SYSVAR_REAL, "$DUTY_TEMP");
                    mobjSysVarString = mobjDataTable.AddSysVar(FRRJIf.FRIF_DATA_TYPE.SYSVAR_STRING, "$TIMER[10].$COMMENT");
                    mobjSysVarPos = mobjDataTable.AddSysVarPos(FRRJIf.FRIF_DATA_TYPE.SYSVAR_POS, "$MNUTOOL[1,1]");
                    mobjVarString = mobjDataTable.AddSysVar(FRRJIf.FRIF_DATA_TYPE.SYSVAR_STRING, "$[HTTPKCL]CMDS[1]");
                    mobjNumReg = mobjDataTable.AddNumReg(FRRJIf.FRIF_DATA_TYPE.NUMREG_INT, 1, 200);
                    mobjNumReg2 = mobjDataTable.AddNumReg(FRRJIf.FRIF_DATA_TYPE.NUMREG_REAL, 6, 10);
                    mobjPosRegXyzwpr = mobjDataTable.AddPosRegXyzwpr(FRRJIf.FRIF_DATA_TYPE.POSREG_XYZWPR, 1, 1, 10);
                    mobjStrReg = mobjDataTable.AddString(FRRJIf.FRIF_DATA_TYPE.STRREG, 1, 3);
                    mobjStrRegComment = mobjDataTable.AddString(FRRJIf.FRIF_DATA_TYPE.STRREG_COMMENT, 1, 3);
                }

                // 2nd data table.
                // You must not set the first data table.
                mobjDataTable2 = mobjCore.DataTable2;
                mobjNumReg3 = mobjDataTable2.AddNumReg(FRRJIf.FRIF_DATA_TYPE.NUMREG_INT, 1, 5);
                mobjSysVarIntArray = new FRRJIf.DataSysVar[10];
                mobjSysVarIntArray[0] = mobjDataTable2.AddSysVar(FRRJIf.FRIF_DATA_TYPE.SYSVAR_INT, "$TIMER[1].$TIMER_VAL");
                mobjSysVarIntArray[1] = mobjDataTable2.AddSysVar(FRRJIf.FRIF_DATA_TYPE.SYSVAR_INT, "$TIMER[2].$TIMER_VAL");
                mobjSysVarIntArray[2] = mobjDataTable2.AddSysVar(FRRJIf.FRIF_DATA_TYPE.SYSVAR_INT, "$TIMER[3].$TIMER_VAL");
                mobjSysVarIntArray[3] = mobjDataTable2.AddSysVar(FRRJIf.FRIF_DATA_TYPE.SYSVAR_INT, "$TIMER[4].$TIMER_VAL");
                mobjSysVarIntArray[4] = mobjDataTable2.AddSysVar(FRRJIf.FRIF_DATA_TYPE.SYSVAR_INT, "$TIMER[5].$TIMER_VAL");
                mobjSysVarIntArray[5] = mobjDataTable2.AddSysVar(FRRJIf.FRIF_DATA_TYPE.SYSVAR_INT, "$TIMER[6].$TIMER_VAL");
                mobjSysVarIntArray[6] = mobjDataTable2.AddSysVar(FRRJIf.FRIF_DATA_TYPE.SYSVAR_INT, "$TIMER[7].$TIMER_VAL");
                mobjSysVarIntArray[7] = mobjDataTable2.AddSysVar(FRRJIf.FRIF_DATA_TYPE.SYSVAR_INT, "$TIMER[8].$TIMER_VAL");
                mobjSysVarIntArray[8] = mobjDataTable2.AddSysVar(FRRJIf.FRIF_DATA_TYPE.SYSVAR_INT, "$TIMER[9].$TIMER_VAL");
                mobjSysVarIntArray[9] = mobjDataTable2.AddSysVar(FRRJIf.FRIF_DATA_TYPE.SYSVAR_INT, "$TIMER[10].$TIMER_VAL");

                //get host name
                strHost = IP;

                //get time out value
                lngTmp = 10000;

                //set port number
                mobjCore.PortNumber = Port;

                //connect
                if (lngTmp > 0)
                    mobjCore.TimeOutValue = lngTmp;
                blnRes = mobjCore.Connect(strHost);

                return blnRes;
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
                System.Environment.Exit(0);

                return false;
            }
        }

        // Disconnection
        public bool Clear()
        {
            bool blnRes = false;

            try
            {
                blnRes = mobjCore.Disconnect();

                mobjCore = null;

                return blnRes;
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
                System.Environment.Exit(0);

                return false;
            }
        }

        // read UO
        public int readUO(int UO)
        {
            Array arr =  new short[11];

            mobjCore.ReadUO(1, ref arr, 11);

            return int.Parse(arr.GetValue(UO-1).ToString());
        }

        // get the Registers
        public bool getRegisterInt(int index, ref object value)
        {
            //// // method -3"
            lock (syncLock_R)
            {
                Thread.Sleep(200);

                //Refresh data table
                var res = mobjDataTable.Refresh();

                return mobjNumReg.GetValue(index, ref value);
            }

            //// method -2: not working. mutex lock

            //bool res = false;

            //mutex.WaitOne();

            //try
            //{
            //    //Refresh data table
            //    mobjDataTable.Refresh();

            //    res = mobjNumReg.GetValue(index, ref value);
            //}
            //finally
            //{
            //    mutex.ReleaseMutex();
            //}

            //return res;

            /// method -1: problem: memory leaking
            /// 
            //lock (syncLock_R)
            //{
            //    //Refresh data table
            //    mobjDataTable.Refresh();

            //    return mobjNumReg.GetValue(index, ref value);
            //}

        }

        // set the Registers R[1] = 1; R[2] = 2
        public bool setRegisterInt(int index, int value, int count)
        {
            lock (syncLock_R)
            {
                return mobjNumReg.SetValuesInt(index, value, count);
            }
        }

        public bool setRegisterReal(int index, float value, int count)
        {
            lock (syncLock_R)
            {
                return mobjNumReg.SetValuesReal(index, value, count);
            }
        }

        // get the Register Pos
        public bool getRegisterPos(int index)
        {
            lock (syncLock_RP)
            {
                //Refresh data table
                mobjDataTable.Refresh();

                return mobjPosReg.GetValue(index, ref RpParamXyzwpr, ref RpParamConfig, ref RpParamJoint, ref RpParamUF, ref RpParamUT, ref RpParamValidC, ref RpParamValidJ);
            }
        }

        // set the Register Pos
        public bool setRegisterPosXyzwpr(int index, float[] posArray, System.Array configArray, short UF, short UT)
        {
            short intUF = UF;
            short intUT = UT;

            xyzwpr.SetValue(posArray[0], 0);
            xyzwpr.SetValue(posArray[1], 1);
            xyzwpr.SetValue(posArray[2], 2);
            xyzwpr.SetValue(posArray[3], 3);
            xyzwpr.SetValue(posArray[4], 4);
            xyzwpr.SetValue(posArray[5], 5);

            return mobjPosReg.SetValueXyzwpr(index, ref xyzwpr, ref config, intUF, intUT);
        }

        public bool setRegisterPosJoint(int index, float[] posArray, short UF, short UT)
        {
            short intUF = UF;
            short intUT = UT;

            joint.SetValue(posArray[0], 0);
            joint.SetValue(posArray[1], 1);
            joint.SetValue(posArray[2], 2);
            joint.SetValue(posArray[3], 3);
            joint.SetValue(posArray[4], 4);
            joint.SetValue(posArray[5], 5);

            return mobjPosReg.SetValueJoint(index, ref joint, intUF, intUT);
        }

        // get robot status
        public bool isEStop()
        {
            return Convert.ToBoolean(this.readUO(6));
        }

        // get robot base cur pos
        public double[] GetRBCurPos()
        {
            double[] cur_pos = new double[6];

            var res = this.getRBCurPos();

            cur_pos[0] = double.Parse(this.strRBCurPosX);
            cur_pos[1] = double.Parse(this.strRBCurPosY);
            cur_pos[2] = double.Parse(this.strRBCurPosZ);
            cur_pos[3] = double.Parse(this.strRBCurPosW);
            cur_pos[4] = double.Parse(this.strRBCurPosP);
            cur_pos[5] = double.Parse(this.strRBCurPosR);

            return cur_pos;
        }

        public bool getRBCurPos()
        {
            // 1. cur_pos: xyzwpr
            // 2. cur_config: config

            string strTmp = null;

            short intUF = 0;
            short intUT = 0;
            short intValidC = 0;
            short intValidJ = 0;
            bool blnDT = false;

            //Refresh data table
            blnDT = mobjDataTable.Refresh();

            if (mobjCurPos.GetValue(ref xyzwpr, ref config, ref joint, ref intUF, ref intUT, ref intValidC, ref intValidJ))
            {
                strRBCurPosX = xyzwpr.GetValue(0).ToString();
                strRBCurPosY = xyzwpr.GetValue(1).ToString();
                strRBCurPosZ = xyzwpr.GetValue(2).ToString();
                strRBCurPosW = xyzwpr.GetValue(3).ToString();
                strRBCurPosP = xyzwpr.GetValue(4).ToString();
                strRBCurPosR = xyzwpr.GetValue(5).ToString();

                return true;
            }
            else
            {
                return false;
            }
        }

        // user frame cur pos
        public double[] GetCurPos()
        {
            double[] cur_pos = new double[6];

            var res = this.getUFCurPos();

            cur_pos[0] = double.Parse(this.strUFCurPosX);
            cur_pos[1] = double.Parse(this.strUFCurPosY);
            cur_pos[2] = double.Parse(this.strUFCurPosZ);
            cur_pos[3] = double.Parse(this.strUFCurPosW);
            cur_pos[4] = double.Parse(this.strUFCurPosP);
            cur_pos[5] = double.Parse(this.strUFCurPosR);
            
            return cur_pos;
        }

        private bool getUFCurPos()
        {
            // 1. cur_pos: xyzwpr
            // 2. cur_config: config

            string strTmp = null;

            short intUF = 1;
            short intUT = 1;
            short intValidC = 0;
            short intValidJ = 0;
            bool blnDT = false;

            //Refresh data table
            blnDT = mobjDataTable.Refresh();

            if (mobjCurPosUF.GetValue(ref xyzwpr, ref config, ref joint, ref intUF, ref intUT, ref intValidC, ref intValidJ))
            {
                strUFCurPosX = (float.Parse(xyzwpr.GetValue(0).ToString())).ToString();
                strUFCurPosY = (float.Parse(xyzwpr.GetValue(1).ToString())).ToString();
                strUFCurPosZ = (float.Parse(xyzwpr.GetValue(2).ToString())).ToString();
                strUFCurPosW = (float.Parse(xyzwpr.GetValue(3).ToString())).ToString();
                strUFCurPosP = (float.Parse(xyzwpr.GetValue(4).ToString())).ToString();
                strUFCurPosR = (float.Parse(xyzwpr.GetValue(5).ToString())).ToString();

                return true;
            }
            else
            {
                return false;
            }
        }

        public void move_uf(double[] via_point)
        {
            // config the next via_point
            short UF = 1;
            short UT = 1;

            float[] PosArray = new float[6];
            short[] ConfigArray = new short[6];

            PosArray[0] = float.Parse(via_point[0].ToString());
            PosArray[1] = float.Parse(via_point[1].ToString());
            PosArray[2] = float.Parse(via_point[2].ToString());
            PosArray[3] = float.Parse(via_point[3].ToString());
            PosArray[4] = float.Parse(via_point[4].ToString());
            PosArray[5] = float.Parse(via_point[5].ToString());

            var res = this.setRegisterPosXyzwpr(99, PosArray, this.config, UF, UT);

            // Move
            this.setRegisterInt(1, 4, 1);
            this.setRegisterInt(2, 2, 1);
        }

        public void GetOffsetTCP()
        {
            getRegisterPos(97);

            offset_tcp[0] = float.Parse(RpParamXyzwpr.GetValue(0).ToString());
            offset_tcp[1] = float.Parse(RpParamXyzwpr.GetValue(1).ToString());
            offset_tcp[2] = float.Parse(RpParamXyzwpr.GetValue(2).ToString());
            offset_tcp[3] = float.Parse(RpParamXyzwpr.GetValue(3).ToString());
            offset_tcp[4] = float.Parse(RpParamXyzwpr.GetValue(4).ToString());
            offset_tcp[5] = float.Parse(RpParamXyzwpr.GetValue(5).ToString());
        }

        public void SetOffsetTCP()
        {
            float[] PosArray = new float[6];

            PosArray = offset_tcp;

            var res = this.setRegisterPosXyzwpr(97, PosArray, this.config, this.RpParamUF, this.RpParamUT);

            // 2. station machine: set tool frame
            this.setRegisterInt(1, 5, 1);
            this.setRegisterInt(2, 2, 1);

            // 3. update TCP
            this.GetOffsetTCP();
        }

        public void SetOffsetTCP(float[] tcp)
        {
            var res = this.setRegisterPosXyzwpr(97, tcp, this.config, this.RpParamUF, this.RpParamUT);

            // 2. station machine: set tool frame
            this.setRegisterInt(1, 5, 1);
            this.setRegisterInt(2, 2, 1);

            this.offset_tcp = tcp;                         
        }

        public bool SetOffsetTCPTemp(Matrix<double> rpy)
        {
            // 1. assign the tcp temp
            short UF = 1;
            short UT = 2;

            float[] PosArray = new float[6];

            PosArray[0] = offset_tcp[0];
            PosArray[1] = offset_tcp[1];
            PosArray[2] = offset_tcp[2];
            PosArray[3] = (float)rpy[0, 0];
            PosArray[4] = (float)rpy[1, 0];
            PosArray[5] = (float)rpy[2, 0];

            var res = this.setRegisterPosXyzwpr(96, PosArray, this.config, UF, UT);

            Thread.Sleep(3000);

            // 1. assign offset_tcp
            if (res)
            {
                // 2. station machine: set tool frame temp
                this.setRegisterInt(1, 6, 1);
                this.setRegisterInt(2, 2, 1);

                return true;
            }
            else
            {
                Console.WriteLine("Cannot Apply TCP! Please Check!");

                return false;
            }
        }

        public bool GetProgramStatus()
        {
            bool blnDT = false;

            //Refresh data table
            blnDT = mobjDataTable.Refresh();

            if (mobjTask.GetValue(ref ProgramName, ref LineNumber, ref State, ref ProgramName))
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        public int GetMoveFlag()
        {
            object Register = -1;
            //bool res;
            //var res = this.getRegisterInt(102, ref Register);
            
            if (this.getRegisterInt(102, ref Register))
            {
                return (int) Register;
                //return int.Parse(Register.ToString());
            }
            else
            {
                return -1;
            }
        }

        public bool SetMoveFlag(int move_flag)
        {
            return this.setRegisterInt(102, move_flag, 1);
        }

        public bool SetSpeed(int speed)
        {
            return this.setRegisterInt(103, speed, 1);
        }

        public bool SetPayloadNo(int payloadNo)
        {
            if (payloadNo >= 1 && payloadNo <= 10)
            {
                this.setRegisterInt(105, payloadNo, 1);
                // station machine - 12: set payload number
                Thread.Sleep(2000);
                this.setRegisterInt(1, 12, 1);
                this.setRegisterInt(2, 2, 1);
                return true;
            }
            else Console.WriteLine("Please Select the Correct Payload Number!");
            return false;
        }

        public int GetSpeed()
        {
            object Register = -1;
            bool res;

            res = this.getRegisterInt(103, ref Register);

            if (res)
            {
                return (int)Register;
            }
            else
            {
                return -1;
            }
        }

        public bool SetSceneStatus(int state)
        {
            return this.setRegisterInt(104, state, 1);
        }

        public int GetSceneStatus()
        {
            object Register = -1;
            bool res;

            res = this.getRegisterInt(104, ref Register);

            if (res)
            {
                SceneStatus = (int)Register;
                return SceneStatus;
            }
            else
            {
                return -1;
            }
        }

        public bool WaitForReady()
        {
            object Register1 = -1;
            object Register2 = -1;

            bool res1, res2;

            // wait for R[1], R[2] = 0
            res1 = this.getRegisterInt(1, ref Register1);
            res2 = this.getRegisterInt(2, ref Register2);

            // Console.WriteLine("Register1: " + Register1);
            // Console.WriteLine("Register2: " + Register2);

            while (Register1.ToString() != 0.ToString() || Register2.ToString() != 0.ToString())
            {
                res1 = this.getRegisterInt(1, ref Register1);
                res2 = this.getRegisterInt(2, ref Register2);
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

        // wait for measurement to finish, PLC102 == 3
        public bool WaitForMeasurementDone()
        {
            object Register = -1;
            
            bool res;

            // wait for R[1], R[2] = 0
            res = this.getRegisterInt(102, ref Register);
            
            // Console.WriteLine("Register1: " + Register1);
            
            while (Register.ToString() != 3.ToString())
            {
                res = this.getRegisterInt(102, ref Register);
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

        public void Wait(int ms)
        {
            Thread.Sleep(ms);
        }
        
        public void SetOrigin()
        {
            short UF = 1;
            short UT = 2;

            float[] PosArray = new float[6];

            PosArray[0] = 0;
            PosArray[1] = 0;
            PosArray[2] = 0;
            PosArray[3] = 0;
            PosArray[4] = 0;
            PosArray[5] = 0;

            var res = this.setRegisterPosXyzwpr(99, PosArray, this.config, UF, UT);
        }

        public void SwitchToRobotBase()
        {
            // 1. Switch to Robot Base.
            this.setRegisterInt(1, 2, 1);
            this.setRegisterInt(2, 2, 1);
        }

        public void SetUserFrame() // SetSceneFrame
        {
            // 1. Switch to Robot Base.
            this.SwitchToRobotBase();

            // 2. Record the current pos to PR[98]
            var resRBCurPos = this.getRBCurPos();

            float[] curPosArray = new float[6];
            short UF = 0;
            short UT = 0;

            if (resRBCurPos)
            {
                curPosArray[0] = float.Parse(this.strRBCurPosX);
                curPosArray[1] = float.Parse(this.strRBCurPosY);
                curPosArray[2] = float.Parse(this.strRBCurPosZ);
                curPosArray[3] = float.Parse(this.strRBCurPosW);
                curPosArray[4] = float.Parse(this.strRBCurPosP);
                curPosArray[5] = float.Parse(this.strRBCurPosR);
            }

            var res = this.setRegisterPosXyzwpr(98, curPosArray, this.config, UF, UT);

            // 3. Define User Frame
            this.setRegisterInt(1, 2, 1);
            this.setRegisterInt(2, 2, 1);

            // 4. Record the user frame origin.
            this.arrayUFOrigin = curPosArray;
        }

        public void SetUserFrameTemp()
        {
            // 1. Switch to Robot Base.
            this.SwitchToRobotBase();

            // 2. Record the current pos to PR[98]
            var resRBCurPos = this.getRBCurPos();

            float[] curPosArray = new float[6];
            short UF = 2;
            short UT = 0;

            if (resRBCurPos)
            {
                curPosArray[0] = float.Parse(this.strRBCurPosX);
                curPosArray[1] = float.Parse(this.strRBCurPosY);
                curPosArray[2] = float.Parse(this.strRBCurPosZ);
                curPosArray[3] = float.Parse(this.strRBCurPosW);
                curPosArray[4] = float.Parse(this.strRBCurPosP);
                curPosArray[5] = float.Parse(this.strRBCurPosR);
            }

            var res = this.setRegisterPosXyzwpr(94, curPosArray, this.config, UF, UT);

            // 3. Define User Frame Temp
            this.setRegisterInt(1, 10, 1);
            this.setRegisterInt(2, 2, 1);

            // 4. Record the user frame temp origin.
            this.arrayUFTempOrigin = curPosArray;
        }

        public void SwitchToUserFrame()
        {
            // 1. Switch To User Frame / Scene Frame
            this.setRegisterInt(1, 11, 1);
            this.setRegisterInt(2, 2, 1);
        }

        public void MovetoOrigin()
        {
            // 0. Initialization
            this.setRegisterInt(1, 0, 1);
            this.setRegisterInt(2, 0, 1);

            // 1. Move to uf_via_point
            this.setRegisterInt(1, 8, 1);
            this.setRegisterInt(2, 2, 1);
        }

        // Cr7 and Cr15 Motion_aPoint()
        /// <summary>
        /// motions in a Scene, but no viaPoints(no need to take data).
        /// </summary>
        /// <param name="via_point"></param>
        public void Motion_aPoint(double[] via_point)
        {
            // config the next via_point
            short UF = 1;
            short UT = 2;

            float[] PosArray_part2 = new float[6];
            short[] ConfigArray = new short[6];

            PosArray_part2[0] = float.Parse(via_point[0].ToString());
            PosArray_part2[1] = float.Parse(via_point[1].ToString());
            PosArray_part2[2] = float.Parse(via_point[2].ToString());
            PosArray_part2[3] = float.Parse(via_point[3].ToString());
            PosArray_part2[4] = float.Parse(via_point[4].ToString());
            PosArray_part2[5] = float.Parse(via_point[5].ToString());

            var res = this.setRegisterPosXyzwpr(99, PosArray_part2, this.config, UF, UT);

            // Move
            this.setRegisterInt(1, 8, 1);
            this.setRegisterInt(2, 2, 1);
        }

        //  Cr15 Motion
        public double[] via_point_location = new double[3];

        // Move
        private int GetTargetPosRegNo(Position pos)
        {
            int PosRegNo = 0;

            switch (pos)
            {
                case Position.Home:
                {
                    PosRegNo = 1;
                    break;
                }
                case Position.InitPos_Scene1B:
                {
                    PosRegNo = 2;
                    break;
                }
                case Position.InitPos_Scene2:
                {
                    PosRegNo = 3;
                    break;
                }
                case Position.InitPos_Scene1C:
                {
                    PosRegNo = 4;
                    break;
                }
                case Position.Home2:
                {
                    PosRegNo = 5;
                    break;
                }
                case Position.InitPos_Scene1A:
                {
                    PosRegNo = 6;
                    break;
                }
                case Position.Ready_Scene1B:
                {
                    PosRegNo = 7;
                    break;
                }
                case Position.Ready_Scene2:
                {
                    PosRegNo = 8;
                    break;
                }
                case Position.Ready_Scene1A:
                {
                    PosRegNo = 9;
                    break;
                }
                case Position.InitPos_Scene3:
                {
                    PosRegNo = 10;
                    break;
                }
                case Position.Home3:
                {
                    PosRegNo = 11;
                    break;
                }
                case Position.Ready_Scene3:
                {
                    PosRegNo = 12;
                    break;
                }
                default:
                {
                    PosRegNo = 0;
                    break;
                }
            }

            return PosRegNo;
        }

        public void SetPosXyzwpr(Position pos, float[] data)
        {
            // 1. get target pos reg no
            var PosRegNo = this.GetTargetPosRegNo(pos);

            // 2. update config/UF/UT
            this.getRBCurPos();

            // 3. assign to target pos
            this.setRegisterPosXyzwpr(PosRegNo, data, this.config, this.RpParamUF, this.RpParamUT);
        }

        public void SetPosJoint(Position pos, float[] data)
        {
            // 1. get target pos reg no
            var PosRegNo = this.GetTargetPosRegNo(pos);

            // 2. update config/UF/UT
            this.getRBCurPos();

            // 3. assign to target pos
            this.setRegisterPosJoint(PosRegNo, data, this.RpParamUF, this.RpParamUT);
        }

        public float[] GetPosXyzwpr(Position pos)
        {
            // 1. get target pos reg no
            var PosRegNo = this.GetTargetPosRegNo(pos);

            // 2. get pos from pos reg no
            this.getRegisterPos(PosRegNo);

            float[] PosArray = new float[6];

            PosArray[0] = float.Parse(this.RpParamXyzwpr.GetValue(0).ToString());
            PosArray[1] = float.Parse(this.RpParamXyzwpr.GetValue(1).ToString());
            PosArray[2] = float.Parse(this.RpParamXyzwpr.GetValue(2).ToString());
            PosArray[3] = float.Parse(this.RpParamXyzwpr.GetValue(3).ToString());
            PosArray[4] = float.Parse(this.RpParamXyzwpr.GetValue(4).ToString());
            PosArray[5] = float.Parse(this.RpParamXyzwpr.GetValue(5).ToString());

            return PosArray;
        }

        public float[] GetPosJoint(Position pos)
        {
            // 1. get target pos reg no
            var PosRegNo = this.GetTargetPosRegNo(pos);

            // 2. get pos from pos reg no
            var res = this.getRegisterPos(PosRegNo);

            float[] PosArray = new float[6];

            PosArray[0] = float.Parse(this.RpParamJoint.GetValue(0).ToString());
            PosArray[1] = float.Parse(this.RpParamJoint.GetValue(1).ToString());
            PosArray[2] = float.Parse(this.RpParamJoint.GetValue(2).ToString());
            PosArray[3] = float.Parse(this.RpParamJoint.GetValue(3).ToString());
            PosArray[4] = float.Parse(this.RpParamJoint.GetValue(4).ToString());
            PosArray[5] = float.Parse(this.RpParamJoint.GetValue(5).ToString());

            return PosArray;
        }

        

        public void MoveToXyzwpr(Position pos)
        {
            // get pos info
            var PosArray = this.GetPosXyzwpr(pos);

            // update config/UF/UT
            this.getRBCurPos();

            // assign to rb
            this.setRegisterPosXyzwpr(100, PosArray, this.config, this.RpParamUF, this.RpParamUT);

            // move to 
            // a. Change to Robot Base
            this.setRegisterInt(1, 1, 1);
            this.setRegisterInt(2, 2, 1);
            // b. Move to rb_via_point
            this.setRegisterInt(1, 3, 1);
            this.setRegisterInt(2, 2, 1);

            // wait until arrive
            this.WaitForReady();
        }

        public void MoveToJoint(Position pos)
        {
            // get pos info
            var PosArray = this.GetPosJoint(pos);

            // update config/UF/UT
            // this.SwitchToRobotBase();
            this.getRBCurPos();

            // assign to pos_in_joint
            var res = this.setRegisterPosJoint(95, PosArray, this.RpParamUF, this.RpParamUT);

            // Console.WriteLine("res: " + res);
            // cout_pos("pos", PosArray);
            // Console.WriteLine("UF: " + this.RpParamUF);
            // Console.WriteLine("UT: " + this.RpParamUT);

            // move to 
            // a. Change to Robot Base
            this.setRegisterInt(1, 1, 1);
            this.setRegisterInt(2, 2, 1);
            // b. Move to rb_via_point
            this.setRegisterInt(1, 7, 1);
            this.setRegisterInt(2, 2, 1);

            // wait until arrive
            this.WaitForReady();
        }

        private void cout_pos(string name, float[] pos)
        {
            Console.WriteLine(name + ": " + pos[0] + ", " + pos[1] + ", " +
                              pos[2] + ", " + pos[3] + ", " + pos[4] + ", " + pos[5]);
        }

        public void GetViaPointLocation()
        {

        }

        // For Ice
        public void RBMove(double[] pos)
        {
            // covert doule[] to float[]
            float[] PosArray = new float[6];

            PosArray[0] = float.Parse(pos[0].ToString());
            PosArray[1] = float.Parse(pos[1].ToString());
            PosArray[2] = float.Parse(pos[2].ToString());
            PosArray[3] = float.Parse(pos[3].ToString());
            PosArray[4] = float.Parse(pos[4].ToString());
            PosArray[5] = float.Parse(pos[5].ToString());

            // update config/UF/UT
            this.getRBCurPos();

            // assign to rb
            this.setRegisterPosXyzwpr(100, PosArray, this.config, this.RpParamUF, this.RpParamUT);

            // move to 
            // a. Change to Robot Base
            this.setRegisterInt(1, 1, 1);
            this.setRegisterInt(2, 2, 1);
            // b. Move to rb_via_point
            this.setRegisterInt(1, 3, 1);
            this.setRegisterInt(2, 2, 1);

            // wait until arrive
            this.WaitForReady();
        }

        public void UFMove(double[] pos)
        {
            // covert doule[] to float[]
            float[] PosArray = new float[6];

            PosArray[0] = float.Parse(pos[0].ToString());
            PosArray[1] = float.Parse(pos[1].ToString());
            PosArray[2] = float.Parse(pos[2].ToString());
            PosArray[3] = float.Parse(pos[3].ToString());
            PosArray[4] = float.Parse(pos[4].ToString());
            PosArray[5] = float.Parse(pos[5].ToString());

            // update config/UF/UT
            this.getRBCurPos();

            // assign to rb
            this.setRegisterPosXyzwpr(99, PosArray, this.config, this.RpParamUF, this.RpParamUT);

            // move to 
            // a. Change to User Frame
            this.setRegisterInt(1, 9, 1);
            this.setRegisterInt(2, 2, 1);
            // b. Move to rb_via_point
            this.setRegisterInt(1, 4, 1);
            this.setRegisterInt(2, 2, 1);

            // wait until arrive
            this.WaitForReady();
        }



    }
}
