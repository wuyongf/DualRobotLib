using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using SinoDev;

namespace DualRobotLib
{
    internal class LiftTable
    {
        // properties
        //private string _io_ip = "192.168.0.119";
        //private int _io_port = 50000;
        //private string _sino_port_name = "COM4";

        // IO Module
        private NetClient _ioModule = new NetClient();

        // Linear Scale   
        public SinoDev.LinearScale _linear = new SinoDev.LinearScale();

        // properties -- linear scale
        public float _highest = 290.0f; // range: 0 - 290
        public float _lowest = 0.0f;
        public float _buffer = 0.5f;   

        // properties -- io board
        public bool IsLowest = false;

        // // Constructor
        // public LiftTable(NetClient ioModule, SinoDev.LinearScale linear)
        // {
        //     _ioModule = ioModule;
        //     _linear = linear;
        // }

        // Methods
        public void Connect(string io_ip, int io_port, string sino_port_name)
        {
            // 1. I/O Module Connection          
            _ioModule.Connect(io_ip, io_port);

            // Wait for connection successful
            Thread.Sleep(2000);

            // 2. Sino Linear Scale Connection   
            _linear.OpenPort(sino_port_name);
        }

        public bool IsConnected()
        {
            if (_ioModule.IsConnected() && _linear.IsConnected())
                return true;
            return false;
        }

        /// <summary>
        /// Disconnect the LiftTable
        /// </summary>
        /// <returns>
        /// True if success, False if fail
        /// </returns>
        public bool Disconnect()
        {
            // 1. I/O Module Disconnection  
            _ioModule.Disconnect();

            var res_io_close = !_ioModule.IsConnected();

            // 2. linear scale Disconnection
            var res_linear_close = _linear.ClosePort();

            return res_io_close && res_linear_close;
        }

        // TODO: Init Safety Issue
        public void Init()
        {
            // 1. IO Module: Set All OFF
            _ioModule.Send(IOModuleCommand.ALL_OFF);

            // 2. Linear Scale: Start Reading Thread
            _linear.ReadThread(true);
        }

        public bool IsHighestPos()
        {
            return false;
        }
        //todo:
        public void MoveToLowestPos()
        {
            this.Down();

            while (true)
            {
                if (CheckIsLowest())
                {
                    this.Stop();
                    break;
                }

                Thread.Sleep(200);
            }
        }

        public void Up()
        {
            int sleep_duration = 500;

            Thread.Sleep(sleep_duration);
            _ioModule.Send(IOModuleCommand.IO_16_ON);
        }

        public void Stop()
        {
            _ioModule.Send(IOModuleCommand.IO_15_ON);

            int sleep_duration = 1000;

            Thread.Sleep(sleep_duration);
            _ioModule.Send(IOModuleCommand.IO_15_OFF);

            Thread.Sleep(sleep_duration);
            _ioModule.Send(IOModuleCommand.IO_14_OFF);

            Thread.Sleep(sleep_duration);
            _ioModule.Send(IOModuleCommand.IO_16_OFF);

            // double check!
            Thread.Sleep(sleep_duration);
            Thread.Sleep(sleep_duration);
            _ioModule.Send(IOModuleCommand.IO_15_ON);
            Thread.Sleep(sleep_duration);
            _ioModule.Send(IOModuleCommand.IO_15_OFF);
        }

        public void Down()
        {
            int sleep_duration = 500;

            Thread.Sleep(sleep_duration);
            _ioModule.Send(IOModuleCommand.IO_14_ON);
        }

        public bool Get_IsMoving()
        {
            var data1 = _linear.GetData(0);

            Thread.Sleep(25);

            var data2 = _linear.GetData(0);

            Thread.Sleep(25);

            var data3 = _linear.GetData(0);

            var data_avg = (data1 + data2 + data3) / 3;

            var error = Math.Abs(data1 - data_avg);

            if (error < 0.008)
                return false;

            return true;
        }

        public double Get_CurHeight()
        {
            return _linear.GetData(0);
        }

        public bool IsSafe()
        {
            var cur_y = _linear.GetData(1);
            var cur_z = _linear.GetData(2);

            if( cur_y < -1 || cur_y > 1 || cur_z < -1 || cur_z > 1)
                return false;
            else
                return true;
        }

        public bool ClearXYZ()
        {
            return _linear.CLRxyzAxis();
        }

        // IO Board
        public void IOBoardSendCommand(string HEXstring)
        {
            _ioModule.Send(HEXstring);
        }

        public bool CheckIsLowest()
        {
            // send command to 读取开关量状态
            _ioModule.Send("CCDDC00100000DCE9C");
            Thread.Sleep(200);

            // check if the lift table is at lowest position
            if (_ioModule.msg == "EEFFC00180000D4E9C")
            {
                return true;
            }

            return false;
        }
    }
}
