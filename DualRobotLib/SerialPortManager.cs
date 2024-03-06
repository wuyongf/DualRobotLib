using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Diagnostics;
using System.IO.Ports;
using System.Linq;
using System.Management;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Threading;

namespace DualRobotLib
{
    public delegate void TextCallback(string command, string data);
    class SerialPortManager
    {
        TextCallback _callbackMethod;

        // serial port properties
        public SerialPort serialPort = new SerialPort();

        public Queue<String> queue;
        public bool Busy;

        DateTime _lastReceived;
        public string comRecData;
        public Action<string> dataRecAction;

        public bool SendRequest;

        public SerialPortManager(TextCallback callbackMethod)
        {
            Busy = false;
            queue = new Queue<string>();
            _callbackMethod = callbackMethod;
        }

        public void Connect(string portName)
        {
            serialPort = new SerialPort(portName, 9600, Parity.None, 8, StopBits.One);
            serialPort.Open();
            serialPort.DataReceived += SerialPortDataReceived;

            Debug.WriteLine("serialPort Connected, " + serialPort.PortName);
        }

        public bool IsConnected()
        {
            return serialPort.IsOpen;
        }

        public void Disconnect()
        {
            if (serialPort != null)
            {
                Console.WriteLine("serialPort Disconnect, " + serialPort.PortName);
                //queue.Clear();
                serialPort.DataReceived -= SerialPortDataReceived;
                serialPort.Close();
                serialPort = null;
            }
        }

        void SerialPortDataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            var current = Application.Current;
            if (current != null && serialPort != null)
            {
                try
                {
                    Thread th1 = new Thread(() => SerialPDataReceived(serialPort.ReadExisting().ToUpper()));
                    th1.Start();

                    // current.Dispatcher.BeginInvoke(new Action(() => )));
                }
                catch (Exception ex)
                {
                    Console.WriteLine("Error in the call back action routine!!! " + ex.Message);
                }
            }
        }

        void SerialPDataReceived(String received)
        {
            _lastReceived = DateTime.Now;

            // Debug.WriteLine("rcd(" + serialPort.PortName + "): " + received);

            comRecData += received;

            NormalDataReceived();

            if (!Busy)
            {
                TrySend();
            }
        }

        void NormalDataReceived()
        {
            // For Motor.

            var match = Regex.Match(comRecData, @"(.*\r\n)+[>?]", RegexOptions.None);

            // no end of transmission found
            if (!match.Success)
            {
                match = Regex.Match(comRecData, @".*\$", RegexOptions.Singleline);
                if (!match.Success)
                {
                    return;
                }
            }

            // the MDrive is no longer busy processing a command
            Busy = false;

            // Debug.WriteLine("rcd(" + serialPort.PortName + "): " + comRecData);

            // split out the text before the end of transmission with anything that might have (unexpectedly) followed it.
            var response = comRecData.Substring(0, match.Length - 1);
            comRecData = comRecData.Substring(match.Length);

            if (response.StartsWith("PR SN"))
            {
                response = response.Substring(5);

                foreach (var parameter in response.Split(new[] { Environment.NewLine }, StringSplitOptions.RemoveEmptyEntries))
                {
                    match = Regex.Match(parameter, @"([0-9]+)", RegexOptions.None); // G is for variables declaired with VA
                    if (match.Success)
                    {
                        // Console.WriteLine("match.Groups[1].Value, SerialNo. = " + match.Groups[1].Value);
                        // this.serialNo = match.Groups[1].Value.ToString();
                        // this.isConnected = true;

                        if (_callbackMethod != null)
                            _callbackMethod("PR SN",match.Groups[1].Value);
                    }
                }
                return;
            }

            if (response.StartsWith("PR MV"))
            {
                response = response.Substring(5);

                // loop through each parameter
                foreach (var parameter in response.Split(new[] { Environment.NewLine }, StringSplitOptions.RemoveEmptyEntries))
                {
                    match = Regex.Match(parameter, @"([0-1]+)", RegexOptions.None); // G is for variables declaired with VA
                    if (match.Success)
                    {
                        //Console.WriteLine("match.Groups[1].Value = " + match.Groups[1].Value);
                        if (_callbackMethod != null)
                            _callbackMethod("PR MV", match.Groups[1].Value);
                    }
                }
                return;
            }

            if (response.StartsWith("PR ST"))
            {
                response = response.Substring(5);

                // loop through each parameter
                foreach (var parameter in response.Split(new[] { Environment.NewLine }, StringSplitOptions.RemoveEmptyEntries))
                {
                    match = Regex.Match(parameter, @"([0-1]+)", RegexOptions.None);
                    if (match.Success)
                    {
                        //Console.WriteLine("match.Groups[1].Value = " + match.Groups[1].Value);
                        if (_callbackMethod != null)
                            _callbackMethod("PR ST", match.Groups[1].Value);
                    }
                }
                return;
            }

            if (response.StartsWith("PR P"))
            {
                response = response.Substring(4);

                foreach (var parameter in response.Split(new[] { Environment.NewLine }, StringSplitOptions.RemoveEmptyEntries))
                {
                    match = Regex.Match(parameter, @"(-?[0-9]+)", RegexOptions.None); // G is for variables declaired with VA
                    if (match.Success)
                    {
                        //Console.WriteLine("match.Groups[1].Value = " + match.Groups[1].Value);
                        if (_callbackMethod != null)
                            _callbackMethod("PR P", match.Groups[1].Value);
                    }
                }
                return;
            }

        }

        void TrySend()
        {
            if (!Busy && queue.Count > 0 && serialPort != null)
            {
                var command = queue.Dequeue();

                Busy = true;

                Debug.WriteLine("Snd(" + serialPort.PortName + "): " + command);

                if (serialPort.IsOpen)
                {
                    serialPort.Write(command + "\r");
                }
                else
                {
                    Console.Write("Serial Port Error found\n It may caused by disconnecting USB cable during operation.\n Please restart the program.");
                    Application.Current.Shutdown();
                }
            }
        }

        public void EnQueue(String command)
        {
            if (command != null)
            {
                queue.Enqueue(command);
                TrySend();
            }
        }
    }
}
