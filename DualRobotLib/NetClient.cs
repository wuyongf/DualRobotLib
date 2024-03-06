using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.ComponentModel;
using System.Data;
using System.Diagnostics;
using System.Drawing;
using System.IO.Ports;
using System.Net.Sockets;
using System.Text.RegularExpressions;

namespace DualRobotLib
{
    // For IO Module
    struct IOModuleCommand
    {
        public const string ALL_OFF = "CCDDA1010000FFFFA040";
        public const string ALL_ON  = "CCDDA101FFFFFFFF9E3C";

        public const string IO_16_OFF = "CCDDA101000080002244";
        public const string IO_16_ON = "CCDDA10180008000A244";

        public const string IO_15_OFF = "CCDDA10100004000E2C4";
        public const string IO_15_ON = "CCDDA101400040002244";

        public const string IO_14_OFF = "CCDDA10100002000C284";
        public const string IO_14_ON = "CCDDA10120002000E2C4";
    }

    internal class NetClient
    {
        Socket socket;
        static byte[] buffer = new byte[1024];

        //十六进制字符串转换为字节数组
        public static byte[] hexStringToByteArray(string hexString)
        {

            hexString = hexString.Replace(" ", "");
            if ((hexString.Length % 2) != 0)
                hexString += " ";
            byte[] returnBytes = new byte[hexString.Length / 2];
            for (int i = 0; i < returnBytes.Length; i++)
            {
                try
                {
                    returnBytes[i] = Convert.ToByte(hexString.Substring(i * 2, 2), 16);
                }
                catch (System.FormatException)
                {
                }
            }
            return returnBytes;

        }

        //字节数组转换为十六进制字符串
        public static string bytesToHex(byte[] bytes)
        {
            char[] hexChars = new char[bytes.Length * 2];
            for (int j = 0; j < bytes.Length; j++)
            {
                int v = bytes[j] & 0xFF;
                hexChars[j * 2] = "0123456789ABCDEF".ToCharArray()[v >> 4];
                hexChars[j * 2 + 1] = "0123456789ABCDEF".ToCharArray()[v & 0x0F];
            }
            return new String(hexChars);


        }

        private void Establish(string ip, int port)
        {
            try
            {
                //①创建一个Socket
                socket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);

                //②连接到指定服务器的指定端口
                socket.Connect(ip, port); 

                //③实现异步接受消息的方法 客户端不断监听消息
                socket.BeginReceive(buffer, 0, buffer.Length, SocketFlags.None, new AsyncCallback(ReceiveMessage), socket);

                //④接受用户输入，将消息发送给服务器端

            }
            catch (Exception ex)
            {
                Console.WriteLine( "client:error " + ex.Message);
            }
        }

        public void Connect(string ip, int port)
        {
            Thread thread = new Thread(() => Establish(ip,port));
            thread.Start();
        }

        public void Disconnect()
        {
            socket.Close();
        }

        public bool IsConnected()
        {
            return socket.Connected;
        }

        public void ReceiveMessage(IAsyncResult ar)
        {
            try
            {
                var socket = ar.AsyncState as Socket;

                //方法参考
                var length = socket.EndReceive(ar);
                //读取出来消息内容
                var message = Encoding.ASCII.GetString(buffer, 0, length);
                byte[] tem = new byte[length];
                Array.Copy(buffer, 0, tem, 0, length);
                Array.Clear(buffer, 0, buffer.Length);
                //显示消息
                Console.WriteLine("ioboard received: " + bytesToHex(tem));

                IncomingMsgHandling(bytesToHex(tem));

                //接收下一个消息(因为这是一个递归的调用，所以这样就可以一直接收消息了）
                socket.BeginReceive(buffer, 0, buffer.Length, SocketFlags.None, new AsyncCallback(ReceiveMessage), socket);
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
            }
        }

        public void Send(string HEXstring)
        {
            try
            {
                //发送数据
                socket.BeginSend(hexStringToByteArray(HEXstring), 0, hexStringToByteArray(HEXstring).Length, SocketFlags.None, null, null);
            }
            catch (Exception err)
            {
                Debug.WriteLine(err);
            }
        }

        // Lift table msg handling
        public string msg;
        private void IncomingMsgHandling(string msg_in)
        {
            // for each incoming msg. Check the meaning
            msg = msg_in;

            // // check if the lift table is at lowest position
            // if (msg == "EEFFC00180000D4E9C")
            // {
            //     IsLowest = true;
            // }
            // else
            // {
            //     IsLowest = false;
            // }
        }
    }
}
