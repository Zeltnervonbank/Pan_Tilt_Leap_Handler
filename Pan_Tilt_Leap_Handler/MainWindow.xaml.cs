using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.IO.Ports;
using Leap;

namespace Pan_Tilt_Leap_Handler
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        long prevTime = 0;
        int pitch = 0;
        int yaw = 0;
        byte[] bytestosend = new byte[3];
        SerialPort serialPort = new SerialPort();
        public MainWindow()
        {
            InitializeComponent();

            serialPort.BaudRate = 9600;
            serialPort.PortName = "COM3";
            serialPort.Open();

            Controller controller = new Controller();

            controller.Connect += OnServiceConnect;
            controller.Device += OnConnect;
            controller.FrameReady += OnFrame;

        }
        public void OnServiceConnect(object sender, ConnectionEventArgs args)
        {
            Console.WriteLine("Service Connected");
        }

        public void OnConnect(object sender, DeviceEventArgs args)
        {
            
        }

        public void OnFrame(object sender, FrameEventArgs args)
        {
            // Get the most recent frame and report some basic information            
            Leap.Frame frame = args.frame;
            if (prevTime + 100000 < frame.Timestamp | prevTime == 0)
            {

                if (frame.Hands.Count != 0)
                {
                    //Pitch
                    pitch = Convert.ToInt32((frame.Hands[0].Direction.Pitch * 180.0 / Math.PI) * 3.0) + 1080;
                    bytestosend[0] = 0x02;
                    bytestosend[1] = (byte)(0xFF & (pitch >> 8));
                    bytestosend[2] = (byte) (0xFF & pitch);

                    serialPort.Write(bytestosend, 0, bytestosend.Length);

                    //Yaw
                    yaw = Convert.ToInt32((frame.Hands[0].Direction.Yaw * 180.0 / Math.PI) * 3.0) + 1080;
                    bytestosend[0] = 0x01;
                    bytestosend[1] = (byte)(0xFF & (yaw >> 8));
                    bytestosend[2] = (byte)(0xFF & yaw);

                    serialPort.Write(bytestosend, 0, bytestosend.Length);
                }                
                Console.Write(
                  "\rFrame id: {0}, timestamp: {1}, hands: {2}, pitch {3}\t yaw:{4}\t {5}            ",
                  frame.Id, frame.Timestamp, frame.Hands.Count, pitch, yaw, bytestosend[2]
                );

                



                prevTime = frame.Timestamp;
            }
        }
    }
}
