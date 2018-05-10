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
using Leap;

namespace Pan_Tilt_Leap_Handler
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        long prevTime = 0;
        public MainWindow()
        {
            InitializeComponent();
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
            Console.WriteLine("Connected");
        }

        public void OnFrame(object sender, FrameEventArgs args)
        {
            // Get the most recent frame and report some basic information            
            Leap.Frame frame = args.frame;
            if (prevTime + 1000000 < frame.Timestamp | prevTime == 0)
            {
                Console.Clear();
                Console.WriteLine(
                  "Frame id: {0}, timestamp: {1}, hands: {2}",
                  frame.Id, frame.Timestamp, frame.Hands.Count
                );
                foreach (Hand hand in frame.Hands)
                {
                    Console.WriteLine("  Hand id: {0}, palm position: {1}, fingers: {2}",
                      hand.Id, hand.PalmPosition, hand.Fingers.Count);
                    // Get the hand's normal vector and direction
                    Leap.Vector normal = hand.PalmNormal;
                    Leap.Vector direction = hand.Direction;

                    // Calculate the hand's pitch, roll, and yaw angles
                    Console.WriteLine(
                      "  Hand pitch: {0} degrees, roll: {1} degrees, yaw: {2} degrees",
                      direction.Pitch * 180.0f / (float)Math.PI,
                      normal.Roll * 180.0f / (float)Math.PI,
                      direction.Yaw * 180.0f / (float)Math.PI
                    );
                }
                prevTime = frame.Timestamp;
            }
        }
    }
}
