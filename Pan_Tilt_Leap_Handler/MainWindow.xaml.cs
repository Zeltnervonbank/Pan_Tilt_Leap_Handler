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
using System.IO;
using Leap;
using System.Threading;

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
        bool fingerExtended = false;

        byte[] bytestosend = new byte[3];
        SerialPort serialPort = new SerialPort();
        Controller controller = new Controller();
        
        string rootDir = AppDomain.CurrentDomain.BaseDirectory;
        int panMax = 0;
        int panMin = 0;
        int tiltMax = 0;
        int tiltMin = 0;
        int panCurr = 0;
        int tiltCurr = 0;

        string aPos = "A: ";
        string bPos = "B: ";

        public MainWindow()
        {
            InitializeComponent();
            string[] configLines = File.ReadAllLines(rootDir + "\\Config.cfg");
            panMax = Convert.ToInt32(configLines[0]);
            panMin = Convert.ToInt32(configLines[1]);
            tiltMax = Convert.ToInt32(configLines[2]);
            tiltMin = Convert.ToInt32(configLines[3]);

            panSlider.Maximum = panMax;
            panSlider.Minimum = panMin;
            panSlider.Value = 0;

            tiltSlider.Maximum = tiltMax;
            tiltSlider.Minimum = tiltMin;
            tiltSlider.Value = 0;
                        
            serialPort.DataReceived += SerialPort_DataReceived;
            string[] ports = SerialPort.GetPortNames();
            foreach (string s in ports)
                COMSelBox.Items.Add(s);
            if(COMSelBox.HasItems)
                COMSelBox.SelectedItem = COMSelBox.Items[0];

            int[] baudRates = { 9600, 14400, 19200, 38400, 57600, 115200, 128000, 256000 };
            foreach(int i in baudRates)
                baudRateComboBox.Items.Add(i);
            baudRateComboBox.SelectedItem = baudRates[0];

            serialPortConnectButton.IsEnabled = !serialPort.IsOpen;
            serialPortDisconnectButton.IsEnabled = serialPort.IsOpen;            
            controller.FrameReady += OnFrame;
        }

        private void SerialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {            
            /*try
            {
                SerialPort sp = (SerialPort)sender;

                byte[] receivedBytes = Encoding.ASCII.GetBytes(sp.ReadExisting());
                for (int i = 0; i < receivedBytes.Length; i++)
                {
                    if (receivedBytes[i] == 97)
                    {
                        int j = (receivedBytes[i + 1] << 8) + receivedBytes[i + 2];
                        Console.WriteLine("A: " + j);
                        aPos = "A: " + j;
                    }
                    if (receivedBytes[i] == 98)
                    {
                        int j = (receivedBytes[i + 1] << 8) + receivedBytes[i + 2];
                        Console.WriteLine("B: " + j);
                        bPos = "B: " + j;
                    }
                }                
            }
            catch(Exception ex)
            {
                Application.Current.Dispatcher.Invoke(new ThreadStart(() =>
                {
                    MainWindow window = Application.Current.MainWindow as MainWindow;
                    window.logBox.Text = ex.Message;
                }));
            }

            Application.Current.Dispatcher.Invoke(new ThreadStart(() =>
            {
                MainWindow window = Application.Current.MainWindow as MainWindow;
                window.aPosLabel.Content = aPos;
                window.bPosLabel.Content = bPos;
            }));*/

        }
        public void homeRobot(bool resetGUI)
        {
            sendUART(0x03, 0x00, 0x00);
            if(resetGUI)
            {
                updatePanAngleBox("0");
                updateTiltAngleBox("0");
                updateSliders(0, 0);
            }
        }

        public void OnFrame(object sender, FrameEventArgs args)
        {
            if (serialPort.IsOpen)
            {
                // Get the most recent frame and report some basic information            
                Leap.Frame frame = args.frame;
                if (prevTime + 100000 < frame.Timestamp | prevTime == 0)
                {

                    if (frame.Hands.Count != 0)
                    {
                        if (fingerExtended)
                        {
                            //Pitch
                            double pitchAngle = -frame.Hands[0].Direction.Pitch * 180.0 / Math.PI;
                            if(pitchAngle > tiltMax)
                            {
                                pitchAngle = tiltMax;
                            }
                            else if(pitchAngle < tiltMin)
                            {
                                pitchAngle = tiltMin;
                            }
                            pitch = Convert.ToInt32(pitchAngle * 3.0) + 1080;                            
                            sendUART(0x02, (byte)(0xFF & (pitch >> 8)), (byte)(0xFF & pitch));

                            //Yaw
                            double yawAngle = frame.Hands[0].Direction.Yaw * 180.0 / Math.PI;
                            if (yawAngle > panMax)
                            {
                                yawAngle = panMax;
                            }
                            else if (yawAngle < panMin)
                            {
                                yawAngle = panMin;
                            }
                            yaw = Convert.ToInt32(yawAngle * 3.0) + 1080;                            
                            sendUART(0x01, (byte)(0xFF & (yaw >> 8)), (byte)(0xFF & yaw));

                            updateTiltAngleBox(Convert.ToInt32(pitchAngle).ToString());
                            updatePanAngleBox(Convert.ToInt32(yawAngle).ToString());
                        }
                        else
                        {
                            homeRobot(true);
                        }

                        Hand hand = frame.Hands[0];
                        Finger index = hand.Fingers[1];

                        fingerExtended = hand.Fingers[1].IsExtended &
                                         hand.Fingers[2].IsExtended &
                                         hand.Fingers[3].IsExtended &
                                         hand.Fingers[4].IsExtended;
                    }
                    else
                    {                        
                        homeRobot(true);
                    }

                    Console.Write(
                      "\rFrame id: {0}, timestamp: {1}, hands: {2}, pitch {3}\t yaw:{4}\t fingers: {5}\t            ",
                      frame.Id, frame.Timestamp, frame.Hands.Count, pitch, yaw, fingerExtended
                    );
                    prevTime = frame.Timestamp;
                }
            }
        }
   
        private void COMSelBox_DropDownOpened(object sender, EventArgs e)
        {
            string[] ports = SerialPort.GetPortNames();
            if (ports.Length > 0)
            {
                COMSelBox.Items.Clear();
                foreach (string s in ports)
                {
                    COMSelBox.Items.Add(s);
                }
                COMSelBox.SelectedItem = COMSelBox.Items[0];
            }
        }

        private void serialPortConnectButton_Click(object sender, RoutedEventArgs e)
        {
            if (!serialPort.IsOpen)
            {
                serialPort.BaudRate = Convert.ToInt32(baudRateComboBox.SelectedItem);
                serialPort.PortName = COMSelBox.SelectedItem.ToString();
                serialPort.Open();
                serialPortConnectButton.IsEnabled = !serialPort.IsOpen;
                serialPortDisconnectButton.IsEnabled = serialPort.IsOpen;
            }
        }

        private void serialPortDisconnectButton_Click(object sender, RoutedEventArgs e)
        {
            if (serialPort.IsOpen)
            {
                serialPort.Close();
                serialPortConnectButton.IsEnabled = !serialPort.IsOpen;
                serialPortDisconnectButton.IsEnabled = serialPort.IsOpen;
            }
            
        }
        private void setUIState(bool state)
        {
            panSlider.IsEnabled = state;
            tiltSlider.IsEnabled = state;
            panAngleBox.IsEnabled = state;
            tiltAngleBox.IsEnabled = state;
            setAngleButton.IsEnabled = state;
            panMinusButton.IsEnabled = state;
            panPlusButton.IsEnabled = state;
            tiltMinusButton.IsEnabled = state;
            tiltPlusButton.IsEnabled = state;
            offsetValueBox.IsEnabled = state;
        }
        
        public void sendUART(byte cmd, byte hDat, byte lDat)
        {
            if (serialPort.IsOpen)
            {
                byte[] command = { cmd, hDat, lDat };
                serialPort.Write(command, 0, command.Length);
            }
        }

        private void panPlusButton_Click(object sender, RoutedEventArgs e)
        {
            int offset = Convert.ToInt32(offsetValueBox.Text);
            sendUART(0x0A,(byte)(0xFF & (offset >> 8)), (byte)(0xFF & offset));
        }        

        private void panMinusButton_Click(object sender, RoutedEventArgs e)
        {
            int offset = Convert.ToInt32(offsetValueBox.Text);
            sendUART(0x0B, (byte)(0xFF & (offset >> 8)), (byte)(0xFF & offset));
        }

        private void tiltPlusButton_Click(object sender, RoutedEventArgs e)
        {
            int offset = Convert.ToInt32(offsetValueBox.Text);
            sendUART(0x0C, (byte)(0xFF & (offset >> 8)), (byte)(0xFF & offset));
        }

        private void tiltMinusButton_Click(object sender, RoutedEventArgs e)
        {
            int offset = Convert.ToInt32(offsetValueBox.Text);
            sendUART(0x0D, (byte)(0xFF & (offset >> 8)), (byte)(0xFF & offset));
        }

        private void homeButton_Click(object sender, RoutedEventArgs e)
        {
            homeRobot(true);
        }

        private void setAngleButton_Click(object sender, RoutedEventArgs e)
        {
            int yawOffset = Convert.ToInt32(panAngleBox.Text);
            int pitchOffset = Convert.ToInt32(tiltAngleBox.Text);

            if (yawOffset > panMax)
            {
                logBox.Text = string.Format("Pan angle too high, max is {0}", panMax);
            }
            else if (yawOffset < panMin)
            {
                logBox.Text = string.Format("Pan angle too low, min is {0}", panMin);
            }
            else if (pitchOffset > tiltMax)
            {
                logBox.Text = string.Format("Tilt angle too high, max is {0}", tiltMax);
            }
            else if (pitchOffset < tiltMin)
            {
                logBox.Text = string.Format("Tilt angle too low, min is {0}", tiltMin);
            }
            else
            {
                updateSliders(yawOffset, pitchOffset);

                yawOffset = (yawOffset * 3) + 1080;
                pitchOffset = (pitchOffset * 3) + 1080;                

                sendUART(0x01, (byte)(0xFF & (yawOffset >> 8)), (byte)(0xFF & yawOffset));
                sendUART(0x02, (byte)(0xFF & (pitchOffset >> 8)), (byte)(0xFF & pitchOffset));
            }
        }

        /*private void sendPosButton_Click(object sender, RoutedEventArgs e)
        {
            if(!sendPosButtonEnabled)
            {
                sendUART(0x0E, 0x00, 0x00);

                sendPosButtonEnabled = true;
                sendPosButton.Content = "Sending Pos";
            }
            else
            {
                sendUART(0x0F, 0x00, 0x00);

                sendPosButtonEnabled = false;
                sendPosButton.Content = "Not Sending Pos";
            }
        }*/

        private void panSlider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            updatePanAngleBox(Convert.ToInt32(panSlider.Value).ToString());
            int panVal = Convert.ToInt32(panSlider.Value) * 3 + 1080;
            sendUART(0x01, (byte)(0xFF & (panVal >> 8)), (byte)(0xFF & panVal));
        }

        private void tiltSlider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {            
            updateTiltAngleBox(Convert.ToInt32(tiltSlider.Value).ToString());
            int tiltVal = Convert.ToInt32(tiltSlider.Value) * 3 + 1080;
            sendUART(0x02, (byte)(0xFF & (tiltVal >> 8)), (byte)(0xFF & tiltVal));
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            serialPort.Close();
            controller.StopConnection();
            Application.Current.Shutdown();
        }

        private void leapEnabledCheck_Click(object sender, RoutedEventArgs e)
        {
            setUIState(!(bool)leapEnabledCheck.IsChecked);
            if ((bool)leapEnabledCheck.IsChecked)
                controller.StartConnection();                            
            else
                controller.StopConnection();
        }

        private void resetModeCheck_Click(object sender, RoutedEventArgs e)
        {
            if (serialPort.IsOpen)
            {
                if ((bool)resetModeCheck.IsChecked)
                    sendUART(0x08, 0x00, 0x00);
                else
                    sendUART(0x09, 0x00, 0x00);
            }
        }
        private void updatePanAngleBox(string newVal)
        {
            Application.Current.Dispatcher.Invoke(new ThreadStart(() =>
            {
                MainWindow window = Application.Current.MainWindow as MainWindow;
                window.panAngleBox.Text = newVal;
            }));
        }
        private void updateTiltAngleBox(string newVal)
        {
            Application.Current.Dispatcher.Invoke(new ThreadStart(() =>
            {
                MainWindow window = Application.Current.MainWindow as MainWindow;
                window.tiltAngleBox.Text = newVal;
            }));
        }
        private void updateLogBox(string newVal)
        {
            Application.Current.Dispatcher.Invoke(new ThreadStart(() =>
            {
                MainWindow window = Application.Current.MainWindow as MainWindow;
                window.logBox.Text = newVal;
            }));
        }
        private void updateSliders(double panVal, double tiltVal)
        {
            Application.Current.Dispatcher.Invoke(new ThreadStart(() =>
            {
                MainWindow window = Application.Current.MainWindow as MainWindow;
                panSlider.Value = panVal;
                tiltSlider.Value = tiltVal;
            }));
        }

        private void angleBox_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
            {
                setAngleButton_Click(sender, e);
            }
        }

        private void panAngleBox_GotKeyboardFocus(object sender, KeyboardFocusChangedEventArgs e)
        {
            panAngleBox.SelectAll();
        }

        private void tiltAngleBox_GotKeyboardFocus(object sender, KeyboardFocusChangedEventArgs e)
        {
            tiltAngleBox.SelectAll();
        }
    }
}
