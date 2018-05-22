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
using Microsoft.Win32;
using System.Timers;
using System.Diagnostics;

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

        string[] pathLines;
        List<List<pathCommand>> commandList = new List<List<pathCommand>>();
        int pathIndex = 0;
        bool pathFollow = false;
        
        string rootDir = AppDomain.CurrentDomain.BaseDirectory;
        int panMax = 0;
        int panMin = 0;
        int tiltMax = 0;
        int tiltMin = 0;
        int panCurr = 0;
        int tiltCurr = 0;

        int expectedPan = 2048;
        int expectedTilt = 2048;

        int resendCounter = 0;
        
        List<string> logLines = new List<string>();
        List<string> cmdLogLines = new List<string>();
        List<string> timingLogLines = new List<string>();
        Stopwatch st = new Stopwatch();

        System.Timers.Timer pathTimer = new System.Timers.Timer(10);

        public struct pathCommand
        {
            public byte cmd, hDat, lDat;
            public int delay;
            
            public pathCommand(byte Cmd, byte HDat, byte LDat, int Delay)
            {
                cmd = Cmd;
                hDat = HDat;
                lDat = LDat;
                delay = Delay;
            }
        }

        public MainWindow()
        {
            InitializeComponent();
            controller.StopConnection();
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
            {*/
                SerialPort sp = (SerialPort)sender;
                if (sp.BytesToRead >= 3)
                {
                    byte[] receivedBytes = new byte[3];
                    while (true)
                    {
                        byte firstByte = (byte)sp.ReadByte();
                        if (firstByte == 0x1E || firstByte == 0x1F || firstByte == 0xFF)
                        {
                            receivedBytes[0] = firstByte;
                            receivedBytes[1] = (byte)sp.ReadByte();
                            receivedBytes[2] = (byte)sp.ReadByte();
                            break;
                        }
                    }

                    int pos = (receivedBytes[1] << 8) + receivedBytes[2];
                    if (receivedBytes[0] == 0x1E)
                    {
                        panCurr = pos;
                    }
                    else if (receivedBytes[0] == 0x1F)
                    {
                        tiltCurr = pos;
                    }
                    string s = string.Format("{0} {1} {2} - {3} - {4}", receivedBytes[0], receivedBytes[1], receivedBytes[2], pos, DateTime.Now);
                    logLines.Add(s);
                }
            /*}
            catch(Exception ex)
            {                
                updateLogBox(ex.Message);
            }*/
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
            File.WriteAllLines(rootDir + "\\Uartlog.txt", logLines);
            File.WriteAllLines(rootDir + "\\CmdLog.txt", cmdLogLines);
            File.WriteAllLines(rootDir + "\\TimingLog.txt", timingLogLines);
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
        private void updateProgressBar(int val)
        {
            Application.Current.Dispatcher.Invoke(new ThreadStart(() =>
            {
                MainWindow window = Application.Current.MainWindow as MainWindow;
                window.pathProgressBar.Value = val;
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

        private void openPathButton_Click(object sender, RoutedEventArgs e)
        {
            OpenFileDialog dialog = new OpenFileDialog();
            if (dialog.ShowDialog() == true)
            {
                commandList.Clear();
                pathLines = File.ReadAllLines(dialog.FileName);
                updateLogBox(string.Format("Loaded {0} lines from path file.", pathLines.Length));
                foreach(string line in pathLines)
                {
                    commandList.AddRange(decodeLine(line));
                }
                pathProgressBar.Maximum = commandList.Count;
            }
            pathIndex = 0;            
        }

        private byte[] hexConvert(int i)
        {
            return new byte[] { (byte)(0xFF & (i >> 8)), (byte)(0xFF & i) };
        }
        private void followPathButton_Click(object sender, RoutedEventArgs e)
        {
            if (!pathFollow)
            {
                pathFollow = true;
                //sendUART(0x0E, 0x00, 0x00);
                pathTimer.Elapsed += PathTimer_Elapsed;
                pathTimer.Start();
            }
        }

        private void PathTimer_Elapsed(object sender, ElapsedEventArgs e)
        {
            pathTimer.Stop(); //Stop the timer while executing
            if(pathIndex == commandList.Count) //If at last command
            {
                pathIndex = 0; //Reset index
                pathFollow = false; //Reenable button
                Task t = new Task(() => { updateProgressBar(pathIndex); }); //Reset progress bar
                t.Start();
                //sendUART(0x0F, 0x00, 0x00);
                return; //Break out without restarting timer
            }
            //If within 5 steps of expected position, or if index == 0
            /*if ((Math.Abs(panCurr - expectedPan) <= 5 && Math.Abs(tiltCurr - expectedTilt) <= 5) || pathIndex == 0)
            {*/
                st.Stop();
                timingLogLines.Add(String.Format("{0} - {1} - {2}", pathIndex, DateTime.Now, st.ElapsedMilliseconds));
                st.Reset();
                st.Start();
                pathModeSendCommand(pathIndex);
                pathIndex++; //Increment the index
                Task r = new Task(() => { updateProgressBar(pathIndex); }); ; //Update the progress bar to new index value
                r.Start();
            /*}
            if (resendCounter == 100)
            {
                pathModeSendCommand(pathIndex - 1);
                resendCounter = 0;
            }
            else
            {
                resendCounter++;
            }*/
            //Restart timer, this is the only thing that happens if none of the above apply
            //This happens if the system is still moving
            pathTimer.Start(); 
        }
        private void pathModeSendCommand(int index)
        {
            foreach (pathCommand p in commandList[index]) //For each movement in the command
            {
                if (p.cmd != 0) //If command isn't a delay
                {
                    sendUART(p.cmd, p.hDat, p.lDat); //Send that command over UART
                    switch (p.cmd)
                    {
                        case 1: //If pan movement set expected pan location
                            expectedPan = ((p.hDat << 8) + p.lDat) + 2048 - 1080;
                            break;
                        case 2: //If tilt movement set expected tilt location
                            expectedTilt = ((p.hDat << 8) + p.lDat) + 2048 - 1080;
                            break;
                        case 3: //If home movement set expected locations
                            expectedPan = expectedTilt = 2048;
                            break;
                    }
                }
                if (p.delay == 0) //If the command's delay was 0
                {
                    pathTimer.Interval = 10; //Insert default delay
                }
                else //If the command had a delay value
                {
                    pathTimer.Interval = p.delay; //Set that delay value
                }
                string s = String.Format("{0} - {1}: {2} {3} {4} - {5}", pathIndex, DateTime.Now, p.cmd, p.hDat, p.lDat, p.delay);
                cmdLogLines.Add(s);
            }
            updateLogBox(String.Format("Sent command nr {0}", index));
        }

        private pathCommand packCommand(byte cmd, int pos, int delay)
        {
            pathCommand pCommand = new pathCommand();
            pCommand.cmd = cmd;
            byte[] bytes = hexConvert(pos + 1080);
            pCommand.hDat = bytes[0];
            pCommand.lDat = bytes[1];
            pCommand.delay = delay;
            return pCommand;
        }

        private List<List<pathCommand>> decodeLine(string line)
        {
            string[] words = line.Split(' '); //Splits the line into "words" separated by spaces
            List<List<pathCommand>> command = new List<List<pathCommand>>(); //Creates new empty list
            switch (words[0].ToLower())
            {
                case "mov":
                    {
                        int panAngle = (Convert.ToInt32(words[1]) * 3); //Get the pan angle 
                        int tiltAngle = (Convert.ToInt32(words[2]) * 3); //Get the tilt angle

                        List<pathCommand> moveCommand = new List<pathCommand>(); //Make new list of movements
                        moveCommand.Add(packCommand(0x01, panAngle, 0)); //Add pan movement
                        moveCommand.Add(packCommand(0x02, tiltAngle, 0)); //Add tilt movement
                        command.Add(moveCommand); //Adds the movements to the command list
                        break;
                    }
                case "slp":
                    {
                        int delay = Convert.ToInt32(words[1]); //Get the delay value from word list
                        command.Add(new List<pathCommand>(new pathCommand[] { packCommand(0x00, -1080, delay) })); //Add command to list
                        break;
                    }
                case "smth":
                    {
                        //Read variables from line
                        //Beginning and end angles converted to steps
                        double x1 = Convert.ToDouble(words[1]) * 3.0;
                        double x2 = Convert.ToDouble(words[2]) * 3.0;
                        double y1 = Convert.ToDouble(words[3]) * 3.0;
                        double y2 = Convert.ToDouble(words[4]) * 3.0;


                        //Stepsize and delay between steps
                        double stepSize = Convert.ToInt32(words[5]);
                        int delay = Convert.ToInt32(words[6]);
                        
                        double dX = x2 - x1;
                        double dY = y2 - y1;

                        double moveDistance = Math.Sqrt(dX * dX + dY * dY);

                        double stepCount = (moveDistance / stepSize);

                        for(int i = 0; i <= (int)stepCount; i++)
                        {
                            int xPos = (int)((stepSize * (double)i) * (1.0 / moveDistance) * dX + x1);
                            int yPos = (int)((stepSize * (double)i) * (1.0 / moveDistance) * dY + y1);
                            command.Add(new List<pathCommand>(new pathCommand[] { packCommand(0x01, xPos, delay), packCommand(0x02, yPos, delay) }));
                        }
                        command.Add(new List<pathCommand>(new pathCommand[] { packCommand(0x01, (int)x2, delay), packCommand(0x02, (int)y2, delay) }));
                        break;
                        #region commented out
                        /*
                        //Equation variables
                        double numerator;
                        double denominator;
                        double slope;
                        double b;

                        if (Math.Abs((int)x1 - (int)x2) > Math.Abs((int)y1 - (int)y2)) //If X movement is larger than Y movement
                        {
                            //Calculate equation
                            numerator = y2 - y1;
                            denominator = x2 - x1;

                            slope = numerator / denominator;
                            b = -(slope * x1 - y1);

                            if (x1 < x2) //If going from low value to high
                            {
                                for (int i = (int)x1; i <= (int)x2; i += stepSize)
                                {
                                    int yStep = (int)(slope * i + b);
                                    command.Add(new List<pathCommand>(new pathCommand[] { packCommand(0x01, i, 10), packCommand(0x02, yStep, 10) }));
                                }
                            }
                            else if (x1 > x2) //If going from high value to low
                            {
                                for (int i = (int)x1; i >= (int)x2; i -= stepSize)
                                {
                                    int yStep = (int)(slope * i + b);
                                    command.Add(new List<pathCommand>(new pathCommand[] { packCommand(0x01, i, 10), packCommand(0x02, yStep, 10) }));
                                }
                            }
                        }
                        else //If Y movement is larger than X movement
                        {
                            //Calculate transposed equation
                            numerator = x2 - x1;
                            denominator = y2 - y1;

                            slope = numerator / denominator;
                            b = -(slope * y1 - x1);

                            if (y1 < y2) //If going from low value to high
                            {
                                for (int i = (int)y1; i <= (int)y2; i += stepSize)
                                {
                                    int xStep = (int)(slope * i + b);
                                    command.Add(new List<pathCommand>(new pathCommand[] { packCommand(0x01, xStep, 10), packCommand(0x02, i, 10) }));
                                }
                            }
                            else if (y1 > y2) //If going from high value to low
                            {
                                for (int i = (int)y1; i >= (int)y2; i -= stepSize)
                                {
                                    int xStep = (int)(slope * i + b);
                                    command.Add(new List<pathCommand>(new pathCommand[] { packCommand(0x01, xStep, 10), packCommand(0x02, i, 10) }));
                                }
                            }
                        }
                        break;*/
#endregion
                    }
                case "home":
                    {
                        command.Add(new List<pathCommand>(new pathCommand[] { packCommand(0x03, -1080, 0) }));
                        break;
                    }
                case "//":
                    {
                        return new List<List<pathCommand>>();                        
                    }
            }
            //Finally return the list of commands
            return command;
        }
    }
}
