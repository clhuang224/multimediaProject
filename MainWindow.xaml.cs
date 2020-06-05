using KinectCoordinateMapping.Utilities;
using Microsoft.Kinect;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
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
using System.Windows.Threading;

namespace KinectCoordinateMapping
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        KinectSensor _sensor;
        MultiSourceFrameReader _reader;
        IList<Body> _bodies;
        List<map> mapList = new List<map>();
        List<map> hitting = new List<map>();
        HandState left_last_state, left_current_state;
        HandState right_last_state, right_current_state;
        Point left_current_position;
        Point right_current_position;
        string left_state;
        DispatcherTimer _timer = new DispatcherTimer();
        DateTime timeStart;
        Random random = new Random();

        CameraMode _mode = CameraMode.Color;

        public MainWindow()
        {
            InitializeComponent();
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            _sensor = KinectSensor.GetDefault();

            if (_sensor != null)
            {
                _sensor.Open();

                _reader = _sensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth | FrameSourceTypes.Infrared | FrameSourceTypes.Body);
                _reader.MultiSourceFrameArrived += Reader_MultiSourceFrameArrived;
            }

            
            for(int i = 1; i < 10; i++)
            {
                map newcircle = new map();
                newcircle.second = i * 2;
                newcircle.X = random.Next(200, 1000);
                newcircle.Y = random.Next(200, 500);
                newcircle.num = i;
                mapList.Add(newcircle);
            }

            _timer.Interval = TimeSpan.FromMilliseconds(2000);
            _timer.Tick += _timer_Tick;
        }

        private void btn_start_Click(object sender, RoutedEventArgs e)
        {
            _timer.Start();
            timeStart = DateTime.Now;
        }
        void _timer_Tick(object sender, EventArgs e)
        {
            if (mapList.Count > 0)
            {
                label1.Content = (DateTime.Now - timeStart).TotalMilliseconds.ToString();
                map nextcircle = mapList[0];
                drawCircle(nextcircle.X, nextcircle.Y, nextcircle.num);
                hitting.Add(nextcircle);
                mapList.RemoveAt(0);
            }
            
            
        }

        private void Window_Closed(object sender, EventArgs e)
        {
            if (_reader != null)
            {
                _reader.Dispose();
            }

            if (_sensor != null)
            {
                _sensor.Close();
            }
        }

        void drawCircle(int x,int y,int num)
        {
            Ellipse ellipse = new Ellipse
            {
                Fill = Brushes.Pink,
                Width = 100,
                Height = 100,
                StrokeThickness = 3,
                Stroke = Brushes.Red
            };

            Canvas.SetLeft(ellipse, x - ellipse.Width / 2);
            Canvas.SetTop(ellipse, y - ellipse.Height / 2);
            map.Children.Add(ellipse);

            TextBlock textBlock = new TextBlock
            {
                Text = num.ToString(),
                TextAlignment = TextAlignment.Center,
                VerticalAlignment = VerticalAlignment.Center,
                FontSize = 60,
                FontWeight = FontWeights.Bold
            };

            Canvas.SetLeft(textBlock, x - ellipse.Width / 6);
            Canvas.SetTop(textBlock, y - ellipse.Height / 2.7);
            map.Children.Add(textBlock);
        }

        void hit(double handX,double handY)
        {
            if (hitting.Count > 0)
            {
                map nexthitcircle = hitting[0];
                if (Math.Sqrt((handX - nexthitcircle.X) * (handX - nexthitcircle.X) + (handY - nexthitcircle.Y) * (handY - nexthitcircle.Y)) < 100)
                {
                    hitting.RemoveAt(0);
                    map.Children.RemoveAt(0);
                    map.Children.RemoveAt(0);

                }
            }
            
        }
        void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            var reference = e.FrameReference.AcquireFrame();

            // Color
            using (var frame = reference.ColorFrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    if (_mode == CameraMode.Color)
                    {
                        camera.Source = frame.ToBitmap();
                    }
                }
            }

            // Depth
            using (var frame = reference.DepthFrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    if (_mode == CameraMode.Depth)
                    {
                        camera.Source = frame.ToBitmap();
                    }
                }
            }

            // Infrared
            using (var frame = reference.InfraredFrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    if (_mode == CameraMode.Infrared)
                    {
                        camera.Source = frame.ToBitmap();
                    }
                }
            }

            // Body
            using (var frame = reference.BodyFrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    canvas.Children.Clear();

                    _bodies = new Body[frame.BodyFrameSource.BodyCount];
                    
                    frame.GetAndRefreshBodyData(_bodies);

                    

                    foreach (var body in _bodies)
                    {
                        //Debug.Write(body.Joints.Values);
                        if (body.IsTracked)
                        {
                            // COORDINATE MAPPING
                            foreach (Joint joint in body.Joints.Values)
                            {
                                if (joint.JointType == JointType.HandLeft || joint.JointType == JointType.HandRight || joint.JointType == JointType.HandTipLeft || joint.JointType == JointType.HandTipRight)
                                {
                                    if (joint.TrackingState == TrackingState.Tracked)
                                    {
                                        // 3D space point
                                        CameraSpacePoint jointPosition = joint.Position;

                                        // 2D space point
                                        Point point = new Point();

                                        if (_mode == CameraMode.Color)
                                        {
                                            ColorSpacePoint colorPoint = _sensor.CoordinateMapper.MapCameraPointToColorSpace(jointPosition);

                                            point.X = float.IsInfinity(colorPoint.X) ? 0 : colorPoint.X;
                                            point.Y = float.IsInfinity(colorPoint.Y) ? 0 : colorPoint.Y;
                                        }
                                        else if (_mode == CameraMode.Depth || _mode == CameraMode.Infrared) // Change the Image and Canvas dimensions to 512x424
                                        {
                                            DepthSpacePoint depthPoint = _sensor.CoordinateMapper.MapCameraPointToDepthSpace(jointPosition);

                                            point.X = float.IsInfinity(depthPoint.X) ? 0 : depthPoint.X;
                                            point.Y = float.IsInfinity(depthPoint.Y) ? 0 : depthPoint.Y;
                                        }



                                        // Draw
                                        Ellipse ellipse = new Ellipse
                                        {
                                            Fill = Brushes.Red,
                                            Width = 30,
                                            Height = 30
                                        };


                                        if (joint.JointType == JointType.HandLeft)
                                        {
                                            position.Content = (point.X / 775 * 512) + " " + (point.Y / 640 * 424);
                                            left_current_position = new Point(point.X / 775 * 512, point.Y / 640 * 424);

                                            left_last_state = left_current_state;
                                            left_current_state = body.HandLeftState;
                                            hand_left_state.Content = body.HandLeftState;

                                            if (left_last_state == HandState.Open && left_current_state == HandState.Closed)
                                            {
                                                left_state = "左手握拳";
                                                grab.Content = left_state;
                                                hit((point.X / 775 * 512), (point.Y / 640 * 424));
                                            }
                                            else if (left_last_state == HandState.Closed && left_current_state == HandState.Open)
                                            {
                                                left_state = "左手張開";
                                                grab.Content = left_state;
                                            }
                                        }
                                        if (joint.JointType == JointType.HandRight)
                                        {
                                            position.Content = (point.X / 775 * 512) + " " + (point.Y / 640 * 424);
                                            right_current_position = new Point(point.X / 775 * 512, point.Y / 640 * 424);

                                            right_last_state = right_current_state;
                                            right_current_state = body.HandRightState;

                                            if (right_last_state == HandState.Open && right_current_state == HandState.Closed)
                                            {
                                                hit((point.X / 775 * 512), (point.Y / 640 * 424));
                                            }
                                        }

                                        Canvas.SetLeft(ellipse, (point.X - ellipse.Width / 2) / 775 * 512);
                                        Canvas.SetTop(ellipse, (point.Y - ellipse.Height / 2) / 640 * 424);

                                        canvas.Children.Add(ellipse);
                                    }
                                }
                                
                            }
                        }
                    }
                }
            }
        }
    }

    enum CameraMode
    {
        Color,
        Depth,
        Infrared
    }
}
