#define HAS_OPENCV

using System.Runtime.InteropServices;
using static lx.LxCamera;
using System.Net.Http.Headers;

#if HAS_OPENCV
using OpenCvSharp; 
#endif





namespace demo
{
    public unsafe static class CvConstants
    {
        public const int CV_CN_SHIFT = 3;
        public const int CV_DEPTH_MAX = (1 << CV_CN_SHIFT);
        public const int CV_MAT_DEPTH_MASK = (CV_DEPTH_MAX - 1);

        public static int CV_MAT_DEPTH(int flags)
        {
            return flags & CV_MAT_DEPTH_MASK;
        }

        public static int CV_MAKETYPE(int depth, int cn)
        {
            return CV_MAT_DEPTH(depth) + ((cn - 1) << CV_CN_SHIFT);
        }

        public static int CV_MAKE_TYPE(int depth, int cn)
        {
            return CV_MAKETYPE(depth, cn);
        }
    }

    unsafe class demo
    {
        static int Main()
        {
            int device_num = 0;
            long handle = 0;
            Console.WriteLine(DcGetApiVersion());

            //设置日志与输出
            if (DcSetInfoOutput(2, false, "./log/") == LX_STATE.LX_SUCCESS) Console.WriteLine("success");
            else Console.WriteLine("failed");

            //寻找设备并列出所有设备信息
            IntPtr list = new IntPtr();
            while (device_num == 0)
            {
                if (DcGetDeviceList(out list, ref device_num) == LX_STATE.LX_SUCCESS)
                {
                    Console.WriteLine("find {0} device", device_num);
                    for (int i = 0; i < device_num; i++)
                    {
                        Console.WriteLine("\n-------------------------");
                        IntPtr pPonitor = new IntPtr(list.ToInt64() + Marshal.SizeOf(typeof(LxDeviceInfo)) * i);
                        LxDeviceInfo _temp = (LxDeviceInfo)Marshal.PtrToStructure(pPonitor, typeof(LxDeviceInfo));
                        System.Console.WriteLine(_temp.id);
                        System.Console.WriteLine(_temp.ip);
                        System.Console.WriteLine(_temp.sn);
                        System.Console.WriteLine(_temp.mac);
                        System.Console.WriteLine(_temp.firmware_ver);
                        System.Console.WriteLine(_temp.algor_ver);
                        System.Console.WriteLine(_temp.handle);
                        System.Console.WriteLine(_temp.dev_type);
                        Console.WriteLine("-------------------------\n\n");
                    }
                }
                else
                {
                    Console.WriteLine("find device failed");
                    Thread.Sleep(1000);
                }
            }

            //通过索引方式连接设备
            LxDeviceInfo info = new LxDeviceInfo();
            IntPtr pBuff = Marshal.AllocHGlobal(Marshal.SizeOf(info));
            Marshal.StructureToPtr(info, pBuff, true);
            while (true)
            {
                LX_STATE ret = DcOpenDevice(LX_OPEN_MODE.OPEN_BY_INDEX, "0", ref handle, pBuff);
                if (ret == LX_STATE.LX_SUCCESS)
                {
                    info = (LxDeviceInfo)Marshal.PtrToStructure(pBuff, typeof(LxDeviceInfo));
                    System.Console.WriteLine("connect");
                    System.Console.WriteLine(info.id);
                    break;
                }
                else Console.WriteLine("open device failed: {0}", ret);
                Thread.Sleep(1000);
            }
            char wait_key='0';
            void WaitKey()
            {
                Console.WriteLine("*********press 'q' to exit*********");
                ConsoleKeyInfo keyInfo = Console.ReadKey();
                wait_key = keyInfo.KeyChar;


            }
            //设置Int参数数据
            //if (DcSetIntValue(handle, 1001, 1001) == LX_STATE.LX_SUCCESS)
            //    Console.WriteLine("SetIntValue success");
            //else Console.WriteLine("SetIntValue failed");

            //获取Int参数数据
            //LxIntValueInfo i_value = new LxIntValueInfo();
            //if (DcGetIntValue(handle, 1001, ref i_value) == LX_STATE.LX_SUCCESS)
            //    Console.WriteLine("DcGetIntValue success:\nis enable:{0}\nmin:{1}\nmax:{2}\ncurrect:{3}\n",
            //        i_value.set_available, i_value.min_value, i_value.max_value, i_value.cur_value);
            //else Console.WriteLine("DcGetIntValue failed");

            //设置float参数数据
            //if (DcSetFloatValue(handle, 2001, (float)0.25) == LX_STATE.LX_SUCCESS)
            //    Console.WriteLine("DcSetFloatValue success");
            //else Console.WriteLine("DcSetFloatValue failed");

            //获取float参数数据
            //LxFloatValueInfo f_value = new LxFloatValueInfo();
            //if (DcGetFloatValue(handle, 2001, ref f_value) == LX_STATE.LX_SUCCESS)
            //    Console.WriteLine("DcSetFloatValue success:is enable:{0}\nmin:{1}\nmax:{2}\ncurrect:{3}\n",
            //        f_value.set_available, f_value.min_value, f_value.max_value, f_value.cur_value);
            //else Console.WriteLine("DcSetFloatValue failed");

            //设置bool参数数据
            //if (DcSetBoolValue(handle, 3002, false) == LX_STATE.LX_SUCCESS)
            //    Console.WriteLine("DcSetBoolValue success");
            //else Console.WriteLine("DcSetBoolValue failed");

            //获取bool参数数据
            //bool b_value = false;
            //if (DcGetBoolValue(handle, 3002, ref b_value) == LX_STATE.LX_SUCCESS)
            //    Console.WriteLine("DcGetBoolValue success");
            //else Console.WriteLine("DcGetBoolValue failed");

            //设置string参数数据
            //LX_STATE fin = DcSetStringValue(handle, 4003, "123");
            //if (fin == LX_STATE.LX_SUCCESS) Console.WriteLine("DcSetStringValue success");
            //else Console.WriteLine("DcSetStringValue failed,{0}", fin);

            //获取string参数数据
            //IntPtr cv = new IntPtr();
            //fin = DcGetStringValue(handle, 4001, out cv);
            //string str = Marshal.PtrToStringAnsi(cv);
            //if (fin == LX_STATE.LX_SUCCESS)
            //    Console.WriteLine("DcGetStringValue success:{0}", str);
            //else Console.WriteLine("DcGetStringValue failed,{0}", fin);

            //设置命令参数数据
            //if (DcSetCmd(handle, 5006) == LX_STATE.LX_SUCCESS)
            //    Console.WriteLine("DcSetCmd success");
            //else Console.WriteLine("DcSetCmd failed");

            //开始取流
            if (DcStartStream(handle) == LX_STATE.LX_SUCCESS) Console.WriteLine("start stream success");
            else Console.WriteLine("start stream failed");

            Thread thread = new Thread(new ThreadStart(WaitKey));
            thread.Start();
            thread.IsBackground = true;

            while (true)
            {
                //获取一帧数据
                if (DcSetCmd(handle, 5001) == LX_STATE.LX_SUCCESS)
                    Console.WriteLine("Get new frame success");
                else Console.WriteLine("Get new frame  failed");

                //获取2D图像数据
                //IntPtr voi = new IntPtr();
                //LX_STATE _fin = DcGetPtrValue(handle, 6001, ref voi);

                TestFrame(handle);
                if (wait_key == 'q' || wait_key == 'Q')
                    break;

            }
            //停止取流
            if (DcStopStream(handle) == LX_STATE.LX_SUCCESS)
                Console.WriteLine("stop stream success");
            else Console.WriteLine("stop stream failed");

            //断开连接
            if (DcCloseDevice(handle) == LX_STATE.LX_SUCCESS)
                Console.WriteLine("close device success");
            else Console.WriteLine("close device failed");

            return 0;
            int TestFrame(long handle)
            {
                //获取图像数据信息
                IntPtr frameInfo;
                LX_STATE _fin = DcGetPtrValue(handle, 6009, out frameInfo);
                FrameExtendInfo* pextendframe = null;


                FrameInfo frame = (FrameInfo)Marshal.PtrToStructure(frameInfo, typeof(FrameInfo));
                if (frame.reserve_data != IntPtr.Zero)
                {
                    pextendframe = (FrameExtendInfo*)frame.reserve_data;
                }
                if (frame.reserve_data != IntPtr.Zero)
                {
                    var reserve_data = frame.reserve_data;
                    Console.WriteLine("reserve_data:\n" + reserve_data);
                }
                System.Console.WriteLine(frame.depth_data.frame_width);
                System.Console.WriteLine(frame.depth_data.frame_height);

                if (_fin == LX_STATE.LX_SUCCESS) Console.WriteLine("DcGetPtrValue success");
                else Console.WriteLine("DcGetPtrValue failed:{0}", _fin);


                //深度图像
                if (frame.depth_data.frame_data != IntPtr.Zero)
                {

                    var depth_data = frame.depth_data;
                    Console.WriteLine("depth_sensor_time:\n" + depth_data.sensor_timestamp);
                    if (pextendframe != null)
                    {
                        Console.WriteLine("depth_frame_id:\n" + pextendframe->depth_frame_id);
                    }

                    //获取xyz点云数据
                    IntPtr xyz;
                    DcGetPtrValue(handle, 6004, out xyz);  //LX_PTR_XYZ_DATA  6004
                                                           //深度图像渲染
#if HAS_OPENCV

                    Mat depth_image = Mat.FromPixelData(depth_data.frame_height, depth_data.frame_width,
                        CvConstants.CV_MAKETYPE((int)depth_data.frame_data_type, depth_data.frame_channel),
                        depth_data.frame_data);
                    Mat xyz_image = Mat.FromPixelData(depth_data.frame_height, depth_data.frame_width,
                        CvConstants.CV_MAKETYPE(5, 3), xyz);

                    Mat show = new Mat();
                    depth_image.ConvertTo(show, MatType.CV_8U, 1.0 / 16);
                    Cv2.ApplyColorMap(show, show, ColormapTypes.Jet);
                    Cv2.NamedWindow("depth", 0);
                    Cv2.ResizeWindow("depth", 640, 480);
                    Cv2.ImShow("depth", show);
                    Cv2.WaitKey(1);
                    show.Dispose();
                    //点云图像渲染（点云图像暂时为平面图像，查看点云可以在客户端中查看）
                    Mat showxyz = new Mat();
                    xyz_image.ConvertTo(showxyz, MatType.CV_8U, 1.0 / 8);
                    Cv2.ApplyColorMap(showxyz, showxyz, ColormapTypes.Jet);
                    Cv2.NamedWindow("xyz", 0);
                    Cv2.ResizeWindow("xyz", 640, 480);
                    Cv2.ImShow("xyz", showxyz);
                    Cv2.WaitKey(1);
                    showxyz.Dispose();
#endif
                }
                //强度图像
                if (frame.amp_data.frame_data != IntPtr.Zero)
                {
                    var amp_data = frame.amp_data;

                    Console.WriteLine("amp_sensor_time:\n" + amp_data.sensor_timestamp);
                    if (pextendframe != null)
                    {
                        Console.WriteLine("amp_frame_id:\n" + pextendframe->amp_frame_id);
                    }

                    //强度图像渲染
#if HAS_OPENCV
                    Mat amp_image = Mat.FromPixelData(amp_data.frame_height, amp_data.frame_width,
                        CvConstants.CV_MAKETYPE((int)amp_data.frame_data_type, amp_data.frame_channel),
                        amp_data.frame_data);
                    Mat show = new Mat();
                    if (amp_image.Type() == MatType.CV_16U)
                    {
                        amp_image.ConvertTo(show, MatType.CV_8U, 1.0 / 8);
                    }
                    else
                    {
                        show = amp_image;
                    }
                    Cv2.ApplyColorMap(show, show, ColormapTypes.Jet);
                    Cv2.NamedWindow("amp", 0);
                    Cv2.ResizeWindow("amp", 640, 480);
                    Cv2.ImShow("amp", show);
                    Cv2.WaitKey(1);
                    show.Dispose();
#endif
                }
                //rgb图像
                if (frame.rgb_data.frame_data != IntPtr.Zero)
                {
                    var rgb_data = frame.rgb_data;
                    Console.WriteLine("rgb_sensor_time:\n" + rgb_data.sensor_timestamp);
                    if (pextendframe != null)
                    {
                        Console.WriteLine("rgb_frame_id:\n" + pextendframe->rgb_frame_id);
                    }
                    //rgb图像渲染
#if HAS_OPENCV
                    Mat rgb_image = Mat.FromPixelData(rgb_data.frame_height, rgb_data.frame_width,
                    CvConstants.CV_MAKETYPE((int)rgb_data.frame_data_type, rgb_data.frame_channel),
                    rgb_data.frame_data);

                    Cv2.NamedWindow("rgb", 0);
                    Cv2.ResizeWindow("rgb", 640, 480);
                    Cv2.ImShow("rgb", rgb_image);
                    Cv2.WaitKey(1);

#endif
                }
              
                return 0;
            }
        }
    }
}