using System.Runtime.InteropServices;
using static lx.LxCamera;
using static lxApplication.LxApplication;
using System.Net.Http.Headers;
using System.Security.Cryptography;
using System.Security.Cryptography.X509Certificates;
using System.Runtime.CompilerServices;
using System.Text;
using System.Diagnostics;
using System.ComponentModel;
using System.Data;
using System.Net.WebSockets;

namespace demo
{
    public static unsafe class CvConstants
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
        unsafe static int Main()
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

         
           
            //获取一帧数据
            if (DcSetCmd(handle, 5001) == LX_STATE.LX_SUCCESS)
                Console.WriteLine("Get new frame success");
            else Console.WriteLine("Get new frame  failed");

            //获取2D图像数据
            //IntPtr voi = new IntPtr();
            //LX_STATE _fin = DcGetPtrValue(handle, 6001, ref voi);

            //获取图像数据信息
            IntPtr frameInfo;
            //void * frameInfo=null;

            //LX_STATE _fin = DcGetPtrValue(handle, 6009, out frameInfo);
            LX_STATE _fin = DcGetPtrValue(handle, 6009,out frameInfo);
            FrameExtendInfo pextendframe;
                             
            FrameInfo frame = (FrameInfo)Marshal.PtrToStructure((nint)frameInfo, typeof(FrameInfo));
            if (frame.reserve_data != IntPtr.Zero)
            {
                pextendframe = (FrameExtendInfo)Marshal.PtrToStructure(frame.reserve_data, typeof(FrameExtendInfo));
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

            //IntPtr gdevice_info=IntPtr.Zero;
            //LxDeviceInfo device_info;
            //string open_param = "192.168.100.82";
            //DcOpenDevice(0, open_param,ref handle,gdevice_info);  //参数一 LX_OPEN_MODE(open_mode)

            //device_info=(LxDeviceInfo)Marshal.PtrToStructure(gdevice_info, typeof(LxDeviceInfo));

           
            Console.Write("device_info\n cameraid:" + new string(info.id)+ "\n uniqueid:" + handle +
                            "\n cameraip:" + new string(info.ip)+ "\n firmware_ver:" + new string(info.firmware_ver)+ "\n sn:" + new string(info.sn)
                            + "\n name:" + new string(info.name) + "\n img_algor_ver" + new string(info.algor_ver)+ "\n");
            ////应用算法
            LxIntValueInfo algor_info;
            algor_info.set_available = false;
            algor_info.cur_value = 0;
            algor_info.max_value = 0;
            algor_info.min_value = 0;
            algor_info.reserve1 = 0;
            algor_info.reserve2 = 0;
            algor_info.reserve3 = 0;
            algor_info.reserve4 = 0;
            DcGetIntValue(handle, 1065,ref algor_info); //LX_INT_ALGORITHM_MODE
            Console.WriteLine("current LX_INT_ALGORITHM_MODE:"+algor_info.cur_value);

            int algor_mode = 0;


            //V2版本
            algor_mode = 4;  //内置避障算法V2 MODE_AVOID_OBSTACLE2=4
            DcSetIntValue(handle, 1065, algor_mode); //LX_INT_ALGORITHM_MODE

            //算法版本号
            char* algor_ver = null;
            DcGetStringValue(handle, 4006, &algor_ver); //LX_STRING_ALGORITHM_VERSION
            if (algor_ver != null)
            {
                string algorVersion = Marshal.PtrToStringAnsi((IntPtr)algor_ver);
                Console.WriteLine("current algor version:" + algorVersion);
            }

            //托盘算法参数可以不设置。如果需要可通过如下函数设置，参数为json格式
            //std::string str_pallet_json = "{}";
            //checkTC(DcSetStringValue(handle, LX_STRING_ALGORITHM_PARAMS, str_pallet_json.c_str()));

            //算法参数，与当前设置有关
            char* cur_algor_json = null;
            DcGetStringValue(handle, 4005, &cur_algor_json); // LX_STRING_ALGORITHM_PARAMS = 4005,  
            if (cur_algor_json != null)
            {
                string curAlgorjson = Marshal.PtrToStringAnsi((IntPtr)cur_algor_json);
                Console.WriteLine("current algor json param:" + curAlgorjson);
            }

            //根据需要，修改算法参数，为json格式,可通过json库也可以通过string直接修改，也可通过文件中转
            //以下为该算法json范例，其中ZoneIndex为当前生效的避障模式索引，对应Zones中多个模式，Zones中的单个模式中区分危险区域和预警区域坐标。
            //    string obstacle_json_param = "{ \
            //\"R\": [0.00000002842, 0.00000119, 1.00000119, -1.00000011920, 0.0000119209, 0.0000011920931, -0.00002384, -1.00000011920, 0.0000119209],\
            //\"T\": [400, 0, 120],\
            //\"ZoneIndex\": 0,\
            //\"Zones\": {\
            //    \"0\": {\
            //        \"DangerZone\": [400, 700, -500, 500, -50, 500],\
            //        \"WarningZone\": [700, 1300, -500, 500, -50, 500]\
            //    },\
            //    \"1\": {\
            //        \"DangerZone\": [400, 700, -400, 400, -50, 500],\
            //        \"WarningZone\": [700, 1300, -400, 400, -50, 500]\
            //    }\
            //},\
            //\"confidences\": [0.29999, 0.29999, 0.29999, 0.29999, 0.29999, 0.29999, 0.29999, 0.29999, 0.29999, 0.29999, 0.29999, 0.29999, 0.29999, 0.29999, 0.29999, 0.29999, 0.29999, 0.29999, 0.29999, 0.29999999999999999],\
            //\"debug\": false,\
            //\"distacne\": 0,\
            //\"ranging\": false,\
            //\"semantics\": true,\
            //\"setDetectGroundPlane\": [20, 500, 40, 0, 150],\
            //\"setDownsampling\": [30, 30, 30],\
            //\"setRadiusFilter\": [-1, 1],\
            //\"setRange\": [0, 4000, -500, 500, -50, 500]\
            //}";

            //修改避障参数，建议通过上位机LxCameraViewer修改。S2仅支持上位机修改！！！
            //checkTC(DcSetStringValue(handle, LX_STRING_ALGORITHM_PARAMS, obstacle_json_param.c_str()));

            //仅切换避障参数可以通过以下接口实现。S2需要上位机配置好每套参数后才能切换，否则无效
            nint pobstacle_mode = 0;//使用第二套参数
            DcSpecialControl(handle,"SetObstacleMOode", ref pobstacle_mode);
            DcStartStream(handle);

            //正常状态下，网络断开或者SDK关闭之后，相机会切换为待机状态。
            //如果算法结果不通过SDK输出（IO 串口 UDP协议等方式），需要设置为常开模式，相机和内置算法会始终保持工作
            DcSetIntValue(handle, 1018, 1); //LX_INT_WORK_MODE
            while (true)
            {

                var ret = DcSetCmd(handle, 5001); //LX_CMD_GET_NEW_FRAME
                if ((ret != LX_STATE.LX_SUCCESS)
                    && (ret != LX_STATE.LX_E_FRAME_ID_NOT_MATCH)
                    && (ret != LX_STATE.LX_E_FRAME_MULTI_MACHINE))

                {
                    Console.WriteLine(1);
                    Console.WriteLine("DcSetCmd LX_CMD_GET_NEW_FRAME failed" + "/n");
                    if (LX_STATE.LX_E_RECONNECTING == ret)
                    {
                        Console.WriteLine("device is reconnecting" + "/n");
                        Thread.Sleep(1000);
                    }
                    continue;
                }

                //获取避障IO输出结果
                nint obstac_io_result = 1;
                DcSpecialControl(handle, "GetObstacleIO", ref obstac_io_result);
                Console.WriteLine("obstacle io result:" + obstac_io_result);
                Console.WriteLine(info.dev_type);

                if (info.dev_type == LX_DEVICE_TYPE.LX_DEVICE_M4_MEGA || info.dev_type == LX_DEVICE_TYPE.LX_DEVICE_M4 ||
                    info.dev_type == LX_DEVICE_TYPE.LX_DEVICE_M4Pro)
                {
                    IntPtr algordata = IntPtr.Zero;
                    DcGetPtrValue(handle, 6008, out algordata); //LX_PTR_ALGORITHM_OUTPUT  6008                 
                    if (algordata == null)
                    {
                        Console.WriteLine(10);
                    }
                    PrintData(algordata, algor_mode);
                }
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
            unsafe int PrintData(IntPtr data_ptr,int obstacle_mode) 
            {


                if (data_ptr == null)
                    return 0;
                StringBuilder  algor_ss = new StringBuilder();
                if (obstacle_mode == 1)  //MODE_AVOID_OBSTACLE
                {
  
                    LxAvoidanceOutputN obstacle = (LxAvoidanceOutputN)Marshal.PtrToStructure(data_ptr, typeof(LxAvoidanceOutput));
                    if (obstacle.state != LxAvState.LxAvSuccess)
                    {
                        Console.WriteLine("state is not success");
                        return 0;
                    }
                    algor_ss.Append("groundPlane.a: " + obstacle.groundPlane.a + " number_3d: " + obstacle.number_3d +
                        " number_box: " + obstacle.number_box + "\n");
                    if (obstacle.cloud_output != null)
                    {
                        algor_ss.Append("cloud data:");
                        var cloud_num = obstacle.number_3d < 10 ? obstacle.number_3d : 10;
                        for (int i = 0; i < (int)cloud_num; i++)
                        {
                            algor_ss.Append("index" + i + "x:" + obstacle.cloud_output[i].x + "y:" + obstacle.cloud_output[i].y +
                                "z:" + obstacle.cloud_output[i].z + "r:" + obstacle.cloud_output[i].r +
                                "g:" + obstacle.cloud_output[i].g + "b:" + obstacle.cloud_output[i].b + "\n");
                        }
                    }

                }
                else if (obstacle_mode == 4) //MODE_AVOID_OBSTACLE2
                {

                    LxAvoidanceOutputN obstacle=(LxAvoidanceOutputN)Marshal.PtrToStructure(data_ptr,typeof(LxAvoidanceOutputN));

                    if (obstacle.state != LxAvState.LxAvSuccess)
                    {
                        Console.WriteLine("state is not success");
                        return 0;
                    }
                  
                    algor_ss.Append("groudPanel: " + obstacle.groundPlane.a + " number_3d: " + obstacle.number_3d + " number_box: " + obstacle.number_box + "\n");

                    if (obstacle.cloud_output != null)
                    {
                        algor_ss.Append("cloud data:");  //输出前10个

                        var cloud_num = obstacle.number_3d < 10 ? obstacle.number_3d : 10;
                        for (int i = 0; i < (int)cloud_num; i++)
                        {
                            algor_ss.Append("index" + i + "x:" + obstacle.cloud_output[i].x + "y:" + obstacle.cloud_output[i].y +
                                "z:" + obstacle.cloud_output[i].z + "r:" + obstacle.cloud_output[i].r +
                                "g:" + obstacle.cloud_output[i].g + "b:" + obstacle.cloud_output[i].b + "\n");
                        }
                    }

                }
                Console.WriteLine(algor_ss.ToString()+"\n");
                algor_ss.Clear();
                algor_ss = null;
                return 0;
            }       

        }
    }
        
 }
