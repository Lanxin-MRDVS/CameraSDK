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
            //void * frameInfo;
            LX_STATE _fin = DcGetPtrValue(handle, 6009, out frameInfo);
            FrameExtendInfo pextendframe;
                             
            FrameInfo frame = (FrameInfo)Marshal.PtrToStructure(frameInfo, typeof(FrameInfo));
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

            Console.Write("device_info\n cameraid:" + new string(info.id) + "\n uniqueid:" + handle +
                               "\n cameraip:" + new string(info.ip) + "\n firmware_ver:" + new string(info.firmware_ver) + "\n sn:" + new string(info.sn)
                               + "\n name:" + new string(info.name) + "\n img_algor_ver" + new string(info.algor_ver) + "\n");

            //开启托盘对接算法
            DcSetBoolValue(handle,3002,false); //LX_BOOL_ENABLE_3D_DEPTH_STREAM = 3002, 
            DcSetBoolValue(handle,3003,false); //LX_BOOL_ENABLE_3D_AMP_STREAM = 3003, 
            DcSetBoolValue(handle,3011,false); //LX_BOOL_ENABLE_2D_STREAM = 3011, 
            DcSetIntValue(handle,1065,2); //LX_INT_ALGORITHM_MODE = 1065   MODE_PALLET_LOCATE = 2

            //应用算法   
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
        
            //算法版本号
            IntPtr algor_ver = IntPtr.Zero; 
            DcGetStringValue(handle, 4006,out algor_ver); //LX_STRING_ALGORITHM_VERSION
            if(algor_ver != IntPtr.Zero)
            {
                string algorVersion = Marshal.PtrToStringAnsi((IntPtr)algor_ver);
                Console.WriteLine("current algor version:" + algorVersion);
            }

            //托盘算法参数可以不设置。如果需要可通过如下函数设置，参数为json格式
            //std::string str_pallet_json = "{}";
            //checkTC(DcSetStringValue(handle, LX_STRING_ALGORITHM_PARAMS, str_pallet_json.c_str()));

            //算法参数，与当前设置有关
            IntPtr cur_algor_json = IntPtr.Zero;
            //char* cur_algor_json = null;
            DcGetStringValue(handle, 4005, out cur_algor_json); // LX_STRING_ALGORITHM_PARAMS = 4005,  
            if (cur_algor_json != IntPtr.Zero)
            {
                string curAlgorjson = Marshal.PtrToStringAnsi((IntPtr)cur_algor_json);
                Console.WriteLine("current algor json param:" + curAlgorjson);
            }
            //正常状态下，网络断开或者SDK关闭之后，相机会切换为待机状态。
            //如果算法结果不通过SDK输出，需要设置为常开模式，相机和内置算法会始终保持工作。此时启停流无效，不允许需要停流才能进行的操作
            //checkTC(DcSetIntValue(handle, LX_INT_WORK_MODE, 1));

            while (true)
            {

                //刷新数据
                //如果开启了多个数据流，只要有一个更新，此函数会正常返回。
                //可以设置LX_BOOL_ENABLE_SYNC_FRAME为true保持所有数据同步。或者通过帧ID和时间戳判断需要的数据是否更新
                var ret = DcSetCmd(handle, 5001); //LX_CMD_GET_NEW_FRAME
                if ((ret != LX_STATE.LX_SUCCESS)
                    &&(ret != LX_STATE.LX_E_FRAME_ID_NOT_MATCH)
                    &&(ret != LX_STATE.LX_E_FRAME_MULTI_MACHINE))

                {
                    Console.WriteLine("DcSetCmd LX_CMD_GET_NEW_FRAME failed"+"/n");
                    if (LX_STATE.LX_E_RECONNECTING == ret)
                    {
                        Console.WriteLine("device is reconnecting" + "/n");
                        Thread.Sleep(1000); 
                    }

                    continue;
                }

                //获取数据
                IntPtr  algordata=IntPtr.Zero;
                DcGetPtrValue(handle, 6008,out algordata); //LX_PTR_ALGORITHM_OUTPUT
                PrintData(algordata);
        
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

            unsafe int PrintData(IntPtr data_ptr)
            {
                if(data_ptr == IntPtr.Zero)
                {
                    return 0;

                }

                LxPalletPose palletdata = (LxPalletPose)Marshal.PtrToStructure(data_ptr, typeof(LxPalletPose));
                
                Console.WriteLine("palletdata: ret"+palletdata.return_val+" x: "+palletdata.x+" y: "+palletdata.y+" yaw: "+palletdata.yaw+"\n");
                return 0;

            }
        }
    }
        
 }
