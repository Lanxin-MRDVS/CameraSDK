using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Drawing;
using System.Windows.Input;
using System.IO;
using System.Runtime.InteropServices;
using lx;
using static lx.LxCamera;

namespace demo
{
    class demo
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

            while (true)
            {
                //获取一帧数据
                if (DcSetCmd(handle, 5001) == LX_STATE.LX_SUCCESS)
                    Console.WriteLine("Get new frame success");
                else Console.WriteLine("Get new frame  failed");

                //获取2D图像数据
                IntPtr voi = new IntPtr();
                LX_STATE _fin = DcGetPtrValue(handle, 6001, ref voi);
                if (_fin == LX_STATE.LX_SUCCESS) Console.WriteLine("DcGetPtrValue success");
                else Console.WriteLine("DcGetPtrValue failed:{0}", _fin);
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

        }
    }
}
