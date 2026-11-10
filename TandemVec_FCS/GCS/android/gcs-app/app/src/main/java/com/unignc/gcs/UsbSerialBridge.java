package com.unignc.gcs;

import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.os.Build;
import android.util.Log;

import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;

import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

/**
 * USB 串口桥：把 Android 的 USB Host API 暴露给 Chaquopy 里的 Python。
 *
 * <p>为什么必须要它：Android 上普通应用看不到 /dev/ttyUSB0，pyserial 的
 * comports() 只会返回空列表——所以地面站界面里"识别不到串口"是必然的，
 * 不是驱动问题。唯一合法的访问方式是 USB Host API，而它只有 Java 能调。
 *
 * <p>设计：全部静态方法 + 单例串口。Python 侧（chaquopy_usb_link.py）
 * 通过 jclass 直接调这些方法，不需要额外的 Python-C 绑定。
 *
 * <p>权限：Android 要求用户对每个 USB 设备点一次"允许"。这里用
 * requestPermission 异步请求，Python 侧 open() 时若尚未授权会返回被拒，
 * 界面提示用户等授权框出现并点允许，再点一次连接。
 */
public final class UsbSerialBridge {

    private static final String TAG = "UsbSerialBridge";
    private static final String ACTION_PERM = "com.unignc.gcs.USB_PERM";

    private static Context sCtx;
    private static UsbSerialPort sPort;
    private static UsbDeviceConnection sConn;
    private static final Object LOCK = new Object();
    private static volatile CountDownLatch sPermLatch;
    private static volatile boolean sPermGranted = false;
    private static BroadcastReceiver sReceiver;

    private UsbSerialBridge() { }

    /** Python 侧启动时调用，注入 Application Context */
    public static void init(Context ctx) {
        if (sCtx == null) {
            sCtx = ctx.getApplicationContext();
        }
    }

    /**
     * 列出所有可用的 USB 串口设备。
     *
     * @return JSON 数组字符串，元素 {vid,pid,name,desc,port}；无设备时返回 "[]"。
     *         用 JSON 而不是并行数组，是为了 Python 侧直接 json.loads，不用猜分隔符。
     */
    public static String listDevices() {
        if (sCtx == null) {
            return "[]";
        }
        try {
            UsbManager um = (UsbManager) sCtx.getSystemService(Context.USB_SERVICE);
            List<UsbSerialDriver> drivers = UsbSerialProber.getDefaultProber().findAllDrivers(um);
            StringBuilder sb = new StringBuilder("[");
            for (int i = 0; i < drivers.size(); i++) {
                UsbSerialDriver d = drivers.get(i);
                UsbDevice dev = d.getDevice();
                boolean granted = um.hasPermission(dev);
                if (i > 0) sb.append(',');
                sb.append('{')
                  .append("\"vid\":").append(dev.getVendorId()).append(',')
                  .append("\"pid\":").append(dev.getProductId()).append(',')
                  .append("\"name\":\"").append(esc(dev.getDeviceName())).append("\",")
                  .append("\"desc\":\"").append(esc(d.getClass().getSimpleName())).append("\",")
                  .append("\"granted\":").append(granted)
                  .append('}');
            }
            return sb.append(']').toString();
        } catch (Throwable e) {
            Log.e(TAG, "listDevices 失败", e);
            return "[]";
        }
    }

    /**
     * 请求某个 USB 设备的访问权限（异步，最多等 10 秒）。
     *
     * @return true = 用户已允许；false = 被拒或超时。界面应提示再试一次。
     */
    public static boolean requestPermission() {
        if (sCtx == null) {
            return false;
        }
        UsbManager um = (UsbManager) sCtx.getSystemService(Context.USB_SERVICE);
        List<UsbSerialDriver> drivers = UsbSerialProber.getDefaultProber().findAllDrivers(um);
        if (drivers.isEmpty()) {
            return false;
        }
        UsbDevice dev = drivers.get(0).getDevice();
        if (um.hasPermission(dev)) {
            return true;
        }

        synchronized (LOCK) {
            if (sPermLatch != null) {
                // 已有一次请求在等待，直接等它
                try {
                    return sPermLatch.await(10, TimeUnit.SECONDS) && sPermGranted;
                } catch (InterruptedException e) {
                    return false;
                }
            }
            sPermLatch = new CountDownLatch(1);
            sPermGranted = false;
        }

        if (sReceiver == null) {
            sReceiver = new BroadcastReceiver() {
                @Override
                public void onReceive(Context context, Intent intent) {
                    if (!ACTION_PERM.equals(intent.getAction())) return;
                    synchronized (LOCK) {
                        sPermGranted = intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false);
                        if (sPermLatch != null) {
                            sPermLatch.countDown();
                        }
                    }
                }
            };
            IntentFilter f = new IntentFilter(ACTION_PERM);
            if (Build.VERSION.SDK_INT >= 33) {
                sCtx.registerReceiver(sReceiver, f, Context.RECEIVER_NOT_EXPORTED);
            } else {
                sCtx.registerReceiver(sReceiver, f);
            }
        }

        int flags = Build.VERSION.SDK_INT >= 31
                ? PendingIntent.FLAG_MUTABLE : 0;
        PendingIntent pi = PendingIntent.getBroadcast(sCtx, 0,
                new Intent(ACTION_PERM).setPackage(sCtx.getPackageName()), flags);
        um.requestPermission(dev, pi);

        try {
            boolean ok = sPermLatch.await(10, TimeUnit.SECONDS);
            Log.i(TAG, "权限请求结果 granted=" + sPermGranted + " settled=" + ok);
            return ok && sPermGranted;
        } catch (InterruptedException e) {
            return false;
        } finally {
            synchronized (LOCK) {
                sPermLatch = null;
            }
        }
    }

    /**
     * 打开指定设备（vid/pid 传 -1 表示"第一个可用设备"，方便界面只给一个选项时的默认值）。
     *
     * @return 空字符串表示成功，否则是中文错误原因（直接可显示给用户）。
     */
    public static String open(int vid, int pid, int baud) {
        synchronized (LOCK) {
            closeQuietly();
            if (sCtx == null) {
                return "USB 桥未初始化";
            }
            UsbManager um = (UsbManager) sCtx.getSystemService(Context.USB_SERVICE);
            List<UsbSerialDriver> drivers = UsbSerialProber.getDefaultProber().findAllDrivers(um);
            if (drivers.isEmpty()) {
                return "未检测到 USB 串口设备：检查 OTG 线、数传供电，以及系统是否识别";
            }
            UsbSerialDriver target = null;
            for (UsbSerialDriver d : drivers) {
                if (vid < 0 || (d.getDevice().getVendorId() == vid && d.getDevice().getProductId() == pid)) {
                    target = d;
                    break;
                }
            }
            if (target == null) {
                return "指定的 USB 设备不在了（可能被拔掉）";
            }
            UsbDevice dev = target.getDevice();
            if (!um.hasPermission(dev)) {
                return "还没有 USB 访问权限：界面点连接后等系统弹框，点允许再试一次";
            }
            sConn = um.openDevice(dev);
            if (sConn == null) {
                return "openDevice 失败：权限被撤销或设备被系统占用";
            }
            sPort = target.getPorts().get(0);
            try {
                sPort.open(sConn);
                sPort.setParameters(baud, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);
                return "";
            } catch (Exception e) {
                Log.e(TAG, "打开串口失败", e);
                closeQuietly();
                return "打开串口失败：" + e.getClass().getSimpleName() + " " + e.getMessage();
            }
        }
    }

    /** 读一批字节（无数据返回空数组）。timeoutMs 起的是等待上限作用。 */
    public static byte[] read(int timeoutMs) {
        synchronized (LOCK) {
            if (sPort == null) {
                return new byte[0];
            }
            try {
                byte[] buf = new byte[512];
                int n = sPort.read(buf, Math.max(50, timeoutMs));
                if (n <= 0) {
                    return new byte[0];
                }
                byte[] out = new byte[n];
                System.arraycopy(buf, 0, out, 0, n);
                return out;
            } catch (Exception e) {
                Log.w(TAG, "读取出错: " + e.getMessage());
                return new byte[0];
            }
        }
    }

    /** 写字节，返回实际写入数；-1 表示未连接。 */
    public static int write(byte[] data) {
        synchronized (LOCK) {
            if (sPort == null) {
                return -1;
            }
            try {
                sPort.write(data, 1000);
                return data.length;
            } catch (Exception e) {
                Log.w(TAG, "写入出错: " + e.getMessage());
                return -1;
            }
        }
    }

    public static boolean isOpen() {
        return sPort != null;
    }

    public static void close() {
        synchronized (LOCK) {
            closeQuietly();
        }
    }

    private static void closeQuietly() {
        try {
            if (sPort != null) {
                sPort.close();
            }
        } catch (Exception ignored) { }
        try {
            if (sConn != null) {
                sConn.close();
            }
        } catch (Exception ignored) { }
        sPort = null;
        sConn = null;
    }

    private static String esc(String s) {
        if (s == null) return "";
        return s.replace("\\", "\\\\").replace("\"", "\\\"");
    }
}
