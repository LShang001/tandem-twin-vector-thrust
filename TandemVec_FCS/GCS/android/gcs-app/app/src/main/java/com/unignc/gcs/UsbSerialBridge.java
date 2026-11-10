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

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

/**
 * USB 串口桥：把 Android 的 USB Host API 暴露给 Chaquopy 里的 Python。
 *
 * <p>为什么必须要它：Android 上普通应用看不到 /dev/ttyUSB0，pyserial 的
 * comports() 只会返回空列表——所以地面站界面里"识别不到串口"是必然的，
 * 不是驱动问题。唯一合法的访问方式是 USB Host API，而它只有 Java 能调。
 *
 * <p>★ 两个实测踩过的坑（2026-11-10）：
 * <ol>
 *   <li><b>open() 必须主动请求权限</b>。第一版 open() 发现没权限只 return
 *       一个错误字符串，requestPermission() 写了却没人调 → 用户永远等不到
 *       系统弹窗。现在 open() 内部直接触发授权并同步等待结果。</li>
 *   <li><b>listDevices() 不能只列"被驱动识别的"设备</b>。有些数传芯片
 *       （如 CH9102、部分 CDC-ACM 变种）不在 usb-serial-for-android 的默认
 *       探测表里，那时界面完全看不到设备，用户只会说"识别不到串口"而无法
 *       提供更多线索。现在把所有 USB 设备都列出来，未识别的标 supported=false
 *       并带上 VID/PID，便于按需补自定义探测。</li>
 * </ol>
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
        if (ctx != null) {
            sCtx = ctx.getApplicationContext();
            Log.i(TAG, "init 完成 ctx=" + sCtx.getPackageName());
        }
    }

    /**
     * 列出 USB 设备：被串口驱动识别的在前，其余设备也列出来（标 supported=false）。
     *
     * @return JSON 数组，元素 {vid,pid,name,desc,granted,supported}；无设备返回 "[]"
     */
    public static String listDevices() {
        if (sCtx == null) {
            return "[]";
        }
        try {
            UsbManager um = (UsbManager) sCtx.getSystemService(Context.USB_SERVICE);
            StringBuilder sb = new StringBuilder("[");
            boolean first = true;

            // 1) 能被串口驱动打开的（正常可用）
            List<UsbSerialDriver> drivers = UsbSerialProber.getDefaultProber().findAllDrivers(um);
            Map<Integer, Boolean> seen = new HashMap<>();
            for (UsbSerialDriver d : drivers) {
                UsbDevice dev = d.getDevice();
                seen.put(dev.getDeviceId(), true);
                if (!first) sb.append(',');
                first = false;
                sb.append('{')
                  .append("\"vid\":").append(dev.getVendorId()).append(',')
                  .append("\"pid\":").append(dev.getProductId()).append(',')
                  .append("\"name\":\"").append(esc(dev.getDeviceName())).append("\",")
                  .append("\"desc\":\"").append(esc(d.getClass().getSimpleName())).append("\",")
                  .append("\"granted\":").append(um.hasPermission(dev)).append(',')
                  .append("\"supported\":true")
                  .append('}');
            }

            // 2) 其余 USB 设备：驱动不认，但要让用户看见并报出 VID/PID
            for (UsbDevice dev : um.getDeviceList().values()) {
                if (seen.containsKey(dev.getDeviceId())) continue;
                if (!first) sb.append(',');
                first = false;
                sb.append('{')
                  .append("\"vid\":").append(dev.getVendorId()).append(',')
                  .append("\"pid\":").append(dev.getProductId()).append(',')
                  .append("\"name\":\"").append(esc(dev.getDeviceName())).append("\",")
                  .append("\"desc\":\"未识别的 USB 设备\"," )
                  .append("\"granted\":").append(um.hasPermission(dev)).append(',')
                  .append("\"supported\":false")
                  .append('}');
            }
            return sb.append(']').toString();
        } catch (Throwable e) {
            Log.e(TAG, "listDevices 失败", e);
            return "[]";
        }
    }

    /** 权限接收器（懒注册，只注册一次） */
    private static void ensureReceiver() {
        if (sReceiver != null || sCtx == null) return;
        sReceiver = new BroadcastReceiver() {
            @Override
            public void onReceive(Context context, Intent intent) {
                if (!ACTION_PERM.equals(intent.getAction())) return;
                synchronized (LOCK) {
                    sPermGranted = intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false);
                    if (sPermLatch != null) sPermLatch.countDown();
                }
                Log.i(TAG, "收到授权广播 granted=" + sPermGranted);
            }
        };
        IntentFilter f = new IntentFilter(ACTION_PERM);
        if (Build.VERSION.SDK_INT >= 33) {
            sCtx.registerReceiver(sReceiver, f, Context.RECEIVER_NOT_EXPORTED);
        } else {
            sCtx.registerReceiver(sReceiver, f);
        }
    }

    /**
     * 为指定设备请求权限并等待结果（最多 10 秒）。
     *
     * @return true = 已授权；false = 被拒或超时
     */
    private static boolean requestPermissionFor(UsbDevice dev) {
        if (sCtx == null || dev == null) return false;
        UsbManager um = (UsbManager) sCtx.getSystemService(Context.USB_SERVICE);
        if (um.hasPermission(dev)) return true;

        ensureReceiver();
        synchronized (LOCK) {
            sPermLatch = new CountDownLatch(1);
            sPermGranted = false;
        }
        // Android 12+ 要求显式声明可变/不可变；Android 14 还要求 setPackage，
        // 否则隐式 mutable PendingIntent 会抛 IllegalArgumentException。
        int flags = 0;
        if (Build.VERSION.SDK_INT >= 31) flags |= PendingIntent.FLAG_MUTABLE;
        PendingIntent pi = PendingIntent.getBroadcast(sCtx, 0,
                new Intent(ACTION_PERM).setPackage(sCtx.getPackageName()), flags);
        Log.i(TAG, "请求 USB 权限 device=" + dev.getDeviceName());
        um.requestPermission(dev, pi);

        try {
            boolean settled = sPermLatch.await(10, TimeUnit.SECONDS);
            Log.i(TAG, "授权等待结束 settled=" + settled + " granted=" + sPermGranted);
            return settled && sPermGranted;
        } catch (InterruptedException e) {
            return false;
        } finally {
            synchronized (LOCK) { sPermLatch = null; }
        }
    }

    /**
     * 打开串口。没权限时**主动请求授权并等待**，成功则直接打开。
     *
     * @param vid vid，-1 表示第一个可用设备
     * @param pid pid，-1 表示不校验
     * @return 空字符串表示成功，否则是中文错误原因（可直接显示）
     */
    public static String open(int vid, int pid, int baud) {
        synchronized (LOCK) {
            closeQuietly();
            if (sCtx == null) {
                return "USB 桥未初始化（应用没启动完，重开一次）";
            }
            UsbManager um = (UsbManager) sCtx.getSystemService(Context.USB_SERVICE);
            List<UsbSerialDriver> drivers = UsbSerialProber.getDefaultProber().findAllDrivers(um);
            if (drivers.isEmpty()) {
                int total = um.getDeviceList().size();
                if (total == 0) {
                    return "没检测到任何 USB 设备：确认 OTG 线通畅、数传已供电、"
                            + "并在系统设置里打开「USB 主机/OTG」";
                }
                return "检测到 " + total + " 个 USB 设备，但没有一个被识别为串口："
                        + "该数传芯片可能不在支持列表（CH340/CP210x/FTDI/PL2303/CDC-ACM），"
                        + "把界面串口列表里的 VID/PID 发出来即可补支持";
            }
            UsbSerialDriver target = null;
            for (UsbSerialDriver d : drivers) {
                if (vid < 0 || (d.getDevice().getVendorId() == vid && d.getDevice().getProductId() == pid)) {
                    target = d;
                    break;
                }
            }
            if (target == null) {
                return "指定的 USB 设备不在了（可能被拔掉），刷新串口列表重选";
            }
            UsbDevice dev = target.getDevice();

            // ★ 关键：没权限就在这里主动触发授权并等结果，而不是返回错误让用户干等
            if (!um.hasPermission(dev)) {
                if (!requestPermissionFor(dev)) {
                    return "USB 授权未通过：系统弹框出现时要点「允许」；"
                            + "若没有弹框，请拔插数传后重试";
                }
                if (!um.hasPermission(dev)) {
                    return "系统未记录到授权（可能被自动拒绝），拔插数传后重试";
                }
            }

            sConn = um.openDevice(dev);
            if (sConn == null) {
                return "openDevice 失败：设备被系统或其他应用占用";
            }
            sPort = target.getPorts().get(0);
            try {
                sPort.open(sConn);
                sPort.setParameters(baud, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);
                Log.i(TAG, "串口已打开 " + baud + " 8N1");
                return "";
            } catch (Exception e) {
                Log.e(TAG, "打开串口失败", e);
                closeQuietly();
                return "打开串口失败：" + e.getClass().getSimpleName() + " " + e.getMessage();
            }
        }
    }

    /** 读一批字节（无数据返回空数组） */
    public static byte[] read(int timeoutMs) {
        synchronized (LOCK) {
            if (sPort == null) return new byte[0];
            try {
                byte[] buf = new byte[512];
                int n = sPort.read(buf, Math.max(50, timeoutMs));
                if (n <= 0) return new byte[0];
                byte[] out = new byte[n];
                System.arraycopy(buf, 0, out, 0, n);
                return out;
            } catch (Exception e) {
                Log.w(TAG, "读取出错: " + e.getMessage());
                return new byte[0];
            }
        }
    }

    /** 写字节，返回实际写入数；-1 = 未连接 */
    public static int write(byte[] data) {
        synchronized (LOCK) {
            if (sPort == null) return -1;
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
        synchronized (LOCK) { closeQuietly(); }
    }

    private static void closeQuietly() {
        try { if (sPort != null) sPort.close(); } catch (Exception ignored) { }
        try { if (sConn != null) sConn.close(); } catch (Exception ignored) { }
        sPort = null;
        sConn = null;
    }

    private static String esc(String s) {
        if (s == null) return "";
        return s.replace("\\", "\\\\").replace("\"", "\\\"");
    }
}
