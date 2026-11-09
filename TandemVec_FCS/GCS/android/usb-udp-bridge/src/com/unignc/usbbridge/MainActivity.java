package com.unignc.usbbridge;

import android.app.Activity;
import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.text.InputType;
import android.view.Gravity;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.TextView;

import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.SocketAddress;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;
import java.util.Locale;

/**
 * UniGNC USB-UDP 桥（平板端）。
 *
 * <p>存在理由：Android 上普通应用打不开 /dev/ttyUSB0，只有原生 App 能经
 * USB Host API 访问串口；而 WebView 拿不到串口。所以把"读串口"这一件事
 * 做成原生 App，把字节转成 UDP —— 后端（电脑或平板上的 Termux）用已有的
 * `udp:端口` 链路接收即可，那条链路已经端到端验证过。
 *
 * <p>数据方向：
 * <pre>
 *   数传 --USB--> 本 App --UDP--> 地面站后端   （遥测）
 *   数传 <--USB-- 本 App <--UDP-- 地面站后端   （命令/参数）
 * </pre>
 *
 * <p>刻意不做：不解析 AnoCom 或 MAVLink，不改字节。这样协议升级不需要改
 * 这个 App，也不会有"App 与后端解析器版本不一致"的问题。
 */
public class MainActivity extends Activity {

    private static final String ACTION_USB_PERMISSION = "com.unignc.usbbridge.USB_PERMISSION";

    private EditText etHost;
    private EditText etPort;
    private EditText etBaud;
    private Button btnStart;
    private TextView tvLog;
    private ScrollView scrollLog;

    private final Handler ui = new Handler(Looper.getMainLooper());
    private volatile boolean running = false;
    private Thread usbToUdp;
    private Thread udpToUsb;
    private UsbSerialPort port;
    private UsbDeviceConnection connection;
    private DatagramSocket udp;
    private SocketAddress peer;

    private long cntToUdp = 0;
    private long cntToUsb = 0;

    private final BroadcastReceiver permReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            if (!ACTION_USB_PERMISSION.equals(intent.getAction())) return;
            UsbDevice dev = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
            if (intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)) {
                log("USB 授权通过：" + (dev != null ? dev.getDeviceName() : "?"));
                startBridge();
            } else {
                log("USB 授权被拒绝 —— 拔插数传后重试，或到系统设置里放开权限");
            }
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(buildUi());

        IntentFilter f = new IntentFilter(ACTION_USB_PERMISSION);
        // Android 13+ 注册动态广播必须显式声明是否导出的标志
        if (android.os.Build.VERSION.SDK_INT >= 33) {
            registerReceiver(permReceiver, f, Context.RECEIVER_NOT_EXPORTED);
        } else {
            registerReceiver(permReceiver, f);
        }
        log("就绪。插上数传 → 填目标地址 → 点开始。");
    }

    /** 纯代码构建界面：这个 App 只有一个用途，不值得为它维护 layout 资源。 */
    private View buildUi() {
        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);
        root.setPadding(36, 48, 36, 36);

        TextView title = new TextView(this);
        title.setText("UniGNC USB-UDP 桥");
        title.setTextSize(22f);
        root.addView(title);

        TextView hint = new TextView(this);
        hint.setText("数传插在本平板，字节经 WiFi 转给跑地面站后端的那台机器。\n"
                + "后端连接时填 udp:<本机IP>:<下面端口>，或后端已监听该端口即可。");
        hint.setTextSize(12f);
        hint.setPadding(0, 12, 0, 24);
        root.addView(hint);

        etHost = addField(root, "地面站 IP", "127.0.0.1");
        etPort = addField(root, "UDP 端口", "14550");
        etBaud = addField(root, "波特率", "921600");

        btnStart = new Button(this);
        btnStart.setText("开始转发");
        btnStart.setOnClickListener(v -> onToggle());
        root.addView(btnStart);

        scrollLog = new ScrollView(this);
        tvLog = new TextView(this);
        tvLog.setTextSize(11f);
        tvLog.setPadding(0, 24, 0, 0);
        scrollLog.addView(tvLog);
        LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, 0, 1f);
        root.addView(scrollLog, lp);
        return root;
    }

    private EditText addField(LinearLayout parent, String label, String def) {
        TextView tv = new TextView(parent.getContext());
        tv.setText(label);
        tv.setTextSize(12f);
        parent.addView(tv);
        EditText et = new EditText(this);
        et.setText(def);
        et.setInputType(InputType.TYPE_CLASS_TEXT);
        parent.addView(et);
        return et;
    }

    private void onToggle() {
        if (running) {
            stopBridge();
            btnStart.setText("开始转发");
            return;
        }
        requestUsbThenStart();
        btnStart.setText("停止");
    }

    /** 请求 USB 权限；已授权则直接开始。授权结果在 permReceiver 里回来。 */
    private void requestUsbThenStart() {
        UsbManager um = (UsbManager) getSystemService(Context.USB_SERVICE);
        List<UsbSerialDriver> drivers = UsbSerialProber.getDefaultProber().findAllDrivers(um);
        if (drivers.isEmpty()) {
            log("未发现 USB 串口设备 —— 检查 OTG 线、数传供电、以及数传是否被系统识别");
            btnStart.setText("开始转发");
            return;
        }
        UsbSerialDriver drv = drivers.get(0);
        UsbDevice dev = drv.getDevice();
        log("发现设备 " + dev.getDeviceName()
                + String.format(Locale.US, " VID=%04x PID=%04x", dev.getVendorId(), dev.getProductId()));
        if (um.hasPermission(dev)) {
            startBridge();
            return;
        }
        log("请求 USB 权限（系统会弹框，请点允许）…");
        int flags = PendingIntent.FLAG_MUTABLE;
        if (android.os.Build.VERSION.SDK_INT >= 31) flags |= PendingIntent.FLAG_MUTABLE;
        PendingIntent pi = PendingIntent.getBroadcast(this, 0,
                new Intent(ACTION_USB_PERMISSION).setPackage(getPackageName()), flags);
        um.requestPermission(dev, pi);
    }

    private void startBridge() {
        if (running) return;
        final int baud;
        final String host = etHost.getText().toString().trim();
        final int udpPort;
        try {
            baud = Integer.parseInt(etBaud.getText().toString().trim());
            udpPort = Integer.parseInt(etPort.getText().toString().trim());
        } catch (NumberFormatException e) {
            log("端口/波特率必须是数字");
            return;
        }
        if (host.isEmpty()) {
            log("目标 IP 不能为空");
            return;
        }

        new Thread(() -> {
            try {
                InetAddress addr = InetAddress.getByName(host);
                peer = new InetSocketAddress(addr, udpPort);
                udp = new DatagramSocket();
                log("UDP 就绪 → " + host + ":" + udpPort);

                UsbManager um = (UsbManager) getSystemService(Context.USB_SERVICE);
                List<UsbSerialDriver> drivers = UsbSerialProber.getDefaultProber().findAllDrivers(um);
                if (drivers.isEmpty()) {
                    log("设备已消失，无法打开");
                    return;
                }
                UsbSerialDriver drv = drivers.get(0);
                connection = um.openDevice(drv.getDevice());
                if (connection == null) {
                    log("openDevice 失败（权限或系统占用）");
                    return;
                }
                port = drv.getPorts().get(0);
                port.open(connection);
                port.setParameters(baud, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);
                log("串口已打开 " + baud + " 8N1，开始转发");

                running = true;
                usbToUdp = new Thread(this::pumpUsbToUdp, "usb->udp");
                udpToUsb = new Thread(this::pumpUdpToUsb, "udp->usb");
                usbToUdp.start();
                udpToUsb.start();
            } catch (Exception e) {
                log("启动失败：" + e.getClass().getSimpleName() + ": " + e.getMessage());
                stopBridge();
                ui.post(() -> btnStart.setText("开始转发"));
            }
        }, "start").start();
    }

    /** 串口 → UDP */
    private void pumpUsbToUdp() {
        byte[] buf = new byte[512];
        long lastStat = System.currentTimeMillis();
        while (running) {
            try {
                int n = port.read(buf, 500);            // 500 ms 超时
                if (n > 0) {
                    DatagramPacket p = new DatagramPacket(buf, n, peer);
                    udp.send(p);
                    cntToUdp += n;
                }
                long now = System.currentTimeMillis();
                if (now - lastStat > 3000) {
                    lastStat = now;
                    log(String.format(Locale.US, "串口→UDP %d B / UDP→串口 %d B", cntToUdp, cntToUsb));
                }
            } catch (IOException e) {
                if (running) log("串口读错误（可能已拔出）：" + e.getMessage());
                break;
            }
        }
    }

    /** UDP → 串口 */
    private void pumpUdpToUsb() {
        byte[] buf = new byte[4096];
        while (running) {
            try {
                DatagramPacket p = new DatagramPacket(buf, buf.length);
                udp.receive(p);                          // 阻塞，超时由 socket 关闭打断
                port.write(p.getData(), p.getLength(), 500);
                cntToUsb += p.getLength();
            } catch (Exception e) {
                if (running) log("UDP/串口写错误：" + e.getMessage());
                break;
            }
        }
    }

    private void stopBridge() {
        running = false;
        try { if (udp != null) udp.close(); } catch (Exception ignored) { }
        try { if (port != null) port.close(); } catch (Exception ignored) { }
        try { if (connection != null) connection.close(); } catch (Exception ignored) { }
        udp = null;
        port = null;
        connection = null;
        log("已停止");
    }

    private void log(String msg) {
        String ts = new SimpleDateFormat("HH:mm:ss", Locale.US).format(new Date());
        ui.post(() -> {
            tvLog.append(ts + "  " + msg + "\n");
            scrollLog.post(() -> scrollLog.fullScroll(View.FOCUS_DOWN));
        });
    }

    @Override
    protected void onDestroy() {
        stopBridge();
        try { unregisterReceiver(permReceiver); } catch (Exception ignored) { }
        super.onDestroy();
    }
}
