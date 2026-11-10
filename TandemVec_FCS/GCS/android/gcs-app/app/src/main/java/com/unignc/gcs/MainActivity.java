package com.unignc.gcs;

import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
import android.view.KeyEvent;
import android.view.ViewGroup;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.webkit.WebViewClient;
import android.widget.TextView;

import com.chaquo.python.PyObject;
import com.chaquo.python.Python;
import com.chaquo.python.android.AndroidPlatform;

/**
 * UniGNC 地面站（安卓版）主界面。
 *
 * <p>做法：在应用进程内用 Chaquopy 起 CPython，直接跑现有的 FastAPI 后端，
 * 再用 WebView 加载 http://127.0.0.1:8091。前端一行不用改，AnoCom / MAVLink
 * 解析与频谱分析（numpy）全部沿用现有 Python 代码——这是不打 APK 之外
 * 唯一不产生"第二套实现"的路子。
 *
 * <p>启动顺序：先显示"正在启动"→ 后台线程启动 Python 后端 → 轮询端口就绪 →
 * 切到 WebView。失败时停留在提示页并显示原因（现场没有 adb 也能看到问题）。
 */
public class MainActivity extends Activity {

    private static final String TAG = "UniGNC";
    private static final int PORT = 8091;

    private WebView web;
    private TextView status;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        status = new TextView(this);
        status.setText("正在启动地面站后端…\n首次启动需要初始化 Python 运行环境，可能要十几秒。");
        status.setTextSize(16f);
        status.setPadding(48, 96, 48, 48);
        setContentView(status);

        // USB 桥必须先拿到 Context 才能枚举设备；Python 侧会通过 Chaquopy 调它
        UsbSerialBridge.init(this);

        new Thread(this::bootBackend, "py-boot").start();
    }

    /** 后台启动 Python 后端，成功后再切 WebView */
    private void bootBackend() {
        try {
            if (!Python.isStarted()) {
                Python.start(new AndroidPlatform(getApplicationContext()));
            }
            PyObject mod = Python.getInstance().getModule("gcs_android_main");
            PyObject r = mod.callAttr("start_backend", PORT);
            int port = r.toInt();
            Log.i(TAG, "Python 后端已启动，端口 " + port);
            waitThenShow(port);
        } catch (Throwable e) {
            Log.e(TAG, "后端启动失败", e);
            uiStatus("后端启动失败：\n" + e.getClass().getSimpleName() + ": " + e.getMessage()
                    + "\n\n把这段信息发出来即可定位（多与 Python 依赖或权限有关）。");
        }
    }

    private void waitThenShow(int port) {
        String url = "http://127.0.0.1:" + port + "/";
        for (int i = 0; i < 120; i++) {          // 最多等 60 s：首启用 numpy 初始化较慢
            if (httpReady(url)) {
                uiShowWeb(url);
                return;
            }
            try { Thread.sleep(500); } catch (InterruptedException ignored) { }
        }
        uiStatus("后端 60 秒内没有就绪。\n可能是端口被占用或 Python 依赖缺失，"
                + "重启应用再试；仍不行就把这句话连同日志发出来。");
    }

    /** 探活：能不能拿到 200。用最轻的方式，避免引入额外依赖。 */
    private boolean httpReady(String url) {
        try {
            java.net.HttpURLConnection c =
                    (java.net.HttpURLConnection) new java.net.URL(url).openConnection();
            c.setConnectTimeout(1000);
            c.setReadTimeout(1000);
            int code = c.getResponseCode();
            c.disconnect();
            return code == 200;
        } catch (Exception e) {
            return false;
        }
    }

    private void uiStatus(String text) {
        runOnUiThread(() -> status.setText(text));
    }

    private void uiShowWeb(String url) {
        runOnUiThread(() -> {
            web = new WebView(this);
            WebSettings s = web.getSettings();
            s.setJavaScriptEnabled(true);
            s.setDomStorageEnabled(true);
            s.setAllowFileAccess(false);
            s.setCacheMode(WebSettings.LOAD_NO_CACHE);   // 改前端后重开即最新，便于迭代
            s.setMediaPlaybackRequiresUserGesture(false); // 视频流页面需要自动播放
            web.setWebViewClient(new WebViewClient());
            web.loadUrl(url);
            setContentView(web, new ViewGroup.LayoutParams(
                    ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT));
        });
    }

    @Override
    public boolean onKeyDown(int keyCode, KeyEvent event) {
        // 返回键交给页面历史，退到底再退出应用
        if (keyCode == KeyEvent.KEYCODE_BACK && web != null && web.canGoBack()) {
            web.goBack();
            return true;
        }
        return super.onKeyDown(keyCode, event);
    }
}
