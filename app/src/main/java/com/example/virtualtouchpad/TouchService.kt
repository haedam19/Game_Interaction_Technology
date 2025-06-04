package com.example.virtualtouchpad

import android.app.Service
import android.content.*
import android.os.Build
import android.os.IBinder
import android.util.Log

class TouchService : Service() {

    private val receiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context?, intent: Intent?) {
            val x = intent?.getFloatExtra("x", -1f) ?: return
            val y = intent.getFloatExtra("y", -1f)
            val type = intent.getStringExtra("type") ?: "tap"

            val service = TouchAccessibilityService.instance

            if (x < 0 || y < 0 || service == null) {
                Log.w("TouchService", "Invalid state or coordinates. type=$type, x=$x, y=$y, service=$service")
                return
            }

            when (type) {
                "tap" -> service.performTouch(x, y)
                "long_press" -> service.performLongPress(x, y)
                "double_tap" -> service.performDoubleTap(x, y)
                "drag" -> {
                    val x2 = intent.getFloatExtra("x2", x)
                    val y2 = intent.getFloatExtra("y2", y)
                    service.performDrag(x, y, x2, y2)
                }
                else -> Log.w("TouchService", "알 수 없는 type: $type")
            }
        }
    }

    override fun onCreate() {
        super.onCreate()
        val flags = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            Context.RECEIVER_NOT_EXPORTED
        } else {
            0
        }
        registerReceiver(receiver, IntentFilter("HAND_COORDINATES"), flags)
    }

    override fun onDestroy() {
        super.onDestroy()
        unregisterReceiver(receiver)
    }

    override fun onBind(intent: Intent?): IBinder? = null
}