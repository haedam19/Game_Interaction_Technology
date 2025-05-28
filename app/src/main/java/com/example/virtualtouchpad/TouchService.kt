package com.example.virtualtouchpad

import android.app.*
import android.content.*
import android.os.Build
import android.os.IBinder
import android.util.Log

class TouchService : Service() {
    override fun onCreate() {
        super.onCreate()
        startForegroundService()
        registerReceiver(receiver, IntentFilter("HAND_COORDINATES"), RECEIVER_NOT_EXPORTED)
    }

    override fun onDestroy() {
        super.onDestroy()
        unregisterReceiver(receiver)
    }

    private fun startForegroundService() {
        val channelId = "touch_service"
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            val channel = NotificationChannel(
                channelId,
                "Touch",
                NotificationManager.IMPORTANCE_LOW
            )
            getSystemService(NotificationManager::class.java).createNotificationChannel(channel)
        }

        val notification = Notification.Builder(this, channelId)
            .setContentTitle("포인터 제어 활성화")
            .setSmallIcon(android.R.drawable.ic_menu_compass)
            .build()

        startForeground(1, notification)
    }

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

    override fun onBind(intent: Intent?): IBinder? = null
}