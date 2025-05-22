package com.example.virtualtouchpad

import android.app.*
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.os.Build
import android.os.IBinder
import android.util.Log

// 터치 조작 관련 서비스
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
            val channel = NotificationChannel(channelId, "Touch", NotificationManager.IMPORTANCE_LOW)
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
            if (x < 0 || y < 0) return

            val service = TouchAccessibilityService.instance
            if (service != null) {
                service.performTouch(x, y)
            } else {
                Log.w("TouchService", "TouchAccessibilityService is not active.")
            }
        }
    }

    override fun onBind(intent: Intent?): IBinder? = null
}