package com.example.virtualtouchpad

import android.accessibilityservice.AccessibilityService
import android.accessibilityservice.GestureDescription
import android.graphics.Path
import android.os.Handler
import android.os.Looper
import android.util.Log
import android.view.accessibility.AccessibilityEvent

class TouchAccessibilityService : AccessibilityService() {
    companion object {
        var instance: TouchAccessibilityService? = null
    }

    override fun onServiceConnected() {
        instance = this
        PointerOverlayManager.init(this)
        Log.d("TouchService", "AccessibilityService connected.")
    }

    override fun onAccessibilityEvent(event: AccessibilityEvent?) {}
    override fun onInterrupt() {}

    fun performTouch(x: Float, y: Float, duration: Long = 80L) {
        PointerOverlayManager.show(x, y, "tap")
        val path = Path().apply { moveTo(x, y) }
        val gesture = GestureDescription.Builder()
            .addStroke(GestureDescription.StrokeDescription(path, 0, duration))
            .build()
        dispatchGesture(gesture, object : GestureResultCallback() {
            override fun onCompleted(gestureDescription: GestureDescription?) {
                Log.d("TouchService", "Tap completed at ($x, $y)")
            }

            override fun onCancelled(gestureDescription: GestureDescription?) {
                Log.w("TouchService", "Tap cancelled at ($x, $y)")
            }
        }, null)
    }

    fun performLongPress(x: Float, y: Float, duration: Long = 600L) {
        PointerOverlayManager.show(x, y, "long_press")
        val path = Path().apply { moveTo(x, y) }
        val gesture = GestureDescription.Builder()
            .addStroke(GestureDescription.StrokeDescription(path, 0, duration))
            .build()
        dispatchGesture(gesture, object : GestureResultCallback() {
            override fun onCompleted(gestureDescription: GestureDescription?) {
                Log.d("TouchService", "Long press completed at ($x, $y)")
            }

            override fun onCancelled(gestureDescription: GestureDescription?) {
                Log.w("TouchService", "Long press cancelled at ($x, $y)")
            }
        }, null)
    }

    fun performDrag(x1: Float, y1: Float, x2: Float, y2: Float, duration: Long = 300L) {
        PointerOverlayManager.show(x1, y1, "drag")
        val path = Path().apply {
            moveTo(x1, y1)
            lineTo(x2, y2)
        }
        val gesture = GestureDescription.Builder()
            .addStroke(GestureDescription.StrokeDescription(path, 0, duration))
            .build()
        dispatchGesture(gesture, object : GestureResultCallback() {
            override fun onCompleted(gestureDescription: GestureDescription?) {
                Log.d("TouchService", "Drag completed from ($x1, $y1) to ($x2, $y2)")
            }

            override fun onCancelled(gestureDescription: GestureDescription?) {
                Log.w("TouchService", "Drag cancelled from ($x1, $y1) to ($x2, $y2)")
            }
        }, null)
    }

    fun performDoubleTap(x: Float, y: Float) {
        PointerOverlayManager.show(x, y, "double_tap")
        performTouch(x, y)
        Handler(Looper.getMainLooper()).postDelayed({
            performTouch(x, y)
        }, 150)
        Log.d("TouchService", "Double tap at ($x, $y)")
    }
}
