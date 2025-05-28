package com.example.virtualtouchpad

import android.content.Context
import android.view.WindowManager

object PointerOverlayManager {
    var overlay: PointerOverlay? = null

    fun init(context: Context) {
        if (overlay != null) return
        overlay = PointerOverlay(context)
        val params = WindowManager.LayoutParams().apply {
            width = WindowManager.LayoutParams.MATCH_PARENT
            height = WindowManager.LayoutParams.MATCH_PARENT
            type = WindowManager.LayoutParams.TYPE_ACCESSIBILITY_OVERLAY
            flags = WindowManager.LayoutParams.FLAG_NOT_FOCUSABLE or
                    WindowManager.LayoutParams.FLAG_NOT_TOUCHABLE or
                    WindowManager.LayoutParams.FLAG_LAYOUT_IN_SCREEN
            format = android.graphics.PixelFormat.TRANSLUCENT
        }
        val wm = context.getSystemService(Context.WINDOW_SERVICE) as WindowManager
        wm.addView(overlay, params)
    }

    fun show(x: Float, y: Float, type: String) {
        overlay?.showFeedback(x, y, type)
    }
}
