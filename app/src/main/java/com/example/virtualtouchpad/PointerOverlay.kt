package com.example.virtualtouchpad

import android.content.Context
import android.graphics.*
import android.os.Handler
import android.os.Looper
import android.util.AttributeSet
import android.view.View

class PointerOverlay @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null
) : View(context, attrs) {

    private var prevLandmarks: List<Pair<Float, Float>>? = null
    private val lerpAlpha = 1.0f

    var landmarks: List<Pair<Float, Float>> = emptyList()
        set(value) {
            if (prevLandmarks != null && prevLandmarks!!.size == value.size) {
                field = value.mapIndexed { i, (x, y) ->
                    val (prevX, prevY) = prevLandmarks!![i]
                    Pair(prevX + (x - prevX) * lerpAlpha, prevY + (y - prevY) * lerpAlpha)
                }
            } else {
                field = value
            }
            prevLandmarks = field
            invalidate()
        }

    private val pointerPaint = Paint().apply {
        color = Color.RED
        style = Paint.Style.FILL
        isAntiAlias = true
        alpha = 200
    }

    private val textPaint = Paint().apply {
        color = Color.WHITE
        textSize = 50f
        isAntiAlias = true
        typeface = Typeface.DEFAULT_BOLD
        setShadowLayer(5f, 3f, 3f, Color.BLACK)
    }

    // 시각 효과 관련
    private var feedbackPoint: Pair<Float, Float>? = null
    private var feedbackRadius = 0f
    private var feedbackAlpha = 0
    private val feedbackPaint = Paint().apply {
        style = Paint.Style.FILL
        isAntiAlias = true
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        if (landmarks.size < 21) return

        val connections = listOf(
            Pair(0,1), Pair(1,2), Pair(2,3), Pair(3,4),
            Pair(0,5), Pair(5,6), Pair(6,7), Pair(7,8),
            Pair(0,9), Pair(9,10), Pair(10,11), Pair(11,12),
            Pair(0,13), Pair(13,14), Pair(14,15), Pair(15,16),
            Pair(0,17), Pair(17,18), Pair(18,19), Pair(19,20),
            Pair(5,9), Pair(9,13), Pair(13,17)
        )

        for ((start, end) in connections) {
            val p1 = landmarks[start]
            val p2 = landmarks[end]
            canvas.drawLine(p1.first, p1.second, p2.first, p2.second, pointerPaint)
        }

        for ((i, point) in landmarks.withIndex()) {
            canvas.drawCircle(point.first, point.second, if (i == 8) 30f else 10f, pointerPaint)
        }

        val indexTip = landmarks[8]
        canvas.drawText(zMessage, indexTip.first + 40f, indexTip.second, textPaint)

        feedbackPoint?.let { (fx, fy) ->
            feedbackPaint.alpha = feedbackAlpha
            canvas.drawCircle(fx, fy, feedbackRadius, feedbackPaint)
        }
    }

    var zMessage: String = ""
        set(value) {
            field = value
            invalidate()
        }

    fun showFeedback(x: Float, y: Float, type: String) {
        feedbackPoint = Pair(x, y)
        feedbackRadius = 0f
        feedbackAlpha = 255

        feedbackPaint.color = when (type) {
            "tap" -> Color.YELLOW
            "long_press" -> Color.RED
            "drag" -> Color.CYAN
            "double_tap" -> Color.MAGENTA
            else -> Color.LTGRAY
        }

        animateFeedback()
    }

    private fun animateFeedback() {
        val handler = Handler(Looper.getMainLooper())
        val duration = 300L
        val startTime = System.currentTimeMillis()

        handler.post(object : Runnable {
            override fun run() {
                val elapsed = System.currentTimeMillis() - startTime
                val progress = elapsed.toFloat() / duration
                if (progress >= 1f) {
                    feedbackPoint = null
                    invalidate()
                    return
                }
                feedbackRadius = 40f + 60f * progress
                feedbackAlpha = (255 * (1 - progress)).toInt()
                invalidate()
                handler.postDelayed(this, 16)
            }
        })
    }
}
