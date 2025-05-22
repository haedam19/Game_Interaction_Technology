package com.example.virtualtouchpad

import android.content.Context
import android.graphics.*
import android.util.AttributeSet
import android.view.View

// 마커 표시
class PointerOverlay @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null
) : View(context, attrs) {

    private var prevLandmarks: List<Pair<Float, Float>>? = null
    private val lerpAlpha = 0.5f // 부드럽게 보간

    // 갱신 & 보간 적용
    var landmarks: List<Pair<Float, Float>> = emptyList()
        set(value) {
            if (prevLandmarks != null && prevLandmarks!!.size == value.size) {
                field = value.mapIndexed { i, (x, y) ->
                    val (prevX, prevY) = prevLandmarks!![i]
                    Pair(
                        prevX + (x - prevX) * lerpAlpha,
                        prevY + (y - prevY) * lerpAlpha
                    )
                }
            } else {
                field = value
            }
            prevLandmarks = field
            invalidate()
        }

    // 포인터 스타일 정의
    private val pointerPaint = Paint().apply {
        color = Color.RED
        style = Paint.Style.FILL
        isAntiAlias = true
        alpha = 200
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        if (landmarks.size < 21) return

        // 각 손가락 연결
        val connections = listOf(
            // 손바닥
            Pair(0,1), Pair(1,2), Pair(2,3), Pair(3,4),    // 엄지
            Pair(0,5), Pair(5,6), Pair(6,7), Pair(7,8),    // 검지
            Pair(0,9), Pair(9,10), Pair(10,11), Pair(11,12), // 중지
            Pair(0,13), Pair(13,14), Pair(14,15), Pair(15,16), // 약지
            Pair(0,17), Pair(17,18), Pair(18,19), Pair(19,20), // 새끼
            // 손가락 옆줄
            Pair(5,9), Pair(9,13), Pair(13,17)
        )

        // 점 연결
        for ((start, end) in connections) {
            val p1 = landmarks[start]
            val p2 = landmarks[end]
            canvas.drawLine(p1.first, p1.second, p2.first, p2.second, pointerPaint)
        }

        // 점 그리기
        // 검지만 크게
        for ((i, point) in landmarks.withIndex()) {
            canvas.drawCircle(point.first, point.second, if (i == 8) 30f else 10f, pointerPaint)
        }
    }

}
