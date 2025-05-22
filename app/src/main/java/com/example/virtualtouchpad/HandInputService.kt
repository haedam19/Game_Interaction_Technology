package com.example.virtualtouchpad

import android.content.Intent
import android.graphics.Bitmap
import android.graphics.PixelFormat
import android.os.Binder
import android.os.IBinder
import android.util.Log
import android.util.Size
import android.view.WindowManager
import androidx.camera.core.*
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.core.content.ContextCompat
import androidx.lifecycle.LifecycleService
import com.google.mediapipe.framework.image.BitmapImageBuilder
import com.google.mediapipe.tasks.core.BaseOptions
import com.google.mediapipe.tasks.vision.core.RunningMode
import com.google.mediapipe.tasks.vision.handlandmarker.HandLandmarker
import java.util.concurrent.Executors

// 백그라운드 실행
// 실제 분석은 백그라운드에서 진행
class HandInputService : LifecycleService() {
    // 서비스 바인더 정의
    inner class LocalBinder : Binder() {
        fun getService(): HandInputService = this@HandInputService
    }

    // 바인드 요청 처리
    override fun onBind(intent: Intent): IBinder {
        super.onBind(intent)
        return LocalBinder()
    }

    companion object {
        var instance: HandInputService? = null
    }

    private lateinit var handLandmarker: HandLandmarker
    private lateinit var pointerOverlay: PointerOverlay
    private lateinit var windowManager: WindowManager
    private val executor = Executors.newSingleThreadExecutor()

    private var cameraRunning = false
    private var cameraProvider: ProcessCameraProvider? = null
    private var imageAnalysis: ImageAnalysis? = null

    private var rotation: Int = 0

    // 서비스 생성 시 오버레이와 MediaPipe 초기화
    override fun onCreate() {
        super.onCreate()
        instance = this
        setupOverlay()
        setupMediaPipe()
    }

    // 서비스 종료 시 오버레이 제거 및 카메라 정리
    override fun onDestroy() {
        if (::pointerOverlay.isInitialized) {
            windowManager.removeView(pointerOverlay)
        }
        instance = null
        stopCameraIfRunning()
        super.onDestroy()
    }

    // 외부에서 프레임(Bitmap) 전달받아 분석 요청
    fun receiveBitmap(bitmap: Bitmap) {
        val mpImage = BitmapImageBuilder(bitmap).build()
        handLandmarker.detectAsync(mpImage, System.currentTimeMillis())
    }

    // MediaPipe HandLandmarker 초기화 및 결과 콜백 처리
    private fun setupMediaPipe() {
        val options = HandLandmarker.HandLandmarkerOptions.builder()
            .setBaseOptions(BaseOptions.builder().setModelAssetPath("hand_landmarker.task").build())
            .setNumHands(1)
            .setRunningMode(RunningMode.LIVE_STREAM)
            .setResultListener { result, input ->
                result.landmarks().firstOrNull()?.let { landmarks ->
                    val imageWidth = input.width.toFloat()
                    val imageHeight = input.height.toFloat()

                    pointerOverlay.post {
                        val viewWidth = pointerOverlay.width.toFloat()
                        val viewHeight = pointerOverlay.height.toFloat()

                        val scale = maxOf(viewWidth / imageWidth, viewHeight / imageHeight)
                        val dx = (viewWidth - imageWidth * scale) / 2f
                        val dy = (viewHeight - imageHeight * scale) / 2f

                        val landmarkPoints = landmarks.map {
                            val x = (1.0f - it.y()) * imageWidth
                            val y = it.x() * imageHeight
                            val viewX = x * scale + dx
                            val viewY = y * scale + dy
                            Pair(viewX, viewY)
                        }

                        pointerOverlay.landmarks = landmarkPoints
                    }
                }
            }
            .build()

        handLandmarker = HandLandmarker.createFromOptions(this, options)
    }

    // 시스템 오버레이로 포인터 출력용 뷰 추가
    private fun setupOverlay() {
        windowManager = getSystemService(WINDOW_SERVICE) as WindowManager
        pointerOverlay = PointerOverlay(this)

        val params = WindowManager.LayoutParams(
            WindowManager.LayoutParams.MATCH_PARENT,
            WindowManager.LayoutParams.MATCH_PARENT,
            if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.O)
                WindowManager.LayoutParams.TYPE_APPLICATION_OVERLAY
            else
                WindowManager.LayoutParams.TYPE_PHONE,
            WindowManager.LayoutParams.FLAG_NOT_FOCUSABLE or
                    WindowManager.LayoutParams.FLAG_LAYOUT_IN_SCREEN or
                    WindowManager.LayoutParams.FLAG_NOT_TOUCHABLE or
                    WindowManager.LayoutParams.FLAG_LAYOUT_NO_LIMITS,
            PixelFormat.TRANSLUCENT
        )

        windowManager.addView(pointerOverlay, params)
    }

    // 서비스가 카메라 직접 실행 (앱 백그라운드일 때
    fun startCameraIfNeeded() {
        Log.d("HandInputService", "startCameraIfNeeded() called, cameraRunning = $cameraRunning")
        stopCameraIfRunning() // 강제 초기화 후 재시작 시도

        val cameraProviderFuture = ProcessCameraProvider.getInstance(this)
        cameraProviderFuture.addListener({
            cameraProvider = cameraProviderFuture.get()

            val analysis = ImageAnalysis.Builder()
                .setTargetResolution(Size(640, 480))
                .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                .setOutputImageFormat(ImageAnalysis.OUTPUT_IMAGE_FORMAT_RGBA_8888)
                .build()

            analysis.setAnalyzer(executor) { imageProxy ->
                val bitmap = imageProxyToBitmap(imageProxy)
                receiveBitmap(bitmap)
                imageProxy.close()
            }

            imageAnalysis = analysis
            cameraProvider?.unbindAll()
            cameraProvider?.bindToLifecycle(this, CameraSelector.DEFAULT_BACK_CAMERA, analysis)

            cameraRunning = true
        }, ContextCompat.getMainExecutor(this))
    }

    // 서비스가 실행 중인 카메라 종료 (앱이 포그라운드일 때 호출)
    fun stopCameraIfRunning() {
        if (!cameraRunning) return
        Log.d("HandInputService", "stopCameraIfRunning() called")
        imageAnalysis?.clearAnalyzer()
        cameraProvider?.unbindAll()
        cameraRunning = false
    }

    // ImageProxy를 Bitmap으로 변환
    private fun imageProxyToBitmap(imageProxy: ImageProxy): Bitmap {
        val plane = imageProxy.planes[0]
        val buffer = plane.buffer
        val pixelStride = plane.pixelStride
        val rowStride = plane.rowStride
        val rowPadding = rowStride - pixelStride * imageProxy.width

        val bitmap = Bitmap.createBitmap(
            imageProxy.width + rowPadding / pixelStride,
            imageProxy.height,
            Bitmap.Config.ARGB_8888
        )
        bitmap.copyPixelsFromBuffer(buffer)
        this.rotation = imageProxy.imageInfo.rotationDegrees
        return Bitmap.createBitmap(bitmap, 0, 0, imageProxy.width, imageProxy.height)
    }
}