package com.example.virtualtouchpad

import android.content.Context
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
import com.example.virtualtouchpad.filters.Landmark
import com.google.mediapipe.framework.image.BitmapImageBuilder
import com.google.mediapipe.tasks.core.BaseOptions
import com.google.mediapipe.tasks.vision.core.RunningMode
import com.google.mediapipe.tasks.vision.handlandmarker.HandLandmarker
import java.util.concurrent.Executors
import com.example.virtualtouchpad.filters.LandmarkFilterManager
import java.io.File

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
    private var lastBitmap: Bitmap? = null
    private val landmarkFilterManager = LandmarkFilterManager(
        freq = 30.0,
        minCutoff = 1.0,
        beta = 5.0,
        dCutoff = 1.0
    )

    private var cameraPos: FloatArray? = null
    private var cameraView: FloatArray? = null
    private var rotationMatrix: FloatArray? = null
    var isCalibrating = false

    // 손끝 좌표 상태 저장용 변수들
    private var isTouching = false
    private var touchStartTime: Long? = null
    private var alreadyTriggered = false

    val zPressThreshold = 0.02f   // 손을 내릴 때
    val zReleaseThreshold = 0.05f  // 손을 뗄 때
    private val longPressThreshold = 600L // ms

    private var isPoseValid: Boolean = false
    private var captureHand: Boolean = false
    private var frameIdx = 0

    // 서비스 생성 시 오버레이와 MediaPipe 초기화
    override fun onCreate() {
        super.onCreate()
        instance = this
        setupOverlay()
        setupMediaPipe()

        val cachePath = filesDir.absolutePath + "/calibration"
        val calibrationDir = File(cachePath)
        calibrationDir.mkdirs()
        calibrationDir.listFiles()?.forEach { it.delete() }
        NativeLib.initCalibrationCache(cachePath)
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
        val config = bitmap.config ?: Bitmap.Config.ARGB_8888
        lastBitmap = bitmap.copy(config, false)

        NativeLib.estimatePose(lastBitmap!!)?.takeIf { it.size == 15 }?.let { pose ->
            isPoseValid = true
            cameraPos = pose.sliceArray(0..2)
            cameraView = pose.sliceArray(3..5)
            rotationMatrix = pose.sliceArray(6..14)
            Log.d("ArucoPose", "Cam Pos: ${cameraPos?.toList()}, View: ${cameraView?.toList()}")
        } ?: run {
            isPoseValid = false
        }

        val mpImage = BitmapImageBuilder(bitmap).build()
        handLandmarker.detectAsync(mpImage, System.currentTimeMillis())
    }

    fun saveCurrentFrame(type: String): Boolean {
        lastBitmap?.let {
            var success = true
            if (type == "camera") success = NativeLib.saveCalibrationImage(it)
            else if (type == "hand") captureHand = true
            return success
        }
        return false
    }

    fun runCalibration(): Boolean {
        return NativeLib.calibrateFromSavedImages()
    }

    fun saveLandmarksToFile(landmarks: List<Landmark>) {
        if (cameraPos == null || rotationMatrix == null) {
            Log.e("Calib", "cameraPos or rotationMatrix is null!")
            return
        }
        val folder = filesDir.absolutePath + "/calibration/hand"
        val dir = File(folder)
        dir.mkdirs()
        val filename = File(dir, "frame_%04d.txt".format(frameIdx++))
        filename.bufferedWriter().use { out ->
            out.write("%.8f,%.8f,%.8f\n".format(cameraPos!![0], cameraPos!![1], cameraPos!![2]))
            out.write(rotationMatrix!!.joinToString(",") { "%.8f".format(it) })
            out.write("\n")
            landmarks.forEach { lmk ->
                out.write("%.8f,%.8f\n".format(lmk.u, lmk.v))
            }
        }
    }

    // MediaPipe HandLandmarker 초기화 및 결과 콜백 처리
    private fun setupMediaPipe() {
        val mainHandler = android.os.Handler(android.os.Looper.getMainLooper())
        var message: String? = null
        val options = HandLandmarker.HandLandmarkerOptions.builder()
            .setBaseOptions(BaseOptions.builder().setModelAssetPath("hand_landmarker.task").build())
            .setNumHands(1)
            .setRunningMode(RunningMode.LIVE_STREAM)
            .setResultListener { result, input ->
                result.landmarks().firstOrNull()?.let { landmarks ->
                    val imageWidth = input.width.toFloat()
                    val imageHeight = input.height.toFloat()

                    val rawLandmarks = landmarks.map {
                        Landmark(
                            u = (1.0 - it.y()).toDouble(),
                            v = it.x().toDouble(),
                            z = it.z().toDouble()
                        )
                    }
                    // 유로 필터
                    val filtered = landmarkFilterManager.filter(rawLandmarks)

                    val landmarkArray = FloatArray(21 * 3)
                    filtered.forEachIndexed { index, it ->
                        landmarkArray[index * 3 + 0] = it.v.toFloat()
                        landmarkArray[index * 3 + 1] = (1.0 - it.u).toFloat()
                        landmarkArray[index * 3 + 2] = it.z.toFloat()
                    }
                    NativeLib.updateLandmarks(landmarkArray)

                    if (captureHand && lastBitmap != null && isCalibrating) {
                        saveLandmarksToFile(filtered)
                        NativeLib.calibrateHandFromLandmarkFiles()
                        captureHand = false
                    }

                    if (isPoseValid && NativeLib.estimateDepth()) {
                        NativeLib.estimateIndexTip()
                        val tipCoord = NativeLib.getLandmarkWorld(8)
                        val x = tipCoord[0]
                        val y = tipCoord[1]
                        val z = tipCoord[2]

                        val zInCm = z * 100
                        message = if (zInCm < 0)
                            "마커 뒤쪽 %.1f cm".format(-zInCm)
                        else
                            "마커 앞쪽 %.1f cm".format(zInCm)

                        Log.d("FingerTip", "Index tip 3D = ($x, $y, $z)")

                        val screenWidth = resources.displayMetrics.widthPixels
                        val screenHeight = resources.displayMetrics.heightPixels
                        val screenX = ((x + 0.05) / 0.10 * screenWidth).toFloat().coerceIn(0f, screenWidth - 1f)
                        val screenY = ((-y + 0.05) / 0.10 * screenHeight).toFloat().coerceIn(0f, screenHeight - 1f)

                        handleDepthTouch(this, screenX, screenY, z)
                    } else {
                        Log.w("Depth", "estimateDepth 불가 - pose valid = $isPoseValid")
                        message = "깊이 추정 불가 (pose 없음)"
                    }

                    val viewWidth = pointerOverlay.width.toFloat()
                    val viewHeight = pointerOverlay.height.toFloat()
                    val scale = maxOf(viewWidth / imageWidth, viewHeight / imageHeight)
                    val dx = (viewWidth - imageWidth * scale) / 2f
                    val dy = (viewHeight - imageHeight * scale) / 2f
                    val overlayLandmarks = filtered.map {
                        val x = (it.u * imageWidth).toFloat()
                        val y = (it.v * imageHeight).toFloat()
                        Pair(x * scale + dx, y * scale + dy)
                    }

                    mainHandler.post {
                        pointerOverlay.landmarks = overlayLandmarks
                        pointerOverlay.zMessage = message ?: ""
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

    fun sendTouchIntent(context: Context, x: Float, y: Float, type: String) {
        val intent = Intent("HAND_COORDINATES").apply {
            putExtra("x", x)
            putExtra("y", y)
            putExtra("type", type)
        }
        context.sendBroadcast(intent)
    }

    // 손가락 깊이에 따라 터치 실행
    // 왠지는 모르겠지만 안됨
    fun handleDepthTouch(context: Context, x: Float, y: Float, z: Float) {
        val now = System.currentTimeMillis()

        if (!isTouching && z < zPressThreshold) {
            touchStartTime = now
            isTouching = true
            alreadyTriggered = false
        }

        if (isTouching && !isCalibrating) {
            if (z < zReleaseThreshold) {
                val held = now - (touchStartTime ?: now)
                if (held > longPressThreshold && !alreadyTriggered) {
                    alreadyTriggered = true
                    Log.d("TouchLogic", "롱터치 실행")
                    sendTouchIntent(context, x, y, "long_press")
                }
            } else {
                if (!alreadyTriggered) {
                    Log.d("TouchLogic", "탭 실행")
                    sendTouchIntent(context, x, y, "tap")
                }
                isTouching = false
                alreadyTriggered = false
                touchStartTime = null
            }
        }
    }

    fun initHandCalibration() {
        val cachePath = filesDir.absolutePath + "/calibration/hand"
        val calibrationDir = File(cachePath)
        calibrationDir.mkdirs()
        calibrationDir.listFiles()?.forEach { it.delete() }
        frameIdx = 0
    }
}