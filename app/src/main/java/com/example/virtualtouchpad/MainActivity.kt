package com.example.virtualtouchpad

import android.Manifest
import android.content.*
import android.graphics.Bitmap
import android.net.Uri
import android.os.Bundle
import android.os.IBinder
import android.provider.Settings
import android.util.Size
import android.view.GestureDetector
import android.view.MotionEvent
import android.widget.Toast
import androidx.activity.ComponentActivity
import androidx.camera.core.*
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.camera.view.PreviewView
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import java.util.concurrent.Executors

// 앱 화면이 실행되면 작동
// 카메라 화면을 확인하기 위한 용도
class MainActivity : ComponentActivity() {
    private lateinit var previewView: PreviewView
    private var cameraProvider: ProcessCameraProvider? = null
    private var imageAnalysis: ImageAnalysis? = null
    private var handService: HandInputService? = null
    private val REQUEST_CODE_ALL_PERMISSIONS = 100
    private var cameraCalibrated = false
    private var handCalibrating = false

    // 서비스 바인딩 콜백
    private val connection = object : ServiceConnection {
        override fun onServiceConnected(className: ComponentName, service: IBinder) {
            val binder = service as HandInputService.LocalBinder
            handService = binder.getService()
        }

        override fun onServiceDisconnected(arg0: ComponentName) {
            handService = null
        }
    }

    // 액티비티 생성 시: UI 초기화 및 서비스 바인딩
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        ActivityCompat.requestPermissions(
            this,
            arrayOf(
                Manifest.permission.CAMERA,
                Manifest.permission.RECORD_AUDIO,
                Manifest.permission.READ_EXTERNAL_STORAGE
            ),
            REQUEST_CODE_ALL_PERMISSIONS
        )

        if (TouchAccessibilityService.instance == null) {
            openAccessibilitySettings(this)
        }

        if (!Settings.canDrawOverlays(this)) {
            val intent = Intent(
                Settings.ACTION_MANAGE_OVERLAY_PERMISSION,
                Uri.parse("package:$packageName")
            )
            intent.addFlags(Intent.FLAG_ACTIVITY_NEW_TASK)
            startActivity(intent)
        }

        window.addFlags(android.view.WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

        previewView = findViewById(R.id.previewView)

        val gestureDetector = GestureDetector(this, object : GestureDetector.SimpleOnGestureListener() {
            override fun onSingleTapUp(e: MotionEvent): Boolean {
                var success = false
                if (!cameraCalibrated) success = handService?.saveCurrentFrame("camera") ?: false
                else if (handCalibrating) success = handService?.saveCurrentFrame("hand") ?: false
                Toast.makeText(this@MainActivity, if (success) "프레임 저장 완료" else "프레임 저장 실패", Toast.LENGTH_SHORT).show()
                return true
            }

            override fun onLongPress(e: MotionEvent) {
                if (!cameraCalibrated) {
                    val success = handService?.runCalibration() ?: false
                    if (success) {
                        cameraCalibrated = true
                        Toast.makeText(this@MainActivity, "카메라 캘리브레이션 완료", Toast.LENGTH_SHORT).show()
                    } else {
                        Toast.makeText(this@MainActivity, "카메라 캘리브레이션 실패", Toast.LENGTH_SHORT).show()
                    }
                } else {
                    handCalibrating = !handCalibrating
                    handService?.isCalibrating = handCalibrating

                    if (handCalibrating) {
                        val success = handService?.initHandCalibration()
                        Toast.makeText(this@MainActivity, "손 캘리브레이션 모드 진입", Toast.LENGTH_SHORT).show()
                    } else {
                        Toast.makeText(this@MainActivity, "손 캘리브레이션 모드 종료", Toast.LENGTH_SHORT).show()
                    }
                }
            }

        })

        previewView.setOnTouchListener { _, event ->
            gestureDetector.onTouchEvent(event)
            true
        }

        val serviceIntent = Intent(this, HandInputService::class.java)
        bindService(serviceIntent, connection, Context.BIND_AUTO_CREATE)

        val intent = Intent(this, TouchService::class.java)
        ContextCompat.startForegroundService(this, intent)
    }

    // 액티비티 화면 진입 시: 서비스 측 카메라 중지 → 프리뷰 + 분석 시작
    override fun onStart() {
        super.onStart()

        previewView.postDelayed({
            startCameraWithAnalysis()
        }, 300)
    }


    // 액티비티 화면 빠져나갈 때: 프리뷰 종료, 서비스 카메라 시작
    override fun onStop() {
        super.onStop()
        stopCamera()
        handService!!.startCameraIfNeeded()
    }

    // 액티비티 종료 시: 서비스 언바인딩
    override fun onDestroy() {
        super.onDestroy()
        unbindService(connection)
    }

    // 카메라 프리뷰 + 분석용 카메라 시작
    private fun startCameraWithAnalysis() {
        val cameraProviderFuture = ProcessCameraProvider.getInstance(this)
        cameraProviderFuture.addListener({
            cameraProvider = cameraProviderFuture.get()

            val preview = Preview.Builder()
                .setTargetResolution(Size(640, 480))
                .build()
                .also { it.setSurfaceProvider(previewView.surfaceProvider) }

            imageAnalysis = ImageAnalysis.Builder()
                .setTargetResolution(Size(640, 480))
                .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                .setOutputImageFormat(ImageAnalysis.OUTPUT_IMAGE_FORMAT_RGBA_8888)
                .build()

            imageAnalysis!!.setAnalyzer(Executors.newSingleThreadExecutor()) { imageProxy ->
                val bitmap = imageProxyToBitmap(imageProxy)
                handService?.receiveBitmap(bitmap)
                imageProxy.close()
            }

            cameraProvider?.unbindAll()
            cameraProvider?.bindToLifecycle(this, CameraSelector.DEFAULT_BACK_CAMERA, preview, imageAnalysis)
        }, ContextCompat.getMainExecutor(this))
    }

    // 프리뷰와 분석 해제
    private fun stopCamera() {
        imageAnalysis?.clearAnalyzer()
        cameraProvider?.unbindAll()
    }

    // ImageProxy → Bitmap 변환 (MediaPipe에 전달하기 위해)
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
        return Bitmap.createBitmap(bitmap, 0, 0, imageProxy.width, imageProxy.height)
    }

    // 접근성 제어 화면
    fun openAccessibilitySettings(context: Context) {
        val intent = Intent(android.provider.Settings.ACTION_ACCESSIBILITY_SETTINGS)
        intent.flags = Intent.FLAG_ACTIVITY_NEW_TASK
        context.startActivity(intent)
    }
}
