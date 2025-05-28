package com.example.virtualtouchpad

import android.graphics.Bitmap

object NativeLib {
    init {
        System.loadLibrary("native-lib")
    }

    external fun initCalibrationCache(path: String)
    external fun detectArucoMarkers(bitmap: Bitmap): Boolean
    external fun saveCalibrationImage(bitmap: Bitmap): Boolean
    external fun calibrateFromSavedImages(): Boolean
    external fun loadCalibrationParams(): Boolean
    external fun estimatePose(bitmap: Bitmap): FloatArray?
    external fun updateLandmarks(landmarks: FloatArray)
    external fun calibrateHand()
    external fun estimateDepth(): Boolean
    external fun estimateIndexTip(): Boolean
    external fun getLandmarkWorld(index: Int): FloatArray
}
