package com.example.virtualtouchpad.filters

import kotlin.math.*

data class Landmark(val u: Double, val v: Double, val z: Double)

class LowPassFilter(private var alpha: Double) {
    private var s: Double? = null

    fun filter(x: Double): Double {
        s = if (s == null) x else alpha * x + (1 - alpha) * s!!
        return s!!
    }

    fun setAlpha(alpha: Double) {
        this.alpha = alpha
    }
}

class OneEuroFilter(
    private val freq: Double,
    private var minCutoff: Double = 1.0,
    private var beta: Double = 0.0,
    private var dCutoff: Double = 1.0
) {
    private val xFilter = LowPassFilter(alpha(minCutoff))
    private val dxFilter = LowPassFilter(alpha(dCutoff))
    private var lastX: Double? = null

    private fun alpha(cutoff: Double): Double {
        val tau = 1.0 / (2 * Math.PI * cutoff)
        val te = 1.0 / freq
        return 1.0 / (1.0 + tau / te)
    }

    fun filter(x: Double): Double {
        val dx = if (lastX == null) 0.0 else (x - lastX!!) * freq
        lastX = x

        val edx = dxFilter.filter(dx)
        val cutoff = minCutoff + beta * abs(edx)
        xFilter.setAlpha(alpha(cutoff))
        return xFilter.filter(x)
    }
}

class OneEuroFilter2D(freq: Double, minCutoff: Double = 1.0, beta: Double = 0.0, dCutoff: Double = 1.0) {
    private val fx = OneEuroFilter(freq, minCutoff, beta, dCutoff)
    private val fy = OneEuroFilter(freq, minCutoff, beta, dCutoff)

    fun filter(u: Double, v: Double): Pair<Double, Double> {
        return Pair(fx.filter(u), fy.filter(v))
    }
}

class LandmarkFilterManager(
    numPoints: Int = 21,
    freq: Double = 30.0,
    minCutoff: Double = 1.0,
    beta: Double = 0.0,
    dCutoff: Double = 1.0
) {
    private val filters = List(numPoints) {
        OneEuroFilter2D(freq, minCutoff, beta, dCutoff)
    }

    fun filter(landmarks: List<Landmark>): List<Landmark> {
        return landmarks.mapIndexed { i, (u, v, z) ->
            val (fu, fv) = filters[i].filter(u, v)
            Landmark(fu, fv, z)
        }
    }
}
