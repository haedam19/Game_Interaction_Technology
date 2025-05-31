// native-lib.cpp
#include <jni.h>
#include <android/bitmap.h>
#include <android/log.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <fstream>

#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, "native-lib", __VA_ARGS__)

// 전역 변수
std::string calibration_cache_dir;
std::string calibration_cache_hand_dir;
int saved_image_count = 0;
cv::aruco::Dictionary dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
cv::aruco::DetectorParameters params;
cv::aruco::ArucoDetector detector(dict, params);
cv::Mat g_K, g_dist, g_Rwc, g_twc, g_Kinv;
bool g_pose_valid = false;
float g_landmarks[21][3];
cv::Point3f g_landmarks_world[21];
std::map<std::string, float> g_landmark_len;
std::map<std::string, float> landmark_len_sum;
std::map<std::string, int> landmark_len_count;

// Calibration 경로 초기화
extern "C"
JNIEXPORT void JNICALL
Java_com_example_virtualtouchpad_NativeLib_initCalibrationCache(JNIEnv *env, jobject, jstring path) {
    const char* pathStr = env->GetStringUTFChars(path, nullptr);
    calibration_cache_dir = std::string(pathStr);
    calibration_cache_hand_dir = calibration_cache_dir + "/hand";
    saved_image_count = 0;
    env->ReleaseStringUTFChars(path, pathStr);
    LOGI("Calibration cache path set: %s", calibration_cache_dir.c_str());
}

// 마커 검출 여부 판단
extern "C"
JNIEXPORT jboolean JNICALL
Java_com_example_virtualtouchpad_NativeLib_detectArucoMarkers(JNIEnv *env, jobject, jobject bitmap) {
    AndroidBitmapInfo info;
    void* pixels;
    if (AndroidBitmap_getInfo(env, bitmap, &info) < 0 || AndroidBitmap_lockPixels(env, bitmap, &pixels) < 0) {
        LOGI("Failed to get bitmap info or lock pixels.");
        return JNI_FALSE;
    }
    cv::Mat rgba(info.height, info.width, CV_8UC4, pixels);
    cv::Mat gray;
    cv::cvtColor(rgba, gray, cv::COLOR_RGBA2GRAY);
    AndroidBitmap_unlockPixels(env, bitmap);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    detector.detectMarkers(gray, corners, ids);

    return ids.empty() ? JNI_FALSE : JNI_TRUE;
}

// 현재 화면에서 캘리브레이션 이미지 저장
extern "C"
JNIEXPORT jboolean JNICALL
Java_com_example_virtualtouchpad_NativeLib_saveCalibrationImage(JNIEnv *env, jobject, jobject bitmap) {
    AndroidBitmapInfo info;
    void* pixels;
    if (AndroidBitmap_getInfo(env, bitmap, &info) < 0 || AndroidBitmap_lockPixels(env, bitmap, &pixels) < 0) {
        LOGI("Failed to get bitmap info or lock pixels.");
        return JNI_FALSE;
    }
    cv::Mat rgba(info.height, info.width, CV_8UC4, pixels);
    cv::Mat bgr;
    cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);
    AndroidBitmap_unlockPixels(env, bitmap);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    detector.detectMarkers(bgr, corners, ids);

    if (ids.empty()) return JNI_FALSE;

    char filename[512];
    snprintf(filename, sizeof(filename), "%s/img_%03d.png", calibration_cache_dir.c_str(), saved_image_count++);
    bool success = cv::imwrite(filename, bgr);
    LOGI("Image saved: %s", filename);
    return static_cast<jboolean>(success ? 1 : 0);
}

// 저장된 이미지로 카메라 캘리브레이션 수행
extern "C"
JNIEXPORT jboolean JNICALL
Java_com_example_virtualtouchpad_NativeLib_calibrateFromSavedImages(JNIEnv *, jobject) {
    // 이미지 로드 및 마커 검출
    std::vector<std::vector<cv::Point3f>> objPoints;
    std::vector<std::vector<cv::Point2f>> imgPoints;
    std::vector<cv::String> files;
    cv::glob(calibration_cache_dir + "/*.png", files);
    if (files.size() < 5) {
        LOGI("Not enough images for calibration: %zu", files.size());
        return JNI_FALSE;
    }

    // 기준 마커 3D 좌표
    std::vector<cv::Point3f> marker_obj = {
            {-0.025f,  0.025f, 0},
            { 0.025f,  0.025f, 0},
            { 0.025f, -0.025f, 0},
            {-0.025f, -0.025f, 0},
    };

    cv::Mat img, gray;

    // 각 이미지에서 마커 검출 및 저장
    for (const auto& fname : files) {
        img = cv::imread(fname);
        if (img.empty()) continue;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        // 마커가 존재할 때만 저장
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        detector.detectMarkers(gray, corners, ids);
        for (size_t i = 0; i < ids.size(); ++i) {
            if (ids[i] == 0) {
                objPoints.push_back(marker_obj);
                imgPoints.push_back(corners[i]);
            }
        }
    }
    if (objPoints.size() < 5) {
        LOGI("Insufficient data for calibration");
        return JNI_FALSE;
    }

    // 내부 파라미터 추정
    cv::calibrateCamera(objPoints, imgPoints, img.size(), g_K, g_dist, cv::noArray(), cv::noArray());
    g_Kinv = g_K.inv();

    // 결과 저장
    std::string outputFile = calibration_cache_dir + "/camera_params.yml";
    cv::FileStorage fs(outputFile, cv::FileStorage::WRITE);
    fs << "K" << g_K;
    fs << "dist" << g_dist;
    fs.release();
    LOGI("Calibration complete. Saved to: %s", outputFile.c_str());
    return JNI_TRUE;
}

// 카메라 파라미터 로드
extern "C"
JNIEXPORT jboolean JNICALL
Java_com_example_virtualtouchpad_NativeLib_loadCalibrationParams(JNIEnv *, jobject) {
    std::string file = calibration_cache_dir + "/camera_params.yml";
    cv::FileStorage fs(file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        LOGI("Failed to open camera_params.yml");
        return JNI_FALSE;
    }
    fs["K"] >> g_K;
    fs["dist"] >> g_dist;
    g_Kinv = g_K.inv();
    fs.release();
    LOGI("Loaded calibration params.");
    return JNI_TRUE;
}

// 마커 기반 카메라 위치 방향 추정
extern "C"
JNIEXPORT jfloatArray JNICALL
Java_com_example_virtualtouchpad_NativeLib_estimatePose(JNIEnv *env, jobject, jobject bitmap) {
    if (g_K.empty() || g_dist.empty()) {
        LOGI("Calibration not loaded.");
        return nullptr;
    }

    // 이미지 처리
    AndroidBitmapInfo info;
    void* pixels;
    if (AndroidBitmap_getInfo(env, bitmap, &info) < 0 || AndroidBitmap_lockPixels(env, bitmap, &pixels) < 0) {
        return nullptr;
    }
    cv::Mat rgba(info.height, info.width, CV_8UC4, pixels);
    cv::Mat gray;
    cv::cvtColor(rgba, gray, cv::COLOR_RGBA2GRAY);
    AndroidBitmap_unlockPixels(env, bitmap);

    // 마커 검출
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    detector.detectMarkers(gray, corners, ids);

    // id==0 마커 기준으로 포즈 추정
    for (size_t i = 0; i < ids.size(); ++i) {
        if (ids[i] == 0) {
            std::vector<cv::Point3f> objPoints = {
                    {-0.025f,  0.025f, 0},
                    { 0.025f,  0.025f, 0},
                    { 0.025f, -0.025f, 0},
                    {-0.025f, -0.025f, 0}
            };
            cv::Mat rvec, tvec;
            bool success = cv::solvePnP(objPoints, corners[i], g_K, g_dist, rvec, tvec);
            if (success) {
                cv::Mat R;
                cv::Rodrigues(rvec, R);
                g_Rwc = R.t();           // 카메라→월드
                g_twc = -g_Rwc * tvec;  // 카메라 위치
                g_pose_valid = true;

                cv::Mat bgr;
                if (rgba.channels() == 4) {
                    cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);
                    cv::drawFrameAxes(bgr, g_K, g_dist, rvec, tvec, 0.03);
                    cv::cvtColor(bgr, rgba, cv::COLOR_BGR2RGBA);
                }

                // 카메라가 바라보는 방향 계산
                cv::Mat viewMat = g_Rwc * (cv::Mat_<double>(3, 1) << 0, 0, -1);
                cv::Vec3d view(
                        viewMat.at<double>(0),
                        viewMat.at<double>(1),
                        viewMat.at<double>(2)
                );

                // 결과 반환
                jfloatArray result = env->NewFloatArray(15);
                float data[] = {
                        (float)g_twc.at<double>(0), (float)g_twc.at<double>(1), (float)g_twc.at<double>(2),
                        (float)view[0], (float)view[1], (float)view[2],
                        (float)g_Rwc.at<double>(0,0), (float)g_Rwc.at<double>(0,1), (float)g_Rwc.at<double>(0,2),
                        (float)g_Rwc.at<double>(1,0), (float)g_Rwc.at<double>(1,1), (float)g_Rwc.at<double>(1,2),
                        (float)g_Rwc.at<double>(2,0), (float)g_Rwc.at<double>(2,1), (float)g_Rwc.at<double>(2,2)
                };
                env->SetFloatArrayRegion(result, 0, 15, data);
                return result;
            }
        }
    }
    return nullptr;
}

// 2D 이미지 좌표를 3D 월드 공간의 방향 벡터로 변환
extern "C"
JNIEXPORT jfloatArray JNICALL
Java_com_example_virtualtouchpad_NativeLib_uvToWorldDir(JNIEnv *env, jobject, jfloat u, jfloat v, jint imgWidth, jint imgHeight, jboolean isNormalized) {
    if (!g_pose_valid || g_Kinv.empty()) {
        return nullptr;
    }

    // 정규화된 좌표라면 픽셀 단위로 변환
    if (isNormalized) {
        u *= imgWidth;
        v *= imgHeight;
    }

    // 이미지 좌표 → 카메라 좌표계 → 월드 좌표계 방향 벡터
    cv::Mat uv_h = (cv::Mat_<double>(3, 1) << u, v, 1.0);
    cv::Mat ray_cam = g_Kinv * uv_h;
    ray_cam /= cv::norm(ray_cam);
    cv::Mat ray_world = g_Rwc * ray_cam;

    jfloatArray result = env->NewFloatArray(3);
    float data[] = {
            (float)ray_world.at<double>(0),
            (float)ray_world.at<double>(1),
            (float)ray_world.at<double>(2)
    };
    env->SetFloatArrayRegion(result, 0, 3, data);
    return result;
}

// 손 2D 좌표 업데이트
extern "C"
JNIEXPORT void JNICALL
Java_com_example_virtualtouchpad_NativeLib_updateLandmarks(JNIEnv *env, jobject, jfloatArray arr) {
    if (env->GetArrayLength(arr) < 63) return;
    jfloat* data = env->GetFloatArrayElements(arr, nullptr);
    for (int i = 0; i < 21; ++i) {
        g_landmarks[i][0] = data[i * 3];
        g_landmarks[i][1] = data[i * 3 + 1];
        g_landmarks[i][2] = data[i * 3 + 2];
    }
    env->ReleaseFloatArrayElements(arr, data, JNI_ABORT);
}

// 카메라 위치에서 방향 벡터와 z 평면의 교점 계산
cv::Point3f intersectRayWithPlane(const cv::Point3f& cam_pos, const cv::Point3f& dir, float z_plane = 0.003f) {
    float t = (z_plane - cam_pos.z) / dir.z;
    return cam_pos + dir * t;
}

// 랜드마크 좌표 파일에서 g_landmarks 세팅
bool loadLandmarksFromFile(const std::string& filename,
                           cv::Point3f& cam_pos,
                           cv::Mat& rotation,
                           float landmarks[21][2]) {
    std::ifstream in(filename);
    if (!in.is_open()) return false;

    float camX, camY, camZ;
    char comma;
    if (!(in >> camX >> comma >> camY >> comma >> camZ)) return false;
    cam_pos = cv::Point3f(camX, camY, camZ);

    rotation = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i) {
        double val;
        if (!(in >> val)) return false;
        if (i < 8) in >> comma;
        rotation.at<double>(i / 3, i % 3) = val;
    }

    for (int i = 0; i < 21; ++i) {
        float u, v;
        if (!(in >> u >> comma >> v)) return false;
        landmarks[i][0] = u;
        landmarks[i][1] = v;
    }
    return true;
}


// 손 캘리브레이션
extern "C"
JNIEXPORT jboolean JNICALL
Java_com_example_virtualtouchpad_NativeLib_calibrateHandFromLandmarkFiles(JNIEnv *, jobject) {
    std::vector<cv::String> files;
    cv::glob(calibration_cache_hand_dir + "/*.txt", files);

    landmark_len_sum.clear();
    landmark_len_count.clear();

    int validCount = 0;
    for (const auto& lmkPath : files) {
        cv::Point3f cam_pos;
        cv::Mat rotation;
        float landmarks[21][2];
        if (!loadLandmarksFromFile(lmkPath, cam_pos, rotation, landmarks)) {
            LOGI("Failed to load landmark file: %s", lmkPath.c_str());
            continue;
        }

        cv::Mat Kinv = g_Kinv.clone();
        cv::Mat Rwc = rotation;
        cv::Point3f twc = cam_pos;

        // 위치 추정
        for (int i = 0; i < 21; ++i) {
            float u = landmarks[i][0];
            float v = landmarks[i][1];
            cv::Mat uv = (cv::Mat_<double>(3, 1) << u, v, 1.0);
            cv::Mat ray_cam = Kinv * uv;
            ray_cam /= cv::norm(ray_cam);
            cv::Mat ray_world = Rwc * ray_cam;
            cv::Point3f dir(ray_world.at<double>(0), ray_world.at<double>(1), ray_world.at<double>(2));
            g_landmarks_world[i] = intersectRayWithPlane(twc, dir);
        }

        auto calcDist = [](const cv::Point3f& a, const cv::Point3f& b) {
            return cv::norm(a - b);
        };
        auto saveLen = [&](int a, int b) {
            char key[8];
            snprintf(key, sizeof(key), "%d-%d", a, b);
            float len = calcDist(g_landmarks_world[a], g_landmarks_world[b]);
            if (landmark_len_count[key] == 0) {
                landmark_len_sum[key] = len;
            } else {
                landmark_len_sum[key] = (landmark_len_sum[key] + len) / 2.0f;
            }
            landmark_len_count[key] += 1;
        };
        saveLen(0, 1); saveLen(1, 2); saveLen(2, 3); saveLen(3, 4);
        saveLen(0, 5); saveLen(0, 9); saveLen(0, 17);
        saveLen(5, 9); saveLen(9, 13); saveLen(13, 17);
        saveLen(5, 6); saveLen(6, 7); saveLen(7, 8);
        saveLen(9, 10); saveLen(10, 11); saveLen(11, 12);
        saveLen(13, 14); saveLen(14, 15); saveLen(15, 16);
        saveLen(17, 18); saveLen(18, 19); saveLen(19, 20);

        ++validCount;
    }

    if (validCount == 0) {
        LOGI("No valid landmark file processed");
        return JNI_FALSE;
    }

    for (const auto& p : landmark_len_sum) {
        g_landmark_len[p.first] = p.second;
    }

    LOGI("Hand calibration from %d landmark files complete.", validCount);
    return JNI_TRUE;
}

std::vector<cv::Point3f> findPointsOnRay(
        const cv::Point3f& p0,
        const cv::Point3f& ray_origin,
        const cv::Point3f& ray_dir,
        float distance
) {
    cv::Point3f v = p0 - ray_origin;
    float b = -2.0f * ray_dir.dot(v);
    float c = v.dot(v) - distance * distance;
    float disc = b * b - 4 * c;
    std::vector<cv::Point3f> results;
    if (disc < 0) return results;
    float sqrt_disc = std::sqrt(disc);
    float t1 = (-b + sqrt_disc) / 2.0f;
    float t2 = (-b - sqrt_disc) / 2.0f;
    results.push_back(ray_origin + ray_dir * t1);
    if (disc > 0.0001f)
        results.push_back(ray_origin + ray_dir * t2);
    return results;
}

// 삼각 측량으로 손바닥의 깊이를 추정
extern "C"
JNIEXPORT jboolean JNICALL
Java_com_example_virtualtouchpad_NativeLib_estimateDepth(JNIEnv *, jobject) {
    if (!g_pose_valid) return JNI_FALSE;

    // 기준
    int baseA = 0, baseB = 5, baseC = 9;
    float lenAB = g_landmark_len["0-5"];
    float lenAC = g_landmark_len["0-9"];
    float lenBC = g_landmark_len["5-9"];

    // ray 계산 함수 (2D → 3D 방향 벡터)
    auto getRay = [](int idx) -> cv::Point3f {
        float u = g_landmarks[idx][0];
        float v = g_landmarks[idx][1];
        cv::Mat uv = (cv::Mat_<double>(3, 1) << u, v, 1.0);
        cv::Mat ray_cam = g_Kinv * uv;
        ray_cam /= cv::norm(ray_cam);
        cv::Mat ray_world = g_Rwc * ray_cam;
        return cv::Point3f(ray_world);
    };

    cv::Point3f ray0 = getRay(baseA);
    cv::Point3f ray5 = getRay(baseB);
    cv::Point3f ray9 = getRay(baseC);
    cv::Point3f cam = cv::Point3f(g_twc);

    float z_min = 0.01f, z_max = 0.8f;
    int samples = 50, best_idx = -1;
    float best_error = FLT_MAX;
    cv::Point3f best0, best5, best9;

    // 깊이 z값을 바꿔서 오차 최소화
    for (int it = 0; it < 3; ++it) {
        float dz = (z_max - z_min) / (samples - 1);
        for (int i = 0; i < samples; ++i) {
            float z = z_min + dz * i;
            cv::Point3f p0 = cam + ray0 * z;
            auto p5s = findPointsOnRay(p0, cam, ray5, lenAB);
            auto p9s = findPointsOnRay(p0, cam, ray9, lenAC);
            if (p5s.empty() || p9s.empty()) continue;

            // 세 점을 삼각형으로 했을 때 AB, AC, BC 거리를 만족하는지 확인
            for (auto& p5 : p5s) {
                for (auto& p9 : p9s) {
                    float d = cv::norm(p5 - p9);
                    float err = std::abs(d - lenBC);
                    if (err < best_error) {
                        best_error = err;
                        best_idx = i;
                        best0 = p0;
                        best5 = p5;
                        best9 = p9;
                    }
                }
            }
        }

        // 검색 범위 좁힘
        if (best_idx != -1) {
            int li = std::max(0, best_idx - 2);
            int hi = std::min(samples - 1, best_idx + 2);
            z_min += dz * li;
            z_max = z_min + dz * (hi - li);
        }
    }

    // 오차가 작으면 성공
    if (best_error < 0.001f) {
        g_landmarks_world[0] = best0;
        g_landmarks_world[5] = best5;
        g_landmarks_world[9] = best9;
        LOGI("Depth estimate success: error=%.6f", best_error);
        return JNI_TRUE;
    }

    LOGI("Depth estimate failed: error=%.6f", best_error);
    return JNI_FALSE;
}

std::vector<cv::Point3f> intersectRaySphere(
        const cv::Point3f& center,
        float radius,
        const cv::Point3f& ray_origin,
        const cv::Point3f& ray_dir
) {
    cv::Point3f oc = ray_origin - center;
    float b = 2.0f * oc.dot(ray_dir);
    float c = oc.dot(oc) - radius * radius;
    float disc = b * b - 4 * c;
    std::vector<cv::Point3f> results;
    if (disc < 0) return results;
    float sqrt_disc = std::sqrt(disc);
    float t1 = (-b - sqrt_disc) / 2.0f;
    float t2 = (-b + sqrt_disc) / 2.0f;
    results.push_back(ray_origin + ray_dir * t1);
    if (disc > 1e-6f) // prevent duplicate
        results.push_back(ray_origin + ray_dir * t2);
    return results;
}

// 기준점을 중심으로 위치를 추정
bool estimateLandmarkFromBase(int baseIdx, int targetIdx) {
    if (!g_pose_valid) return false;

    float u = g_landmarks[targetIdx][0];
    float v = g_landmarks[targetIdx][1];
    float z_target = g_landmarks[targetIdx][2];
    float z_base   = g_landmarks[baseIdx][2];

    cv::Mat uv = (cv::Mat_<double>(3, 1) << u, v, 1.0);
    cv::Mat ray_cam = g_Kinv * uv;
    ray_cam /= cv::norm(ray_cam);
    cv::Mat ray_world = g_Rwc * ray_cam;
    cv::Point3f ray(ray_world);

    cv::Point3f origin(g_twc);
    cv::Point3f center = g_landmarks_world[baseIdx];

    char key[8];
    snprintf(key, sizeof(key), "%d-%d",
             std::min(baseIdx, targetIdx),
             std::max(baseIdx, targetIdx));
    float radius = g_landmark_len[key];

    auto candidates = intersectRaySphere(center, radius, origin, ray);
    if (candidates.empty()) return false;

    if (candidates.size() == 1) {
        g_landmarks_world[targetIdx] = candidates[0];
    } else {
        if (z_target > z_base) {
            g_landmarks_world[targetIdx] = candidates[1];
        } else {
            g_landmarks_world[targetIdx] = candidates[0];
        }
    }

    return true;
}

// 손바닥 추정
extern "C"
JNIEXPORT jboolean JNICALL
Java_com_example_virtualtouchpad_NativeLib_estimateIndexTip(JNIEnv *, jobject) {
    try {
        if (!estimateLandmarkFromBase(5, 6)) return JNI_FALSE;
        if (!estimateLandmarkFromBase(6, 7)) return JNI_FALSE;
        if (!estimateLandmarkFromBase(7, 8)) return JNI_FALSE;
        LOGI("Index tip estimation succeeded.");
        return JNI_TRUE;
    } catch (...) {
        LOGI("Index tip estimation failed.");
        return JNI_FALSE;
    }
}

// 지정된 인덱스의 손 3D 위치를 반환
extern "C"
JNIEXPORT jfloatArray JNICALL
Java_com_example_virtualtouchpad_NativeLib_getLandmarkWorld(JNIEnv *env, jobject, jint idx) {
    if (idx < 0 || idx >= 21) return nullptr;
    jfloatArray result = env->NewFloatArray(3);
    float data[3] = {
            g_landmarks_world[idx].x,
            g_landmarks_world[idx].y,
            g_landmarks_world[idx].z
    };
    env->SetFloatArrayRegion(result, 0, 3, data);
    return result;
}
