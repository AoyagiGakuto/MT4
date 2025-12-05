#include <Novice.h>
#include <cmath>

// ベクトル
struct Vector3 {
    float x, y, z;
};

struct Quaternion {
    float x, y, z, w;
};

// 正規化
Vector3 Normalize(const Vector3& v)
{
    float len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (len == 0.0f)
        return { 0.0f, 0.0f, 0.0f };
    return { v.x / len, v.y / len, v.z / len };
}

// 任意軸回転行列
Quaternion MakeRotateAxisAngleQuaternion(const Vector3& axis, float angle)
{
    Vector3 n = axis;

    float sinHalf = std::sin(angle / 2.0f);
    float cosHalf = std::cos(angle / 2.0f);
    return { n.x * sinHalf, n.y * sinHalf, n.z * sinHalf, cosHalf };
}

Quaternion Slerp(const Quaternion& q0, const Quaternion& q1, float t)
{
    float dot = q0.x * q1.x + q0.y * q1.y + q0.z * q1.z + q0.w * q1.w;

    Quaternion q1_adjusted = q1;

    if (dot < 0.0f) {
        q1_adjusted = { -q1.x, -q1.y, -q1.z, -q1.w };
        dot = -dot;
    }

    float theta = std::acos(dot);
    float sinTheta = std::sin(theta);
    float scale0 = std::sin((1.0f - t) * theta) / sinTheta;
    float scale1 = std::sin(t * theta) / sinTheta;

    if (sinTheta < 0.001f) {
        scale0 = 1.0f - t;
        scale1 = t;
    }
    return {
        scale0 * q0.x + scale1 * q1_adjusted.x,
        scale0 * q0.y + scale1 * q1_adjusted.y,
        scale0 * q0.z + scale1 * q1_adjusted.z,
        scale0 * q0.w + scale1 * q1_adjusted.w
    };
}

// クォータニオンの数値を画面表示する関数
void QuaternionScreenPrintf(int x, int y, const Quaternion& q, const char* label)
{
    Novice::ScreenPrintf(x, y, "%4.2f  %4.2f  %4.2f  %4.2f  : %s",
        q.x, q.y, q.z, q.w, label);
}

const char kWindowTitle[] = "LE2C_01_アオヤギ_ガクト_MT4";

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int)
{

    // ライブラリの初期化
    Novice::Initialize(kWindowTitle, 1280, 720);

    // キー入力結果を受け取る箱
    char keys[256] = { 0 };
    char preKeys[256] = { 0 };

    // 2つの回転クォータニオンを生成
    Quaternion rotation0 = MakeRotateAxisAngleQuaternion({ 0.71f, 0.71f, 0.0f }, 0.3f);
    Quaternion rotation1 = MakeRotateAxisAngleQuaternion({ 0.71f, 0.0f, 0.71f }, 3.141592f);

    // Slerpで補間
    Quaternion interpolate0 = Slerp(rotation0, rotation1, 0.0f);
    Quaternion interpolate1 = Slerp(rotation0, rotation1, 0.3f);
    Quaternion interpolate2 = Slerp(rotation0, rotation1, 0.5f);
    Quaternion interpolate3 = Slerp(rotation0, rotation1, 0.7f);
    Quaternion interpolate4 = Slerp(rotation0, rotation1, 1.0f);

    // ウィンドウの×ボタンが押されるまでループ
    while (Novice::ProcessMessage() == 0) {
        // フレームの開始
        Novice::BeginFrame();

        // キー入力を受け取る
        memcpy(preKeys, keys, 256);
        Novice::GetHitKeyStateAll(keys);

        ///
        /// ↓更新処理ここから
        ///

        ///
        /// ↑更新処理ここまで
        ///

        ///
        /// ↓描画処理ここから
        ///

        int startX = 10;
        int startY = 10;
        int lineHeight = 20;

        QuaternionScreenPrintf(startX, startY + lineHeight * 0, interpolate0, "interpolate0, Slerp(q0, q1, 0.0f)");
        QuaternionScreenPrintf(startX, startY + lineHeight * 1, interpolate1, "interpolate1, Slerp(q0, q1, 0.3f)");
        QuaternionScreenPrintf(startX, startY + lineHeight * 2, interpolate2, "interpolate2, Slerp(q0, q1, 0.5f)");
        QuaternionScreenPrintf(startX, startY + lineHeight * 3, interpolate3, "interpolate3, Slerp(q0, q1, 0.7f)");
        QuaternionScreenPrintf(startX, startY + lineHeight * 4, interpolate4, "interpolate4, Slerp(q0, q1, 1.0f)");

        ///
        /// ↑描画処理ここまで
        ///

        // フレームの終了
        Novice::EndFrame();

        // ESCキーが押されたらループを抜ける
        if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0) {
            break;
        }
    }

    // ライブラリの終了
    Novice::Finalize();
    return 0;
}