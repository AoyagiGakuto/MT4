#include <Novice.h>
#include <cmath>
#include <iomanip>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ベクトル
struct Vector3 {
    float x, y, z;
};

// 4x4行列
struct Matrix4x4 {
    float m[4][4];
};

struct Quaternion {
    float x;
    float y;
    float z;
    float w;
};

// 積
Quaternion Multiply(const Quaternion& lhs, const Quaternion& rhs)
{
    Quaternion out;
    float v1x = lhs.x, v1y = lhs.y, v1z = lhs.z, w1 = lhs.w;
    float v2x = rhs.x, v2y = rhs.y, v2z = rhs.z, w2 = rhs.w;

    out.w = w1 * w2 - (v1x * v2x + v1y * v2y + v1z * v2z);

    out.x = w1 * v2x + w2 * v1x + (v1y * v2z - v1z * v2y);
    out.y = w1 * v2y + w2 * v1y + (v1z * v2x - v1x * v2z);
    out.z = w1 * v2z + w2 * v1z + (v1x * v2y - v1y * v2x);

    return out;
}

// 単位
Quaternion IdentityQuaternion()
{
    return Quaternion { 0.0f, 0.0f, 0.0f, 1.0f };
}

Quaternion Conjugate(const Quaternion& q)
{
    return Quaternion { -q.x, -q.y, -q.z, q.w };
}

// 長さ
float Norm(const Quaternion& q)
{
    return std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
}

// 正規化
Quaternion Normalize(const Quaternion& q)
{
    float n = Norm(q);
    if (n == 0.0f) {
        return IdentityQuaternion();
    }
    float inv = 1.0f / n;
    return Quaternion { q.x * inv, q.y * inv, q.z * inv, q.w * inv };
}

Quaternion Inverse(const Quaternion& q)
{
    float norm2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (norm2 == 0.0f) {
        return IdentityQuaternion();
    }
    Quaternion conj = Conjugate(q);
    float inv = 1.0f / norm2;
    return Quaternion { conj.x * inv, conj.y * inv, conj.z * inv, conj.w * inv };
}

// 正規化
Vector3 Normalize(const Vector3& v)
{
    float len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (len == 0.0f)
        return { 0.0f, 0.0f, 0.0f };
    return { v.x / len, v.y / len, v.z / len };
}

static inline Vector3 operator-(const Vector3& v)
{
    return { -v.x, -v.y, -v.z };
}

static inline float Dot(const Vector3& a, const Vector3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline Vector3 Cross(const Vector3& a, const Vector3& b)
{
    return Vector3 {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

static inline Matrix4x4 Identity()
{
    Matrix4x4 I {};
    I.m[0][0] = I.m[1][1] = I.m[2][2] = I.m[3][3] = 1.0f;
    return I;
}

// 任意軸回転行列
Matrix4x4 MakeRotateAxisAngle(const Vector3 axis, float angle)
{
    Vector3 a = Normalize(axis);
    float x = a.x;
    float y = a.y;
    float z = a.z;

    float c = std::cos(angle);
    float s = std::sin(angle);
    float oneMinusC = 1.0f - c;

    Matrix4x4 result {};

    result.m[0][0] = c + oneMinusC * x * x;
    result.m[0][1] = oneMinusC * x * y + s * z;
    result.m[0][2] = oneMinusC * x * z - s * y;
    result.m[0][3] = 0.0f;

    result.m[1][0] = oneMinusC * y * x - s * z;
    result.m[1][1] = c + oneMinusC * y * y;
    result.m[1][2] = oneMinusC * y * z + s * x;
    result.m[1][3] = 0.0f;

    result.m[2][0] = oneMinusC * z * x + s * y;
    result.m[2][1] = oneMinusC * z * y - s * x;
    result.m[2][2] = c + oneMinusC * z * z;
    result.m[2][3] = 0.0f;

    result.m[3][0] = 0.0f;
    result.m[3][1] = 0.0f;
    result.m[3][2] = 0.0f;
    result.m[3][3] = 1.0f;

    return result;
}

// ある方向をある方向へ向ける回転行列
Matrix4x4 DirectionToDirection(const Vector3 fromRaw, const Vector3 toRaw)
{
    const float EPS = 1e-6f;

    Vector3 from = Normalize(fromRaw);
    Vector3 to = Normalize(toRaw);
    if (from.x == 0 && from.y == 0 && from.z == 0)
        return Identity();
    if (to.x == 0 && to.y == 0 && to.z == 0)
        return Identity();

    float d = Dot(from, to);
    d = (d > 1.0f) ? 1.0f : (d < -1.0f ? -1.0f : d);

    if (std::fabs(d - 1.0f) < EPS)
        return Identity();

    if (std::fabs(d + 1.0f) < EPS) {
        Vector3 helper;
        if (std::fabs(from.x) >= std::fabs(from.y) && std::fabs(from.x) >= std::fabs(from.z)) {
            helper = Vector3 { 0, 0, 1 };
        } else {
            helper = Vector3 { 1, 0, 0 };
        }

        Vector3 axis = Normalize(Cross(from, helper));
        return MakeRotateAxisAngle(axis, (float)M_PI);
    }

    Vector3 axis = Cross(from, to);
    float angle = std::acos(d);
    return MakeRotateAxisAngle(axis, angle);
}

void MatrixScreenPrintf(int x, int y, const Matrix4x4& matrix, const char* label)
{
    Novice::ScreenPrintf(x, y, "%s", label);

    for (int i = 0; i < 4; i++) {
        Novice::ScreenPrintf(x, y + 20 + i * 20, "%7.3f %7.3f %7.3f %7.3f",
            matrix.m[i][0], matrix.m[i][1],
            matrix.m[i][2], matrix.m[i][3]);
    }
}

void ScreenPrintfQuaternion(int x, int y, const char* label, const Quaternion& q)
{
    Novice::ScreenPrintf(x, y, "%s", label);
    Novice::ScreenPrintf(x + 260, y, "%7.2f %7.2f %7.2f %7.2f",
        q.x, q.y, q.z, q.w);
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

    Quaternion q1 = { 2.0f, 3.0f, 4.0f, 1.0f };
    Quaternion q2 = { 1.0f, 3.0f, 5.0f, 2.0f };

    Quaternion identity = IdentityQuaternion();
    Quaternion conj = Conjugate(q1);
    Quaternion inv = Inverse(q1);
    Quaternion normal = Normalize(q1);
    Quaternion mul1 = Multiply(q1, q2);
    Quaternion mul2 = Multiply(q2, q1);
    float norm = Norm(q1);

    // int kRowHeight = 20;
    int row = 20;
    int startXLabel = 20;
    int startY = 20;

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

        // 描画
        ScreenPrintfQuaternion(startXLabel, startY + row * 0, "Identity :", identity);
        ScreenPrintfQuaternion(startXLabel, startY + row * 1, "Conjugate:", conj);
        ScreenPrintfQuaternion(startXLabel, startY + row * 2, "Inverse  :", inv);
        ScreenPrintfQuaternion(startXLabel, startY + row * 3, "Normalize:", normal);
        ScreenPrintfQuaternion(startXLabel, startY + row * 4, "Multiply(q1,q2):", mul1);
        ScreenPrintfQuaternion(startXLabel, startY + row * 5, "Multiply(q2,q1):", mul2);

        Novice::ScreenPrintf(startXLabel, startY + row * 7, "Norm(q1) : %7.2f", norm);

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