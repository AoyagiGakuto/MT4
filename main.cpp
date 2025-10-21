#include <Novice.h>
#include <cmath>

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

// 回転行列
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

const char kWindowTitle[] = "LE2C_01_アオヤギ_ガクト_MT4";

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int)
{

    // ライブラリの初期化
    Novice::Initialize(kWindowTitle, 1280, 720);

    // キー入力結果を受け取る箱
    char keys[256] = { 0 };
    char preKeys[256] = { 0 };

    Vector3 from0 = Normalize(Vector3 { 1.0f, 0.7f, 0.5f });
    Vector3 to0 = -from0;
    Vector3 from1 = Normalize(Vector3 { -0.6f, 0.9f, 0.2f });
    Vector3 to1 = Normalize(Vector3 { 0.4f, 0.7f, -0.5f });
    Matrix4x4 rotateMatrix0 = DirectionToDirection(
        Normalize(Vector3 { 1.0f, 0.0f, 0.0f }), Normalize(Vector3 { -1.0f, 0.0f, 0.0f }));
    Matrix4x4 rotateMatrix1 = DirectionToDirection(from0, to0);
    Matrix4x4 rotateMatrix2 = DirectionToDirection(from1, to1);
    int kRowHeight = 20;

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

        MatrixScreenPrintf(0, 0, rotateMatrix0, "rotateMatrix0");
        MatrixScreenPrintf(0, kRowHeight * 5, rotateMatrix1, "rotateMatrix1");
        MatrixScreenPrintf(0, kRowHeight * 10, rotateMatrix2, "rotateMatrix2");
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