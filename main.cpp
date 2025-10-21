#include <Novice.h>
#include <cmath>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

struct Vector3 {
    float x, y, z;
};
struct Matrix4x4 {
    float m[4][4];
};

// ---- math helpers ----
static inline float LengthSq(const Vector3& v) { return v.x * v.x + v.y * v.y + v.z * v.z; }
static inline float Length(const Vector3& v) { return std::sqrt(LengthSq(v)); }

static inline Vector3 Normalize(const Vector3& v)
{
    float len = Length(v);
    if (len <= 1e-8f)
        return { 0, 0, 0 };
    return { v.x / len, v.y / len, v.z / len };
}

static inline float Dot(const Vector3& a, const Vector3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline Vector3 Cross(const Vector3& a, const Vector3& b)
{
    return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}

static inline Matrix4x4 Identity()
{
    Matrix4x4 I {};
    I.m[0][0] = I.m[1][1] = I.m[2][2] = I.m[3][3] = 1.0f;
    return I;
}

// 任意軸回転
static inline Matrix4x4 MakeRotateAxisAngle(Vector3 axis, float angle)
{
    axis = Normalize(axis);
    float x = axis.x, y = axis.y, z = axis.z;
    float c = std::cos(angle), s = std::sin(angle), oneMinusC = 1.0f - c;

    Matrix4x4 r {};
    r.m[0][0] = c + x * x * oneMinusC;
    r.m[0][1] = x * y * oneMinusC + z * s;
    r.m[0][2] = x * z * oneMinusC - y * s;
    r.m[0][3] = 0;
    r.m[1][0] = y * x * oneMinusC - z * s;
    r.m[1][1] = c + y * y * oneMinusC;
    r.m[1][2] = y * z * oneMinusC + x * s;
    r.m[1][3] = 0;
    r.m[2][0] = z * x * oneMinusC + y * s;
    r.m[2][1] = z * y * oneMinusC - x * s;
    r.m[2][2] = c + z * z * oneMinusC;
    r.m[2][3] = 0;
    r.m[3][0] = r.m[3][1] = r.m[3][2] = 0;
    r.m[3][3] = 1;
    return r;
}

static inline Vector3 AnyPerpendicular(Vector3 v)
{
    v = Normalize(v);
    float ax = fabsf(v.x), ay = fabsf(v.y), az = fabsf(v.z);
    if (ax <= ay && ax <= az)
        return Normalize(Vector3 { 0.0f, -v.z, v.y });
    if (ay <= az)
        return Normalize(Vector3 { -v.y, 0.0f, v.x });
    return Normalize(Vector3 { -v.z, v.x, 0.0f });
}

// 回転行列
static inline Matrix4x4 DirectionToDirection(Vector3 from, Vector3 to)
{
    const float EPS = 1e-6f;
    if (Length(from) < EPS || Length(to) < EPS)
        return Identity();
    from = Normalize(from);
    to = Normalize(to);

    float c = Dot(from, to); // cosθ
    c = (c > 1.0f) ? 1.0f : (c < -1.0f ? -1.0f : c);

    if (fabsf(c - 1.0f) < EPS)
        return Identity();

    if (fabsf(c + 1.0f) < EPS) {
        Vector3 axis = AnyPerpendicular(from);
        return MakeRotateAxisAngle(axis, (float)M_PI);
    }

    Vector3 v = Cross(from, to);
    float s2 = LengthSq(v);
    float k = (1.0f - c) / s2;

    float vx = v.x, vy = v.y, vz = v.z;
    float K[3][3] = {
        { 0, -vz, vy },
        { vz, 0, -vx },
        { -vy, vx, 0 }
    };

    Matrix4x4 R = Identity();

    R.m[0][0] += 0;
    R.m[0][1] += vz;
    R.m[0][2] += -vy;
    R.m[1][0] += -vz;
    R.m[1][1] += 0;
    R.m[1][2] += vx;
    R.m[2][0] += vy;
    R.m[2][1] += -vx;
    R.m[2][2] += 0;

    float K2[3][3];

    K2[0][0] = -(vy * vy + vz * vz);
    K2[0][1] = vx * vy;
    K2[0][2] = vx * vz;
    K2[1][0] = vx * vy;
    K2[1][1] = -(vx * vx + vz * vz);
    K2[1][2] = vy * vz;
    K2[2][0] = vx * vz;
    K2[2][1] = vy * vz;
    K2[2][2] = -(vx * vx + vy * vy);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R.m[i][j] += K2[i][j] * k;
        }
    }
    return R;
}

static inline void MatrixScreenPrintf(int x, int y, const Matrix4x4& m, const char* label)
{
    Novice::ScreenPrintf(x, y, "%s", label);
    for (int i = 0; i < 4; i++) {
        Novice::ScreenPrintf(x, y + 20 + i * 20,
            "%7.3f %7.3f %7.3f %7.3f",
            m.m[i][0], m.m[i][1], m.m[i][2], m.m[i][3]);
    }
}

const char kWindowTitle[] = "LE2C_01_アオヤギ_ガクト_MT4";

int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int)
{
    Novice::Initialize(kWindowTitle, 1280, 720);

    char keys[256] {}, preKeys[256] {};
    const int kRowHeight = 20;

    Vector3 from0 = Normalize(Vector3 { 1.0f, 0.7f, 0.5f });
    Vector3 from1 = Normalize(Vector3 { -0.6f, 0.9f, 0.2f });
    Vector3 to0 = Vector3 { -from0.x, -from0.y, -from0.z };
    Vector3 to1 = Normalize(Vector3 { 0.4f, 0.7f, -0.5f });

    Matrix4x4 rotateMatrix0 = DirectionToDirection(Normalize(Vector3 { 1.0f, 0.0f, 0.0f }), Normalize(Vector3 { -1.0f, 0.0f, 0.0f }));
    Matrix4x4 rotateMatrix1 = DirectionToDirection(from0, to0);
    Matrix4x4 rotateMatrix2 = DirectionToDirection(from1, to1);

    while (Novice::ProcessMessage() == 0) {
        Novice::BeginFrame();
        std::memcpy(preKeys, keys, 256);
        Novice::GetHitKeyStateAll(keys);

        MatrixScreenPrintf(0, 0, rotateMatrix0, "rotateMatrix0");
        MatrixScreenPrintf(0, kRowHeight * 5, rotateMatrix1, "rotateMatrix1");
        MatrixScreenPrintf(0, kRowHeight * 10, rotateMatrix2, "rotateMatrix2");

        Novice::EndFrame();
        if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0)
            break;
    }
    Novice::Finalize();
    return 0;
}
