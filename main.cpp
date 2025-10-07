#include <Novice.h>
#include <cmath>

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

// 任意軸回転行列作成
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

void MatrixScreenPrintf(int x, int y, const Matrix4x4& matrix,const char*label)
{
    Novice::ScreenPrintf(x, y, "%s",label);

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

     Vector3 axis = Normalize({ 1.0f, 1.0f, 1.0f });
    float angle = 0.44f;
    Matrix4x4 rotateMatrix = MakeRotateAxisAngle(axis, angle);

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
        
        MatrixScreenPrintf(0, 0, rotateMatrix, "rotateMatrix");

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