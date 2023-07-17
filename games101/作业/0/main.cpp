#include<cmath>
#include<eigen3/Eigen/Core>
#include<iostream>

int main() {
    using namespace Eigen;
    using namespace std;

    Vector3f p;
    p << 2.0f, 1.0f, 1.0f;

    // 旋转45度（pi/4）平移（1，2）
    Matrix3f rotate_and_translate;
    float rad = M_PI_4f;
    rotate_and_translate
            <<
            cos(rad), -sin(rad), 1,
            sin(rad), cos(rad), 2,
            0, 0, 1;

    p = rotate_and_translate * p;
    cout << p;
    return 0;
}