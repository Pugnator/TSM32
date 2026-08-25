#include "../Core/Src/ahrs/inc/axis_remap.h"
#include "../Core/Src/ahrs/inc/fusion_math.h"

#include <cmath>
#include <cstdio>

namespace
{
int failures = 0;

#define CHECK(condition)                                                       \
    do {                                                                       \
        if (!(condition)) {                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
            ++failures;                                                        \
        }                                                                      \
    } while (0)

bool near(float a, float b)
{
    return std::fabs(a - b) < 1e-5f;
}
}

int main()
{
    const VectorFloat body = Ahrs::sensorToBodyZ180(1.0f, 2.0f, 3.0f, 0.5f);
    CHECK(near(body.x, -0.5f));
    CHECK(near(body.y, -1.0f));
    CHECK(near(body.z, 1.5f));

    float s0 = 0.0f;
    float s1 = 0.0f;
    float s2 = 0.0f;
    float s3 = 0.0f;
    CHECK(!Ahrs::detail::normalizeGradient(s0, s1, s2, s3));
    CHECK(std::isfinite(s0) && std::isfinite(s1) &&
          std::isfinite(s2) && std::isfinite(s3));

    s0 = 3.0f;
    s1 = 4.0f;
    CHECK(Ahrs::detail::normalizeGradient(s0, s1, s2, s3));
    CHECK(near(s0, 0.6f));
    CHECK(near(s1, 0.8f));

    std::printf("AHRS math tests: %s\n", failures ? "FAIL" : "PASS");
    return failures ? 1 : 0;
}
