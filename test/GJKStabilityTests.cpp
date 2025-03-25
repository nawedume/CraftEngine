#include <catch2/catch_test_macros.hpp>
#include <random>

#include <glm/glm.hpp>
#include <utility>

template <typename T> struct Vec3 {
    T x, y, z;

    // Addition operator
    Vec3<T> operator+(const Vec3<T> &other) const {
        return {x + other.x, y + other.y, z + other.z};
    }

    // Subtraction operator
    Vec3<T> operator-(const Vec3<T> &other) const {
        return {x - other.x, y - other.y, z - other.z};
    }

    // Negation operator
    Vec3<T> operator-() const { return {-x, -y, -z}; }

    Vec3<T> operator*(T s) const { return {s * x, s * y, s * z}; }

    // Dot product function
    T dot(const Vec3<T> &other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    // Cross product function
    Vec3<T> cross(const Vec3<T> &other) const {
        return {y * other.z - z * other.y, z * other.x - x * other.z,
                x * other.y - y * other.x};
    }

    // Magnitude (length) of the vector
    T magnitude() const { return std::sqrt(x * x + y * y + z * z); }

    // Normalize the vector (make it unit length)
    Vec3<T> normalize() const {
        T mag = magnitude();
        return {x / mag, y / mag, z / mag};
    }

    std::string toString() const {
        std::ostringstream oss;
        oss << "(" << x << ", " << y << ", " << z << ")";
        return oss.str();
    }
};

template <typename T> struct Line {
    Vec3<T> PointA;
    Vec3<T> PointB;

    std::string toString() const {
        std::ostringstream oss;
        oss << PointA.toString() << "->" << PointB.toString();
        return oss.str();
    }
};

// Helper function to convert glm::vec3 to string
std::string vec3ToString(const glm::vec3 &v) {
    std::ostringstream oss;
    oss << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    return oss.str();
}

template <typename T> Vec3<T> DirToOriginCrossProductMethod(Line<T> line) {
    Vec3<T> ab = line.PointB - line.PointA;
    Vec3<T> a0 = -line.PointA;
    return ab.cross(a0).cross(ab);
}

template <typename T> Vec3<T> DirToOriginClosestPointMethod(Line<T> line) {
    Vec3<T> ab = line.PointB - line.PointA;

    T abSqMag = ab.dot(ab);

    // projection onto the line, not normalized ab
    T t = (-line.PointA).dot(ab);

    t = t / abSqMag;
    return line.PointA + (ab * t);
}

// Function to generate a random number of arbitrary precision
template <typename T>
T GenerateRandomNumber(T lower_bound = static_cast<T>(-100.0),
                       T upper_bound = static_cast<T>(100.0), int seed = 0) {
    static std::mt19937 generator(seed); // Mersenne Twister engine

    std::uniform_real_distribution<T> distribution(lower_bound, upper_bound);
    return distribution(generator);
}

// Function to generate a random Vec3
template <typename T>
Vec3<T> GenerateVec3(T lower_bound = static_cast<T>(-100.0),
                     T upper_bound = static_cast<T>(100.0)) {
    return {GenerateRandomNumber<T>(lower_bound, upper_bound),
            GenerateRandomNumber<T>(lower_bound, upper_bound),
            GenerateRandomNumber<T>(lower_bound, upper_bound)};
}

// Function to generate random lines
template <typename T>
void GenerateRandomLines(int numLines, Line<T> *lines,
                         T lower_bound = static_cast<T>(-100.0),
                         T upper_bound = static_cast<T>(100.0)) {
    for (int i = 0; i < numLines; ++i) {
        Vec3<T> p1 = GenerateVec3<T>(lower_bound, upper_bound);
        Vec3<T> p2 = GenerateVec3<T>(lower_bound, upper_bound);
        lines[i] = Line<T>(p1, p2);
    }
}

template <typename T>
void assertEquals(Vec3<T> a, Vec3<T> b, double precision = 1e-12) {
    REQUIRE((a - b).magnitude() < precision);
}

int GetSupportPoint(Vec3<double> *points, Vec3<double> direction,
                    int numPoints) {
    float maxVal = 0.0f;
    int pointIdx = -1;
    for (int i = 0; i < numPoints; ++i) {
        float val = points[i].dot(direction);
        if (val > maxVal) {
            maxVal = val;
            pointIdx = i;
        }
    }

    return pointIdx;
}

Vec3<double> toDouble(Vec3<float> v) { return Vec3<double>(v.x, v.y, v.z); }

Line<double> toDouble(Line<float> line) {
    return {.PointA = toDouble(line.PointA), .PointB = toDouble(line.PointB)};
}

TEST_CASE("Equality of methods", "[gjk]") {
    const int NUM_OF_LINES = 100000;

    Line<float> lines[NUM_OF_LINES];
    GenerateRandomLines(NUM_OF_LINES, lines);

    Vec3<double> doubleCrossProductMethod[NUM_OF_LINES];
    Vec3<double> doubleClosestPointMethod[NUM_OF_LINES];

    // Equality of the two methods
    SECTION("Precision Tests") {

        SECTION("Double equality tests, 1e-12") {
            for (int i = 0; i < NUM_OF_LINES; ++i) {
                Line<float> l = lines[i];
                Line<double> doubleLine = {
                    .PointA = Vec3<double>(l.PointA.x, l.PointA.y, l.PointA.z),
                    .PointB = Vec3<double>(l.PointB.x, l.PointB.z, l.PointB.z)};
                Vec3<double> v1 =
                    DirToOriginCrossProductMethod<double>(doubleLine);
                Vec3<double> v2 =
                    -DirToOriginClosestPointMethod<double>(doubleLine);
                assertEquals(v1.normalize(), v2.normalize());
            }
        }

        SECTION("Float equality tests") {
            for (int i = 0; i < NUM_OF_LINES; ++i) {
                Line<float> floatLine = lines[i];
                Vec3<float> v1 =
                    DirToOriginCrossProductMethod<float>(floatLine);
                Vec3<float> v2 =
                    -DirToOriginClosestPointMethod<float>(floatLine);
                assertEquals(v1.normalize(), v2.normalize(),
                             1e-4); // fails for 1e-5 in some cases
            }
        }

        SECTION("Float method closes to truth") {
            double maxDifferenceCrossProductMethod = 0.0f;
            double maxNormalizeDifferenceCrossProductMethod = 0.0f;
            double maxDifferenceClosestPointMethod = 0.0f;
            double maxNormalizeDifferenceClosestPointMethod = 0.0f;

            int maxNormalizedCrossMethodIdx = 0;
            int maxNormalizedClosestMethodIdx = 0;

            for (int i = 0; i < NUM_OF_LINES; ++i) {
                Line<float> l = lines[i];

                Line<double> doubleLine = {
                    .PointA = Vec3<double>(l.PointA.x, l.PointA.y, l.PointA.z),
                    .PointB = Vec3<double>(l.PointB.x, l.PointB.y, l.PointB.z)};
                Line<float> floatLine = l;

                Vec3<double> expectedCross =
                    DirToOriginCrossProductMethod<double>(doubleLine);
                Vec3<double> expectedClosest =
                    -DirToOriginClosestPointMethod<double>(doubleLine);

                Vec3<double> normalizeExpectedCross = expectedCross.normalize();
                Vec3<double> normalizeExpectedClosest =
                    expectedClosest.normalize();

                Vec3<float> v1 =
                    DirToOriginCrossProductMethod<float>(floatLine);
                Vec3<float> v2 =
                    -DirToOriginClosestPointMethod<float>(floatLine);

                Vec3<double> dv1 = Vec3<double>(v1.x, v1.y, v1.z);
                Vec3<double> dv2 = Vec3<double>(v2.x, v2.y, v2.z);

                // using the cross product method as the source of truth
                double diffCrossMethod = (expectedCross - dv1).magnitude();
                double diffClosestMethod = (expectedClosest - dv2).magnitude();

                double diffNormalizeCrossMethod =
                    (normalizeExpectedCross - dv1.normalize()).magnitude();
                double diffNormalizeClosestMethod =
                    (normalizeExpectedClosest - dv2.normalize()).magnitude();

                if (maxDifferenceCrossProductMethod < diffCrossMethod) {
                    maxDifferenceCrossProductMethod = diffCrossMethod;
                }

                if (maxDifferenceClosestPointMethod < diffClosestMethod) {
                    maxDifferenceClosestPointMethod = diffClosestMethod;
                }

                if (maxNormalizeDifferenceCrossProductMethod <
                    diffNormalizeCrossMethod) {
                    maxNormalizeDifferenceCrossProductMethod =
                        diffNormalizeCrossMethod;
                    maxNormalizedCrossMethodIdx = i;
                }

                if (maxNormalizeDifferenceClosestPointMethod <
                    diffNormalizeClosestMethod) {
                    maxNormalizeDifferenceClosestPointMethod =
                        diffNormalizeClosestMethod;
                    maxNormalizedClosestMethodIdx = i;
                }
            }

            printf("Max difference cross product method: %lf\n",
                   maxDifferenceCrossProductMethod);
            printf("Max difference closest point method: %lf\n\n",
                   maxDifferenceClosestPointMethod);
            printf(
                "Max difference normalized cross product method: %lf; %d; "
                "%s:%s\n",
                maxNormalizeDifferenceCrossProductMethod,
                maxNormalizedCrossMethodIdx,
                lines[maxNormalizedCrossMethodIdx].PointA.toString().c_str(),
                lines[maxNormalizedCrossMethodIdx].PointB.toString().c_str());
            printf(
                "Max difference normalized closest point method: %lf; %d; "
                "%s:%s\n\n",
                maxNormalizeDifferenceClosestPointMethod,
                maxNormalizedClosestMethodIdx,
                lines[maxNormalizedClosestMethodIdx].PointA.toString().c_str(),
                lines[maxNormalizedClosestMethodIdx].PointB.toString().c_str());

            printf("CrossProductMaxAnalysis: %d :: %s\n",
                   maxNormalizedCrossMethodIdx,
                   lines[maxNormalizedCrossMethodIdx].toString().c_str());
            Line<float> l;
            Line<double> ld;
            l = lines[maxNormalizedCrossMethodIdx];
            ld = {.PointA = Vec3<double>{l.PointA.x, l.PointA.y, l.PointA.z},
                  .PointB = Vec3<double>{l.PointB.x, l.PointB.y, l.PointB.z}};
            Vec3<double> rd;
            Vec3<float> r;
            rd = DirToOriginCrossProductMethod(ld).normalize();
            r = DirToOriginCrossProductMethod(l).normalize();
            printf("%s\n", rd.toString().c_str());
            printf("%s\n", r.toString().c_str());
            printf("%lf\n", (rd - Vec3<double>(r.x, r.y, r.z)).magnitude());

            printf("ClosestPointMaxAnalysis: %d :: %s\n",
                   maxNormalizedClosestMethodIdx,
                   lines[maxNormalizedClosestMethodIdx].toString().c_str());
            l = lines[maxNormalizedClosestMethodIdx];
            ld = {.PointA = Vec3<double>{l.PointA.x, l.PointA.y, l.PointA.z},
                  .PointB = Vec3<double>{l.PointB.x, l.PointB.y, l.PointB.z}};
            rd = DirToOriginClosestPointMethod(ld).normalize();
            r = DirToOriginClosestPointMethod(l).normalize();
            printf("%s\n", rd.toString().c_str());
            printf("%s\n", r.toString().c_str());
            printf("%lf\n", (rd - Vec3<double>(r.x, r.y, r.z)).magnitude());
        }

        // We see that the cross product method results in a more accurate
        // result when compared to the closest point method. This was evaluated
        // using the normalized direction vector in the final result. But the
        // cross product method also yields a larger result in general, So, it
        // might be good to check what's the effect of the larger result when
        // trying to find the support point. We will generate 100000 points, and
        // scan through each point for each resulting vector to find the
        // supporting point. We record the matches.
        //
        // Result: There were no violations
        SECTION("Support point tests") {

            const int NUM_POINTS = 100000;
            Vec3<double>* points = new Vec3<double>[NUM_POINTS];
            for (int i = 0; i < NUM_POINTS; ++i) {
                points[i] = GenerateVec3<double>(-10.0f, 10.0f);
            }

            int numViolations = 0;
            for (int i = 0; i < NUM_OF_LINES; ++i) {
                Line<float> line = lines[i];
                Line<double> lineDouble = toDouble(line);
                Vec3<double> controlDirectionCross =
                    DirToOriginCrossProductMethod(lineDouble);
                Vec3<double> controlDirectionClosest =
                    -DirToOriginClosestPointMethod(lineDouble);

                int pointControlCross =
                    GetSupportPoint(points, controlDirectionCross, NUM_POINTS);
                int pointControlClosest = GetSupportPoint(
                    points, controlDirectionClosest, NUM_POINTS);
                REQUIRE(pointControlCross == pointControlClosest);

                Vec3<float> dirCross = DirToOriginCrossProductMethod(line);
                Vec3<float> dirClosest = -DirToOriginClosestPointMethod(line);
                int pointCross =
                    GetSupportPoint(points, toDouble(dirCross), NUM_POINTS);
                int pointClosest =
                    GetSupportPoint(points, toDouble(dirClosest), NUM_POINTS);

                REQUIRE(pointCross == pointControlCross);
                REQUIRE(pointClosest == pointControlClosest);

                if (pointCross != pointClosest) {
                    printf("Line %d (%s) :: [%d  %s] vs [%d  %s]\n", i,
                           line.toString().c_str(), pointCross,
                           points[pointCross].toString().c_str(), pointClosest,
                           points[pointClosest].toString().c_str());
                    ++numViolations;
                }
            }

            printf("%d / %d violations\n", numViolations, NUM_POINTS);
        }
    }
}
