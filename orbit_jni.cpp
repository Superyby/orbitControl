// orbit_jni.cpp - SGP4 Orbit Propagation JNI Interface
// Based on Vallado SGP4 implementation

#include <jni.h>
#include <vector>
#include <cstring>
#include <cmath>
#include <string>
#include "sgp4io.h"
#include "sgp4coord.h"

extern "C" {

    /**
     * Propagate satellite orbit from TLE (Two-Line Element) data.
     *
     * @param line1         TLE line 1 (69 characters)
     * @param line2         TLE line 2 (69 characters)
     * @param durationHours Total propagation duration in hours
     * @param stepMinutes   Time step between points in minutes
     * @return double array with 9 values per point:
     *         [x, y, z, vx, vy, vz, lat, lon, alt]
     *         Position (km), Velocity (km/s), Lat/Lon (deg), Alt (km)
     */
    JNIEXPORT jdoubleArray JNICALL
        Java_com_orbitYu_orbit_OrbitPropagator_propagateFromTle(
            JNIEnv* env, jclass clazz,
            jstring line1,
            jstring line2,
            jdouble durationHours,
            jdouble stepMinutes)
    {
        // 1. Get TLE strings from Java
        const char* l1 = env->GetStringUTFChars(line1, nullptr);
        const char* l2 = env->GetStringUTFChars(line2, nullptr);
        if (!l1 || !l2) {
            env->ReleaseStringUTFChars(line1, l1);
            env->ReleaseStringUTFChars(line2, l2);
            env->ThrowNew(env->FindClass("java/lang/IllegalArgumentException"), "TLE lines cannot be null");
            return nullptr;
        }

        // 2. Parse TLE using Vallado SGP4
        elsetrec satrec;
        double startmfe, stopmfe, deltamin;
        char longstr1[130], longstr2[130];
        strncpy(longstr1, l1, 129); longstr1[129] = '\0';
        strncpy(longstr2, l2, 129); longstr2[129] = '\0';
        twoline2rv(longstr1, longstr2, 'c', 'e', 'i', wgs84, startmfe, stopmfe, deltamin, satrec);
        if (satrec.error != 0) {
            env->ReleaseStringUTFChars(line1, l1);
            env->ReleaseStringUTFChars(line2, l2);
            env->ThrowNew(env->FindClass("java/lang/IllegalArgumentException"), "Invalid TLE format");
            return nullptr;
        }

        // 3. Calculate time steps
        double totalMinutes = durationHours * 60.0;
        int steps = static_cast<int>(std::floor(totalMinutes / stepMinutes));
        if (steps < 0) steps = 0;
        int totalPoints = steps + 1;

        // 4. Trajectory storage (9 values per point: x,y,z,vx,vy,vz,lat,lon,alt)
        std::vector<double> trajectory;
        trajectory.reserve(totalPoints * 9);

        // Radian to degree conversion
        const double rad2deg = 180.0 / pi;

        // 5. SGP4 propagation loop
        for (int i = 0; i < totalPoints; ++i) {
            double tsince = i * stepMinutes; // minutes since TLE epoch
            double r[3], v[3];

            if (!sgp4(wgs84, satrec, tsince, r, v)) {
                env->ReleaseStringUTFChars(line1, l1);
                env->ReleaseStringUTFChars(line2, l2);
                env->ThrowNew(env->FindClass("java/lang/RuntimeException"),
                    ("SGP4 propagation failed at step " + std::to_string(i)).c_str());
                return nullptr;
            }

            // State: [x,y,z,vx,vy,vz] in TEME (km, km/s)
            trajectory.push_back(r[0]);
            trajectory.push_back(r[1]);
            trajectory.push_back(r[2]);
            trajectory.push_back(v[0]);
            trajectory.push_back(v[1]);
            trajectory.push_back(v[2]);

            // Convert TEME to ECEF, then to Lat/Lon/Alt
            double jdut1 = satrec.jdsatepoch + tsince / 1440.0; // Julian date at this time
            double recef[3], vecef[3];
            teme2ecef(r, v, jdut1, recef, vecef);

            double latlongh[3]; // [lat(rad), lon(rad), alt(km)]
            ijk2ll(recef, latlongh);

            // Append lat, lon (in degrees), alt (in km)
            trajectory.push_back(latlongh[0] * rad2deg);  // latitude (deg)
            trajectory.push_back(latlongh[1] * rad2deg);  // longitude (deg)
            trajectory.push_back(latlongh[2]);            // altitude (km)
        }

        // 6. Release TLE strings
        env->ReleaseStringUTFChars(line1, l1);
        env->ReleaseStringUTFChars(line2, l2);

        // 7. Return flat array to Java
        jsize arraySize = static_cast<jsize>(trajectory.size());
        jdoubleArray result = env->NewDoubleArray(arraySize);
        if (!result) {
            env->ThrowNew(env->FindClass("java/lang/OutOfMemoryError"), "Failed to allocate result array");
            return nullptr;
        }
        env->SetDoubleArrayRegion(result, 0, arraySize, trajectory.data());
        return result;
    }

} // extern "C"
