#ifndef CALIBRATIONUTILS_H
#define CALIBRATIONUTILS_H

namespace CalibrationUtils {
    bool estimateEllipsoid(float x1, float y1, float z1,
                        float x2, float y2, float z2,
                        float x3, float y3, float z3,
                        float x4, float y4, float z4,
                        float x5, float y5, float z5,
                        float x6, float y6, float z6,
                        float* centerX, float* centerY, float* centerZ,
                        float* radX, float* radY, float* radZ);

    void ransacEllipsoidFilter(float* x, float* y, float* z, bool* inliers, int n,
                            float threshold, int maxIterations, float* bestCenterX, float* bestCenterY, float* bestCenterZ,
                            float* bestRadX, float* bestRadY, float* bestRadZ);

    void geometricCalibration(float* x, float* y, float* z, int n,
                            float& centerX, float& centerY, float& centerZ,
                            float& radX, float& radY, float& radZ, 
                            int maxIterations, float tolerance, 
                            bool* inliers=nullptr);

    void ellipsoidFitting(float* x, float* y, float* z, int n,
                                        float* centerX, float* centerY, float* centerZ,
                                        float* scaleX, float* scaleY, float* scaleZ);
}


#endif