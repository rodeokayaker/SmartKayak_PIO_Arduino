#include "CalibrationUtils.h"
#include <Arduino.h>

namespace CalibrationUtils {

/* void estimateEllipsoid(float x1, float y1, float z1,
                       float x2, float y2, float z2,
                       float x3, float y3, float z3,
                       float x4, float y4, float z4,
                       float x5, float y5, float z5,
                       float x6, float y6, float z6,
                       float* centerX, float* centerY, float* centerZ,
                       float* radX, float* radY, float* radZ) {
    // Формируем матрицу A и вектор b для решения системы линейных уравнений
    float A[6][6] = {
        {x1*x1, y1*y1, z1*z1, 2*x1, 2*y1, 2*z1},
        {x2*x2, y2*y2, z2*z2, 2*x2, 2*y2, 2*z2},
        {x3*x3, y3*y3, z3*z3, 2*x3, 2*y3, 2*z3},
        {x4*x4, y4*y4, z4*z4, 2*x4, 2*y4, 2*z4},
        {x5*x5, y5*y5, z5*z5, 2*x5, 2*y5, 2*z5},
        {x6*x6, y6*y6, z6*z6, 2*x6, 2*y6, 2*z6}
    };
    float b[6] = {1, 1, 1, 1, 1, 1};

    // Решаем систему линейных уравнений методом Гаусса-Жордана
    for (int i = 0; i < 6; i++) {
        // Ищем максимальный элемент в столбце i
        int maxRow = i;
        for (int j = i+1; j < 6; j++) {
            if (fabs(A[j][i]) > fabs(A[maxRow][i])) {
                maxRow = j;
            }
        }

        // Переставляем строки i и maxRow
        for (int k = i; k < 6; k++) {
            float tmp = A[maxRow][k];
            A[maxRow][k] = A[i][k];
            A[i][k] = tmp;
        }
        float tmp = b[maxRow];
        b[maxRow] = b[i];
        b[i] = tmp;

        // Приводим к верхнетреугольному виду
        for (int j = i+1; j < 6; j++) {
            float c = -A[j][i] / A[i][i];
            for (int k = i; k < 6; k++) {
                if (i == k) {
                    A[j][k] = 0;
                } else {
                    A[j][k] += c * A[i][k];
                }
            }
            b[j] += c * b[i];
        }
    }

    // Обратная подстановка
    float x[6];
    for (int i = 5; i >= 0; i--) {
        x[i] = b[i] / A[i][i];
        for (int j = i-1; j >= 0; j--) {
            b[j] -= A[j][i] * x[i];
        }
    }

    // Извлекаем параметры эллипсоида из решения
    float f_a = x[0], f_b = x[1], f_c = x[2];
    float f_d = x[3], f_e = x[4], f_f = x[5];
    
    *centerX = -f_d / (2*f_a);
    *centerY = -f_e / (2*f_b);
    *centerZ = -f_f / (2*f_c);
    
    float denomX = f_a * (*centerX) * (*centerX) + f_d * (*centerX) - 1;
    float denomY = f_b * (*centerY) * (*centerY) + f_e * (*centerY) - 1;
    float denomZ = f_c * (*centerZ) * (*centerZ) + f_f * (*centerZ) - 1;
    
    *radX = sqrt(1 / denomX);
    *radY = sqrt(1 / denomY);
    *radZ = sqrt(1 / denomZ);
} */

bool estimateEllipsoid(float x1, float y1, float z1,
                       float x2, float y2, float z2,
                       float x3, float y3, float z3,
                       float x4, float y4, float z4,
                       float x5, float y5, float z5,
                       float x6, float y6, float z6,
                       float* centerX, float* centerY, float* centerZ,
                       float* radX, float* radY, float* radZ) {
    // Нормализация входных данных
    float xm = (x1 + x2 + x3 + x4 + x5 + x6) / 6;
    float ym = (y1 + y2 + y3 + y4 + y5 + y6) / 6;
    float zm = (z1 + z2 + z3 + z4 + z5 + z6) / 6;
    
    float scale = 0;
    for(int i = 0; i < 6; i++) {
        float dx = *(&x1 + i*3) - xm;
        float dy = *(&y1 + i*3) - ym;
        float dz = *(&z1 + i*3) - zm;
        scale += sqrt(dx*dx + dy*dy + dz*dz);
    }
    scale /= 6;
    if(scale < 1e-6) return false;

    // Нормализованные координаты
    float nx1 = (x1 - xm)/scale, ny1 = (y1 - ym)/scale, nz1 = (z1 - zm)/scale;
    float nx2 = (x2 - xm)/scale, ny2 = (y2 - ym)/scale, nz2 = (z2 - zm)/scale;
    float nx3 = (x3 - xm)/scale, ny3 = (y3 - ym)/scale, nz3 = (z3 - zm)/scale;
    float nx4 = (x4 - xm)/scale, ny4 = (y4 - ym)/scale, nz4 = (z4 - zm)/scale;
    float nx5 = (x5 - xm)/scale, ny5 = (y5 - ym)/scale, nz5 = (z5 - zm)/scale;
    float nx6 = (x6 - xm)/scale, ny6 = (y6 - ym)/scale, nz6 = (z6 - zm)/scale;

    // Матрица системы уравнений
    float D[6][6] = {
        {nx1*nx1, ny1*ny1, nz1*nz1, nx1, ny1, nz1},
        {nx2*nx2, ny2*ny2, nz2*nz2, nx2, ny2, nz2},
        {nx3*nx3, ny3*ny3, nz3*nz3, nx3, ny3, nz3},
        {nx4*nx4, ny4*ny4, nz4*nz4, nx4, ny4, nz4},
        {nx5*nx5, ny5*ny5, nz5*nz5, nx5, ny5, nz5},
        {nx6*nx6, ny6*ny6, nz6*nz6, nx6, ny6, nz6}
    };
    float v[6] = {1, 1, 1, 1, 1, 1};

    // Решение методом Гаусса с выбором главного элемента
    for(int i = 0; i < 6; i++) {
        // Поиск максимального элемента для стабильности
        int maxRow = i;
        float maxVal = fabs(D[i][i]);
        for(int j = i + 1; j < 6; j++) {
            if(fabs(D[j][i]) > maxVal) {
                maxVal = fabs(D[j][i]);
                maxRow = j;
            }
        }
        if(maxVal < 1e-10) return false;

        // Перестановка строк
        if(maxRow != i) {
            for(int j = i; j < 6; j++) {
                float temp = D[i][j];
                D[i][j] = D[maxRow][j];
                D[maxRow][j] = temp;
            }
            float temp = v[i];
            v[i] = v[maxRow];
            v[maxRow] = temp;
        }

        // Исключение переменных
        for(int j = i + 1; j < 6; j++) {
            float c = D[j][i] / D[i][i];
            for(int k = i; k < 6; k++) {
                D[j][k] -= c * D[i][k];
            }
            v[j] -= c * v[i];
        }
    }

    // Обратный ход
    float solution[6];
    for(int i = 5; i >= 0; i--) {
        solution[i] = v[i];
        for(int j = i + 1; j < 6; j++) {
            solution[i] -= D[i][j] * solution[j];
        }
        solution[i] /= D[i][i];
    }

    // Восстановление параметров эллипсоида
    *centerX = -solution[3]/(2*solution[0])*scale + xm;
    *centerY = -solution[4]/(2*solution[1])*scale + ym;
    *centerZ = -solution[5]/(2*solution[2])*scale + zm;

    *radX = scale/sqrt(solution[0]);
    *radY = scale/sqrt(solution[1]);
    *radZ = scale/sqrt(solution[2]);

    // Проверка корректности результата
    if(*radX <= 0 || *radY <= 0 || *radZ <= 0) return false;
    
    return true;
}

void ransacEllipsoidFilter(float* x, float* y, float* z, bool* inliers, int n,
                           float threshold, int maxIterations, float* bestCenterX, float* bestCenterY, float* bestCenterZ,
                           float* bestRadX, float* bestRadY, float* bestRadZ) {
    int bestInliers = -1;
//    float bestCenterX, bestCenterY, bestCenterZ;
//    float bestRadX, bestRadY, bestRadZ;

    for (int iter = 0; iter < maxIterations; iter++) {
        // Случайно выбираем 6 точек для оценки параметров эллипсоида
        int idx1 = rand() % n;
        int idx2 = rand() % n;
        int idx3 = rand() % n;
        int idx4 = rand() % n;
        int idx5 = rand() % n;
        int idx6 = rand() % n;

        // Оцениваем параметры эллипсоида по выбранным точкам
        float centerX, centerY, centerZ;
        float radX, radY, radZ;
        if (!estimateEllipsoid(x[idx1], y[idx1], z[idx1],
                          x[idx2], y[idx2], z[idx2],
                          x[idx3], y[idx3], z[idx3],
                          x[idx4], y[idx4], z[idx4],
                          x[idx5], y[idx5], z[idx5],
                          x[idx6], y[idx6], z[idx6],
                          &centerX, &centerY, &centerZ,
                          &radX, &radY, &radZ)) continue;

        // Классифицируем все точки как инлайеры или выбросы
        int inlierCount = 0;
        for (int i = 0; i < n; i++) {
            float dx = x[i] - centerX;
            float dy = y[i] - centerY;
            float dz = z[i] - centerZ;
            float dist = sqrt(dx*dx/radX/radX + dy*dy/radY/radY + dz*dz/radZ/radZ);
            if (dist < threshold) {
                inlierCount++;
            }
        }

        // Обновляем лучшую модель, если текущая имеет больше инлайеров
        if (inlierCount > bestInliers) {
            bestInliers = inlierCount;
            *bestCenterX = centerX;
            *bestCenterY = centerY;
            *bestCenterZ = centerZ;
            *bestRadX = radX;
            *bestRadY = radY;
            *bestRadZ = radZ;
        }
    }
    // Классифицируем точки как инлайеры или выбросы относительно лучшей модели
    bestInliers = 0;
    for (int i = 0; i < n; i++) {
        float dx = x[i] - *bestCenterX;
        float dy = y[i] - *bestCenterY;
        float dz = z[i] - *bestCenterZ;
        float dist = sqrt(dx*dx/(*bestRadX)*(*bestRadX) + dy*dy/(*bestRadY)/(*bestRadY) + dz*dz/(*bestRadZ)/(*bestRadZ));
        inliers[i] = (dist < threshold);
        if (inliers[i]) bestInliers++;
    }
    Serial.printf("bestInliers = %d\n", bestInliers);
}

/*
void geometricCalibration(float* x, float* y, float* z, int n,
                                          float& centerX, float& centerY, float& centerZ,
                                          float& radX, float& radY, float& radZ, int maxIterations, float tolerance, 
                                          bool* inliers=nullptr) {

    // Итеративная минимизация геометрической ошибки
    for (int iter = 0; iter < maxIterations; iter++) {
        float dCenterX = 0, dCenterY = 0, dCenterZ = 0;
        float dRadX = 0, dRadY = 0, dRadZ = 0;

        for (int i = 0; i < n; i++) {
            if (inliers&&!inliers[i]) continue;
            float dx = x[i] - centerX;
            float dy = y[i] - centerY;
            float dz = z[i] - centerZ;

            float dist = sqrt(dx*dx/radX/radX + dy*dy/radY/radY + dz*dz/radZ/radZ);
            float err = dist - 1;

            float nx = dx/radX/radX/dist;
            float ny = dy/radY/radY/dist;
            float nz = dz/radZ/radZ/dist;

            dCenterX += err * nx;
            dCenterY += err * ny;
            dCenterZ += err * nz;

            dRadX += err * dx*dx/radX/radX/radX/dist;
            dRadY += err * dy*dy/radY/radY/radY/dist;
            dRadZ += err * dz*dz/radZ/radZ/radZ/dist;
        }

        centerX += dCenterX / n;
        centerY += dCenterY / n;
        centerZ += dCenterZ / n;

        radX += dRadX / n;
        radY += dRadY / n;
        radZ += dRadZ / n;

        if (fabs(dCenterX/n) < tolerance && fabs(dCenterY/n) < tolerance && fabs(dCenterZ/n) < tolerance &&
            fabs(dRadX/n) < tolerance && fabs(dRadY/n) < tolerance && fabs(dRadZ/n) < tolerance) {
            break;
        }
    }

} */

void geometricCalibration(float* x, float* y, float* z, int n,
                         float& centerX, float& centerY, float& centerZ,
                         float& radX, float& radY, float& radZ, 
                         int maxIterations, float tolerance, 
                         bool* inliers) {
    
    int validPoints;
    for (int iter = 0; iter < maxIterations; iter++) {
        float dCenterX = 0, dCenterY = 0, dCenterZ = 0;
        float dRadX = 0, dRadY = 0, dRadZ = 0;
        validPoints = 0;

        for (int i = 0; i < n; i++) {
            if (inliers && !inliers[i]) continue;
            
            float dx = x[i] - centerX;
            float dy = y[i] - centerY;
            float dz = z[i] - centerZ;

            // Проверка деления на ноль
            if (radX <= 0 || radY <= 0 || radZ <= 0) continue;

            float termX = dx*dx/(radX*radX);
            float termY = dy*dy/(radY*radY);
            float termZ = dz*dz/(radZ*radZ);
            
            float dist = sqrt(termX + termY + termZ);
            if (dist < 1e-6) continue;  // Избегаем деления на очень маленькие числа

            float err = dist - 1;

            float nx = dx/(radX*radX*dist);
            float ny = dy/(radY*radY*dist);
            float nz = dz/(radZ*radZ*dist);

            dCenterX += err * nx;
            dCenterY += err * ny;
            dCenterZ += err * nz;

            dRadX += err * dx*dx/(radX*radX*radX*dist);
            dRadY += err * dy*dy/(radY*radY*radY*dist);
            dRadZ += err * dz*dz/(radZ*radZ*radZ*dist);
            
            validPoints++;
        }

        // Проверяем, что есть достаточно точек для обновления
        if (validPoints < 6) {  // Минимум 6 точек нужно для определения эллипсоида
            break;
        }

        float scale = 1.0f / validPoints;
        centerX += dCenterX * scale;
        centerY += dCenterY * scale;
        centerZ += dCenterZ * scale;

        radX += dRadX * scale;
        radY += dRadY * scale;
        radZ += dRadZ * scale;

        // Проверяем, что радиусы остаются положительными
        if (radX <= 0) radX = 0.1f;
        if (radY <= 0) radY = 0.1f;
        if (radZ <= 0) radZ = 0.1f;

        // Проверка сходимости
        if (fabs(dCenterX*scale) < tolerance && 
            fabs(dCenterY*scale) < tolerance && 
            fabs(dCenterZ*scale) < tolerance &&
            fabs(dRadX*scale) < tolerance && 
            fabs(dRadY*scale) < tolerance && 
            fabs(dRadZ*scale) < tolerance) {
            break;
        }
    }
}

void ellipsoidFitting(float* x, float* y, float* z, int n,
                                      float* centerX, float* centerY, float* centerZ,
                                      float* scaleX, float* scaleY, float* scaleZ) {
    // Вычисление матрицы A и вектора b для метода наименьших квадратов
    float A[9] = {0};
    float b_vec[3] = {0};
    for(int i = 0; i < n; i++) {
        A[0] += x[i] * x[i];
        A[1] += x[i] * y[i];
        A[2] += x[i] * z[i];
        A[4] += y[i] * y[i];
        A[5] += y[i] * z[i];
        A[8] += z[i] * z[i];
        
        b_vec[0] += x[i];
        b_vec[1] += y[i];
        b_vec[2] += z[i];
    }
    A[3] = A[1];
    A[6] = A[2];
    A[7] = A[5];
    
    // Решение системы уравнений методом Крамера
    float detA = A[0]*(A[4]*A[8]-A[5]*A[7]) - A[1]*(A[3]*A[8]-A[5]*A[6]) + A[2]*(A[3]*A[7]-A[4]*A[6]);
    
    *centerX = (b_vec[0]*(A[4]*A[8]-A[5]*A[7]) - A[1]*(b_vec[1]*A[8]-A[5]*b_vec[2]) + A[2]*(b_vec[1]*A[7]-A[4]*b_vec[2])) / detA;
    *centerY = (A[0]*(b_vec[1]*A[8]-A[5]*b_vec[2]) - b_vec[0]*(A[3]*A[8]-A[5]*A[6]) + A[2]*(A[3]*b_vec[2]-b_vec[1]*A[6])) / detA;
    *centerZ = (A[0]*(A[4]*b_vec[2]-b_vec[1]*A[7]) - A[1]*(A[3]*b_vec[2]-b_vec[1]*A[6]) + b_vec[0]*(A[3]*A[7]-A[4]*A[6])) / detA;
    Serial.printf("centerX = %f, centerY = %f, centerZ = %f\n", *centerX, *centerY, *centerZ);
    // Вычисление ковариационной матрицы
    float covXX = 0, covXY = 0, covXZ = 0, covYY = 0, covYZ = 0, covZZ = 0;
    for(int i = 0; i < n; i++) {
        float dx = x[i] - *centerX;
        float dy = y[i] - *centerY;
        float dz = z[i] - *centerZ;
        covXX += dx * dx;
        covXY += dx * dy;
        covXZ += dx * dz;
        covYY += dy * dy;
        covYZ += dy * dz;
        covZZ += dz * dz;
    }
    covXX /= n;
    covXY /= n;
    covXZ /= n;
    covYY /= n;
    covYZ /= n;
    covZZ /= n;


    // Вычисление собственных значений и векторов матрицы ковариации
    float a = covYY*covZZ - covYZ*covYZ;
    float b = covXZ*covYZ - covXY*covZZ;
    float c = covXY*covYZ - covXZ*covYY;
    float d = covXX*covZZ - covXZ*covXZ;
    float e = covXX*covYY - covXY*covXY;
    float f = covXX*a + covXY*b + covXZ*c;

    float q = (covXX + covYY + covZZ) / 3;
    float p = (covXX - q)*(covXX - q) + (covYY - q)*(covYY - q) + (covZZ - q)*(covZZ - q) + 2*(covXY*covXY + covXZ*covXZ + covYZ*covYZ);
    p = sqrt(p / 6);

    float r = (1/p) * ((covXX - q)*(covYY - q)*(covZZ - q) + 2*covXY*covXZ*covYZ - (covXX - q)*covYZ*covYZ - (covYY - q)*covXZ*covXZ - (covZZ - q)*covXY*covXY);
    r = r / 2;

    float phi = acos(r) / 3;
    float eig1 = q + 2*p*cos(phi);
    float eig2 = q + 2*p*cos(phi + 2*M_PI/3);
    float eig3 = 3*q - eig1 - eig2;

    float vec1X = (a*(eig1 - covYY) - b*covXY) / (covXY*(eig1 - covYY) - (eig1 - covXX)*covYZ);
    float vec1Y = (b*(eig1 - covXX) - a*covXY) / (covXY*(eig1 - covYY) - (eig1 - covXX)*covYZ);
    float vec1Z = 1;

    float vec2X = (a*(eig2 - covYY) - b*covXY) / (covXY*(eig2 - covYY) - (eig2 - covXX)*covYZ);
    float vec2Y = (b*(eig2 - covXX) - a*covXY) / (covXY*(eig2 - covYY) - (eig2 - covXX)*covYZ);
    float vec2Z = 1;

    float vec3X = vec1Y*vec2Z - vec1Z*vec2Y;
    float vec3Y = vec1Z*vec2X - vec1X*vec2Z;
    float vec3Z = vec1X*vec2Y - vec1Y*vec2X;

    // Нормализация собственных векторов
    float norm1 = sqrt(vec1X*vec1X + vec1Y*vec1Y + vec1Z*vec1Z);
    vec1X /= norm1;
    vec1Y /= norm1;
    vec1Z /= norm1;

    float norm2 = sqrt(vec2X*vec2X + vec2Y*vec2Y + vec2Z*vec2Z);
    vec2X /= norm2;
    vec2Y /= norm2;
    vec2Z /= norm2;

    float norm3 = sqrt(vec3X*vec3X + vec3Y*vec3Y + vec3Z*vec3Z);
    vec3X /= norm3;
    vec3Y /= norm3;
    vec3Z /= norm3;


    *scaleX = 1.0f / sqrt(eig1);
    *scaleY = 1.0f / sqrt(eig2);
    *scaleZ = 1.0f / sqrt(eig3);
}

}