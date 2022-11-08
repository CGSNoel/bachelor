#include "CudaRaytracing.h"


// helpers for vector operations
__host__ __device__ float3 operator+(float3 vec1, float3 vec2) {
    float3 result;
    result.x = vec1.x + vec2.x;
    result.y = vec1.y + vec2.y;
    result.z = vec1.z + vec2.z;
    return result;
}

__host__ __device__ float3 operator-(float3 vec1, float3 vec2) {
    float3 result;
    result.x = vec1.x - vec2.x;
    result.y = vec1.y - vec2.y;
    result.z = vec1.z - vec2.z;
    return result;
}

__host__ __device__ float3 operator*(float3 vec1, float scal) {
    float3 result;
    result.x = vec1.x * scal;
    result.y = vec1.y * scal;
    result.z = vec1.z * scal;
    return result;
}

__host__ __device__ float3 operator*(float scal, float3 vec1) {
    float3 result;
    result.x = vec1.x * scal;
    result.y = vec1.y * scal;
    result.z = vec1.z * scal;
    return result;
}

__host__ __device__ float3 operator*(float3 vec1, float3 vec2)
{
    return float3{ vec1.x * vec2.x, vec1.y * vec2.y, vec1.z * vec2.z };
}

__host__ __device__ float3 operator/(float3 vec1, float scal) {
    float3 result;
    result.x = vec1.x / scal;
    result.y = vec1.y / scal;
    result.z = vec1.z / scal;
    return result;
}

// vector length
__host__ __device__ float veclen(float3 vec) {
    float len = sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
    return len;
}

// normalizes vector
__host__ __device__ float3 norm(float3 vec) {
    return vec / veclen(vec);
}

// dot product
__host__ __device__ float dot(float3 vec1, float3 vec2) {
    return vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z;
}

// cross product
__host__ __device__ float3 cross(float3 vec1, float3 vec2){
    float3 result;
    result.x = vec1.y * vec2.z - vec1.z * vec2.y;
    result.y = vec1.z * vec2.x - vec1.x * vec2.z;
    result.z = vec1.x * vec2.y - vec1.y * vec2.x;
    return result;
}

// reflect vector
__host__ __device__ float3 reflect(float3 vec, float3 N) {
    return vec - 2 * (dot(vec, N) * N);
}

// clamp color
__host__ __device__ void clampCol(float3& col) {
    if (col.x > 1.0) col.x = 1.0;
    if (col.y > 1.0) col.y = 1.0;
    if (col.z > 1.0) col.z = 1.0;
}

// std::fabs can't be called from device functions :^)
__host__ __device__ float absf(float x) {
    if (x >= 0) return x;
    else return -x;
}

// std::fmax can't be called from device functions :^)
__host__ __device__ float maxf(float x, float y) {
    if (x > y) return x;
    else return y;
}

// END OF HELPERS   --------------------------------------------------------------------
// END OF HELPERS   --------------------------------------------------------------------
// END OF HELPERS   --------------------------------------------------------------------


// get interval
__device__ float2 getInterval(float3 rayO, float3 rayD, float3* d_kdInfo, int nodeNum) {

    if (rayD.x == 0 && (rayO.x < d_kdInfo[3 * nodeNum].x || rayO.x > d_kdInfo[3 * nodeNum].y)) return { -1.0f,-1.0f };
    if (rayD.y == 0 && (rayO.y < d_kdInfo[3 * nodeNum].z || rayO.y > d_kdInfo[3 * nodeNum + 1].x)) return { -1.0f,-1.0f };    // ray is parallel and doesn't intersect
    if (rayD.z == 0 && (rayO.z < d_kdInfo[3 * nodeNum + 1].y || rayO.z > d_kdInfo[3 * nodeNum + 1].z)) return { -1.0f,-1.0f };


    float2 ts;
    float invRdx = 1 / rayD.x;
    float invRdy = 1 / rayD.y;
    float invRdz = 1 / rayD.z;

    float t1;
    float t2;
    float temp;

    // getting the x interval
    ts.x = (d_kdInfo[3 * nodeNum].x - rayO.x) * invRdx; // tstart
    ts.y = (d_kdInfo[3 * nodeNum].y - rayO.x) * invRdx; // tend
    if (ts.x > ts.y) {
        temp = ts.x;
        ts.x = ts.y;
        ts.y = temp;
    }
    if (ts.x < 0) ts.x = 0.0f;
    if (ts.y < 0) return ts;    // box is behind eye in x

    // y interval
    t1 = (d_kdInfo[3 * nodeNum].z - rayO.y) * invRdy;
    t2 = (d_kdInfo[3 * nodeNum + 1].x - rayO.y) * invRdy;
    if (t1 > t2) {
        temp = t1;
        t1 = t2;
        t2 = temp;
    }
    if (t1 < 0) t1 = 0.0f;
    if (t2 < 0) return { ts.x, t2 };    // box is behind eye in y
    if (t1 > ts.x) ts.x = t1;
    if (t2 < ts.y) ts.y = t2;

    // z interval
    t1 = (d_kdInfo[3 * nodeNum + 1].y - rayO.z) * invRdz;
    t2 = (d_kdInfo[3 * nodeNum + 1].z - rayO.z) * invRdz;
    if (t1 > t2) {
        temp = t1;
        t1 = t2;
        t2 = temp;
    }
    if (t1 < 0) t1 = 0.0f;
    if (t2 < 0) return { ts.x, t2 };    // box is behind eye in z
    if (t1 > ts.x) ts.x = t1;
    if (t2 < ts.y) ts.y = t2;

    return ts;
}

// intersect 
__device__ GPUHit intersect(float3* d_triangles, int triangleIdx, float3 rayO, float3 rayD) {
    float3 a = d_triangles[triangleIdx];
    float3 b = d_triangles[triangleIdx + 1];
    float3 c = d_triangles[triangleIdx + 2];


    float3 E0 = b - a;
    float3 E2 = c - a;
    float3 N = cross(E0, E2);
    float det = dot(E0, cross(rayD, E2));

    if (absf(det) < 1e-6) return GPUHit(__builtin_nanf("0"), { __builtin_nanf("0"),__builtin_nanf("0"),__builtin_nanf("0") }, -1, true);
    float invdet = 1 / det;
    float3 tvec = rayO - a;
    float3 qvec = cross(tvec, E0);

    float u = dot(tvec, cross(rayD, E2)) * invdet;
    if (u < 0 || u > 1) return GPUHit(__builtin_nanf("0"), { __builtin_nanf("0"),__builtin_nanf("0"),__builtin_nanf("0") }, -1, true);
    float v = dot(rayD, qvec) * invdet;
    if (v < 0 || u + v > 1) return GPUHit(__builtin_nanf("0"), { __builtin_nanf("0"),__builtin_nanf("0"),__builtin_nanf("0") }, -1, true);

    float t = dot(E2, qvec) * invdet;
    if (t < 0) return GPUHit(__builtin_nanf("0"), { __builtin_nanf("0"),__builtin_nanf("0"),__builtin_nanf("0") }, -1, true);
    return GPUHit(t, norm(N), triangleIdx);
}

// traversal function
// kdInfo: (min_x, max_x, min_y, max_y, min_z, max_z, splitDist, tstart, tend) DISCLAIMER: tstart and tend are dummies, they have to be updated for each ray individually, they don't get updated in global memory
// leafIndices: (splitDim, triangleStart, triangleEnd)
__device__ GPUHit cudaTraverse(float3* d_triangles, int* d_triIndices, int3* d_leafIndices, float3* d_kdInfo, float3 rayO, float3 rayD, int nodeNum, int numNodes, float tstart, float tend) {

    // get interval for root node
    if (nodeNum == 0) {
        float2 ts = getInterval(rayO, rayD, d_kdInfo, nodeNum);
        tstart = ts.x;
        tend = ts.y;
        if (tstart > 0 && tstart > tend) return GPUHit(__builtin_nanf("0"), { __builtin_nanf("0"),__builtin_nanf("0"),__builtin_nanf("0") }, -1, true);
        if (tend < 0) return GPUHit(__builtin_nanf("0"), { __builtin_nanf("0"),__builtin_nanf("0"),__builtin_nanf("0") }, -1, true);
    }


    if (nodeNum >= numNodes / 2) {  // leaf case

        GPUHit min_hit(__builtin_huge_valf(), float3{});

        // loop for intersections with leaf triangles
        int intersectStart = d_leafIndices[nodeNum].y;
        int intersectEnd = d_leafIndices[nodeNum].z;
        for (int i = intersectStart; i <= intersectEnd; i++) {
            GPUHit hit(intersect(d_triangles, 5 * d_triIndices[i], rayO, rayD));
            if (hit.t < min_hit.t) {
                min_hit = hit;
            }
        }

        return min_hit;

        // relevant for overlapping triangles in different bounding boxes
        /*if (min_hit.t <= tend && min_hit.t >= tstart) return min_hit;
        else return GPUHit(__builtin_nanf("0"), { __builtin_nanf("0"),__builtin_nanf("0"),__builtin_nanf("0") }, -1, true);*/
    }

    float splitDist = d_kdInfo[3 * nodeNum + 2].x;
    int splitDim = d_leafIndices[nodeNum].x;
    float direction;
    float t;

    //                                                      min[splitDim]                        
    if (splitDim == 0) { direction = 1 / rayD.x; t = (d_kdInfo[3 * nodeNum].x - rayO.x + splitDist) * direction; }
    if (splitDim == 1) { direction = 1 / rayD.y; t = (d_kdInfo[3 * nodeNum].z - rayO.y + splitDist) * direction; }
    if (splitDim == 2) { direction = 1 / rayD.z; t = (d_kdInfo[3 * nodeNum + 1].y - rayO.z + splitDist) * direction; }

    if (direction < 0.0f) {    // swap front and back if the ray comes from the opposite direction along the split dimension
        if (t <= tstart) {  // only intersect front
            return cudaTraverse(d_triangles, d_triIndices, d_leafIndices, d_kdInfo, rayO, rayD, nodeNum * 2 + 1, numNodes, tstart, tend);    // front
        }
        else if (t >= tend) {   // only intersect back
            return cudaTraverse(d_triangles, d_triIndices, d_leafIndices, d_kdInfo, rayO, rayD, nodeNum * 2 + 2, numNodes, tstart, tend);   // back
        }
        else {
            GPUHit hit = cudaTraverse(d_triangles, d_triIndices, d_leafIndices, d_kdInfo, rayO, rayD, nodeNum * 2 + 2, numNodes, tstart, t);    // back
            if (hit.t <= t) return hit;
            else return cudaTraverse(d_triangles, d_triIndices, d_leafIndices, d_kdInfo, rayO, rayD, nodeNum * 2 + 1, numNodes, t, tend);   // front
        }
    }
    else {
        if (t <= tstart) {
            return cudaTraverse(d_triangles, d_triIndices, d_leafIndices, d_kdInfo, rayO, rayD, nodeNum * 2 + 2, numNodes, tstart, tend);    // back
        }
        else if (t >= tend) {
            return cudaTraverse(d_triangles, d_triIndices, d_leafIndices, d_kdInfo, rayO, rayD, nodeNum * 2 + 1, numNodes, tstart, tend);   // front
        }
        else {
            GPUHit hit = cudaTraverse(d_triangles, d_triIndices, d_leafIndices, d_kdInfo, rayO, rayD, nodeNum * 2 + 1, numNodes, tstart, t);    // front
            if (hit.t <= t) return hit;
            else return cudaTraverse(d_triangles, d_triIndices, d_leafIndices, d_kdInfo, rayO, rayD, nodeNum * 2 + 2, numNodes, t, tend);    // back
        }
    }
}

// same as cudaTraverse but it returns on the first hit rather than the closest hit
__device__ GPUHit shadowTraverse(float3* d_triangles, int* d_triIndices, int3* d_leafIndices, float3* d_kdInfo, float3 rayO, float3 rayD, int nodeNum, int numNodes, float tstart, float tend) {

    // get interval for root node
    if (nodeNum == 0) {
        float2 ts = getInterval(rayO, rayD, d_kdInfo, nodeNum);
        tstart = ts.x;
        tend = ts.y;
        if (tstart > 0 && tstart > tend) return GPUHit(__builtin_nanf("0"), { __builtin_nanf("0"),__builtin_nanf("0"),__builtin_nanf("0") }, -1, true);
        if (tend < 0) return GPUHit(__builtin_nanf("0"), { __builtin_nanf("0"),__builtin_nanf("0"),__builtin_nanf("0") }, -1, true);
    }


    if (nodeNum >= numNodes / 2) {  // leaf case

        GPUHit min_hit(__builtin_huge_valf(), float3{});

        // loop for intersections with leaf triangles
        int intersectStart = d_leafIndices[nodeNum].y;
        int intersectEnd = d_leafIndices[nodeNum].z;
        for (int i = intersectStart; i <= intersectEnd; i++) {
            GPUHit hit(intersect(d_triangles, 5 * d_triIndices[i], rayO, rayD));
            if (hit.t < min_hit.t && d_triangles[hit.hitobject + 4].z == 0) {
                min_hit = hit;
                break;
            }
        }

        return min_hit;

        // relevant for overlapping triangles in different bounding boxes
        /*if (min_hit.t <= tend && min_hit.t >= tstart) return min_hit;
        else return GPUHit(__builtin_nanf("0"), { __builtin_nanf("0"),__builtin_nanf("0"),__builtin_nanf("0") }, -1, true);*/
    }

    float splitDist = d_kdInfo[3 * nodeNum + 2].x;
    int splitDim = d_leafIndices[nodeNum].x;
    float direction;
    float t;

    //                                                    min[splitDim]                        
    if (splitDim == 0) { direction = 1 / rayD.x; t = (d_kdInfo[3 * nodeNum].x - rayO.x + splitDist) * direction; }
    if (splitDim == 1) { direction = 1 / rayD.y; t = (d_kdInfo[3 * nodeNum].z - rayO.y + splitDist) * direction; }
    if (splitDim == 2) { direction = 1 / rayD.z; t = (d_kdInfo[3 * nodeNum + 1].y - rayO.z + splitDist) * direction; }

    if (direction < 0.0f) {    // swap front and back if the ray comes from the opposite direction along the split dimension
        if (t <= tstart) {  // only intersect front
            return shadowTraverse(d_triangles, d_triIndices, d_leafIndices, d_kdInfo, rayO, rayD, nodeNum * 2 + 1, numNodes, tstart, tend);    // front
        }
        else if (t >= tend) {   // only intersect back
            return shadowTraverse(d_triangles, d_triIndices, d_leafIndices, d_kdInfo, rayO, rayD, nodeNum * 2 + 2, numNodes, tstart, tend);   // back
        }
        else {
            GPUHit hit = shadowTraverse(d_triangles, d_triIndices, d_leafIndices, d_kdInfo, rayO, rayD, nodeNum * 2 + 2, numNodes, tstart, t);    // back
            if (hit.t <= t) return hit;
            else return shadowTraverse(d_triangles, d_triIndices, d_leafIndices, d_kdInfo, rayO, rayD, nodeNum * 2 + 1, numNodes, t, tend);   // front
        }
    }
    else {
        if (t <= tstart) {
            return shadowTraverse(d_triangles, d_triIndices, d_leafIndices, d_kdInfo, rayO, rayD, nodeNum * 2 + 2, numNodes, tstart, tend);    // back
        }
        else if (t >= tend) {
            return shadowTraverse(d_triangles, d_triIndices, d_leafIndices, d_kdInfo, rayO, rayD, nodeNum * 2 + 1, numNodes, tstart, tend);   // front
        }
        else {
            GPUHit hit = shadowTraverse(d_triangles, d_triIndices, d_leafIndices, d_kdInfo, rayO, rayD, nodeNum * 2 + 1, numNodes, tstart, t);    // front
            if (hit.t <= t) return hit;
            else return shadowTraverse(d_triangles, d_triIndices, d_leafIndices, d_kdInfo, rayO, rayD, nodeNum * 2 + 2, numNodes, t, tend);    // back
        }
    }
}

// naive trace function
__device__ float3 trace(float3* d_triangles, float3 rayO, float3 rayD, int numTris, float3 lightPos, float3 lightCol) {

    float s = 512.0f;  // shininess for specular lighting
    float magnitude = 1.0f; // keeps track of how much the color of the next hit object will contribute to the total color
    float3 col = { 0.0f,0.0f,0.0f };
    float3 rayOr = rayO;
    float3 rayDir = rayD;

    for (int i = 0; i < 10; i++) {
        int triangleHit = NULL;

        GPUHit min_hit(__builtin_huge_valf(), float3{});

        for (int i = 0; i < numTris; i++) {
            GPUHit hit(intersect(d_triangles, i * 5, rayOr, rayDir));
            if (hit.t < min_hit.t) {
                min_hit = hit;
            }
        }

        triangleHit = min_hit.hitobject;    // contains index of STARTING POSITION of the triangle data in d_triangles array

        if (triangleHit == -1) break;  // BGColor




        // Phong Model

        float3 hitPoint = rayOr + rayDir * min_hit.t; // coords of hit point
        float3 N = min_hit.N;   // normal of hit object
        float3 L = norm(lightPos - hitPoint);  // vector from light to hit point
        float3 V = rayDir * -1.0f;  // view direction
        float3 R = reflect(L * -1.0f, N);   // light reflect direction


        // shading 
        float Iambi = 0.1f; // intensity of ambient lighting
        float Idiff;    // intensity of diffuse lighting at hit point
        float Ispec;    // intensity of specular lighting at hit point

        float matDiff = d_triangles[triangleHit + 4].x;       // material attribute diffuse
        float matSpec = d_triangles[triangleHit + 4].y;       // material attribute specular, 1.0 = mirror
        float matRefrIdx = d_triangles[triangleHit + 4].z;    // material attribute refractive index


        // REFRACTION
        if (matRefrIdx > 0) {
            float cosi = dot(N, rayDir);
            float eta = matRefrIdx;
            float3 n = N;
            if (cosi < 0) { cosi = -cosi; eta = 1 / matRefrIdx; }
            else { n = N * -1; }
            if (cosi > 1) cosi = 1;
            float k = 1 - eta * eta * (1 - cosi * cosi);
            if (k < 0) {
                rayDir = norm(reflect(V * -1.0f, N));
                rayOr = hitPoint + rayDir * 0.01f;
            }
            else {
                rayDir = eta * rayDir + (eta * cosi - sqrtf(k)) * n;
                rayOr = hitPoint + rayDir * 0.01f;
            }
        }
        else {
            if (matSpec < 1.0f) {   // not a mirror
                // shadow rays
                min_hit.t = __builtin_huge_valf();
                for (int i = 0; i < numTris; i++) {
                    GPUHit hit(intersect(d_triangles, i * 5, hitPoint + L * 0.01f, L));
                    if (hit.t < min_hit.t && d_triangles[hit.hitobject + 4].z == 0) {
                        min_hit = hit;
                        break;
                    }
                }
                if (min_hit.t != __builtin_huge_valf()) { Idiff = 0; Ispec = 0; }
                else { Idiff = absf(dot(N, L)); Ispec = pow(maxf(0.0f, dot(V, R)), s); }

                float3 ambient = Iambi * lightCol;
                float3 diffuse = Idiff * matDiff * lightCol;
                float3 specular = Ispec * matSpec * lightCol;
                col = col + magnitude * d_triangles[triangleHit + 3] * (diffuse + ambient + specular);
            }
            else {  // mirror
                Ispec = pow(maxf(0.0f, dot(V, R)), s);
                col = col + magnitude * Ispec * matSpec * lightCol;
            }
            if (matSpec > 0.0f) {
                rayDir = norm(reflect(V * -1, N));
                rayOr = hitPoint + rayDir * 0.01f;
                magnitude *= matSpec;
            }
            else break;
        }
    }
    return col;
}


// trace function with kd tree traversal
__device__ float3 kdTrace(float3* d_triangles, int* d_triIndices, int3* d_leafIndices, float3* d_kdInfo, float3 rayO, float3 rayD, int numNodes, float3 lightPos, float3 lightCol) {

    float s = 512.0f;  // shininess for specular lighting
    float magnitude = 1.0f; // keeps track of how much the color of the next hit object will contribute to the total color
    float3 col = { 0.0f,0.0f,0.0f };
    float3 rayOr = rayO;
    float3 rayDir = rayD;

    for (int i = 0; i < 10; i++) { // bouncing rays around iteratively rather than recursively to avoid register spilling
        int triangleHit = NULL;

        GPUHit min_hit(__builtin_huge_valf(), float3{});

        min_hit = cudaTraverse(d_triangles, d_triIndices, d_leafIndices, d_kdInfo, rayOr, rayDir, 0, numNodes, 0.0f, 0.0f);

        triangleHit = min_hit.hitobject;    // contains index of STARTING POSITION of the triangle data in d_triangles array

        if (triangleHit == -1) break;  // hit background

        // Phong Model

        float3 hitPoint = rayOr + rayDir * min_hit.t; // coords of hit point
        float3 N = min_hit.N;   // normal of hit object
        float3 L = norm(lightPos - hitPoint);  // vector from hit point to light
        float3 V = rayDir * -1.0f;  // view direction
        float3 R = reflect(L * -1.0f, N);   // light reflect direction

        // shading 
        float Iambi = 0.1f; // intensity of ambient lighting
        float Idiff;    // intensity of diffuse lighting at hit point
        float Ispec;    // intensity of specular lighting at hit point

        float matDiff = d_triangles[triangleHit + 4].x;       // material attribute diffuse
        float matSpec = d_triangles[triangleHit + 4].y;       // material attribute specular, 1.0 = mirror
        float matRefrIdx = d_triangles[triangleHit + 4].z;    // material attribute refractive index

        // REFRACTION
        if (matRefrIdx > 0) {
            float cosi = dot(N, rayDir);
            float eta = matRefrIdx;
            float3 n = N;
            if (cosi < 0) { cosi = -cosi; eta = 1 / matRefrIdx; }
            else { n = N * -1; }
            if (cosi > 1) cosi = 1;
            float k = 1 - eta * eta * (1 - cosi * cosi);
            if (k < 0) {
                rayDir = norm(reflect(V * -1.0f, N));
                rayOr = hitPoint + rayDir * 0.01f;
            }
            else {
                rayDir = eta * rayDir + (eta * cosi - sqrtf(k)) * n;
                rayOr = hitPoint + rayDir * 0.01f;
            }
        }
        else {
            if (matSpec < 1.0f) {   // not a mirror
                // shadow rays
                if (shadowTraverse(d_triangles, d_triIndices, d_leafIndices, d_kdInfo, hitPoint + L * 0.01f, L, 0, numNodes, 0, 0).hitobject != -1) { Idiff = 0; Ispec = 0; }
                else { Idiff = absf(dot(N, L)); Ispec = pow(maxf(0.0f, dot(V, R)), s); }
                float3 ambient = Iambi * lightCol;
                float3 diffuse = Idiff * matDiff * lightCol;
                float3 specular = Ispec * matSpec * lightCol;
                col = col + magnitude * d_triangles[triangleHit + 3] * (diffuse + ambient + specular);
            }
            else {  // mirror
                Ispec = pow(maxf(0.0f, dot(V, R)), s);
                col = col + magnitude * Ispec * matSpec * lightCol;
            }
            if (matSpec > 0.0f) {   // bouncing rays
                rayDir = norm(reflect(V * -1.0f, N));
                rayOr = hitPoint + rayDir * 0.01f;
                magnitude *= matSpec;
            }
            else break; // no bounce because object is opaque
        }
    }

    return col;

}



// NAIVE KERNEL
__global__ void cudaNaiveTraceKernel(float3* d_triangles, int numTris, float3* d_imgArr, int imgSize, int imgWidth, float3 origin, float3 right, float3 up, float3 eye, float3 lightPos, float3 lightCol) {

    int thid = blockIdx.x * blockDim.x + threadIdx.x;   // thread ID 
    int numth = gridDim.x * blockDim.x;                 // total number of threads

    int x;
    int y;


    // each pixel gets a thread computing its color
    for (int i = thid; i < imgSize; i += numth) {
        x = i % imgWidth;
        y = i / imgWidth;


        float3 total = { 0.0f,0.0f,0.0f };

        // pixel
        float3 pixel;

        // ray
        float3 rayO = eye;
        float3 rayD;


        // super sampling / anti aliasing
        int aa = 1; // aa^2 = numSamples
        for (int i = 1; i <= aa; i++) {
            for (int j = 1; j <= aa; j++) {
                pixel = origin + right * (x - 0.5f + i * 1.0f / (aa + 1)) - up * (y - 0.5f + j * 1.0f / (aa + 1));
                rayD = norm(pixel - eye);
                total = total + trace(d_triangles, rayO, rayD, numTris, lightPos, lightCol);
            }
        }
        total = total / (aa * aa);


        clampCol(total);

        d_imgArr[i] = total;
        

    }
}

// KD KERNEL
__global__ void cudaKDTraceKernel(float3* d_triangles, float3* d_imgArr, int* d_triIndices, int3* d_leafIndices, float3* d_kdInfo, int numNodes, int imgSize, int imgWidth, float3 origin, float3 right, float3 up, float3 eye, float3 lightPos, float3 lightCol) {

    int thid = blockIdx.x * blockDim.x + threadIdx.x;   // thread ID 
    int numth = gridDim.x * blockDim.x;                 // total number of threads

    int x;
    int y;

    for (int i = thid; i < imgSize; i += numth) {
        x = i % imgWidth;
        y = i / imgWidth;


        float3 total = { 0.0f,0.0f,0.0f };

        // pixel
        float3 pixel;

        // ray
        float3 rayO = eye;
        float3 rayD;


        // super sampling / anti aliasing
        int aa = 1; // aa^2 = numSamples
        for (int i = 1; i <= aa; i++) {
            for (int j = 1; j <= aa; j++) {
                pixel = origin + right * (x - 0.5f + i * 1.0f / (aa + 1)) - up * (y - 0.5f + j * 1.0f / (aa + 1));
                rayD = norm(pixel - eye);
                total = total + kdTrace(d_triangles, d_triIndices, d_leafIndices, d_kdInfo, rayO, rayD, numNodes, lightPos, lightCol);
            }
        }
        total = total / (aa*aa);

        clampCol(total);

        d_imgArr[i] = total;


    }
}


// naive cuda tracing
void cudaNaiveTrace(Triangle* allTriangles, Image& img, int numTris) {
    
    // performance metrics setup
    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);
    float time;
    unsigned int inBytes = numTris * 5  * sizeof(float3);
    unsigned int outBytes = img.size() * sizeof(float3);


    // lights
    float3 lightPos = { -100.0f, 200.0f, 1200.0f };
    float3 lightCol = { 1.0f, 1.0f, 0.8f };
    
    int w = img.width();
    int h = img.height();
    
    // camera
    const float3 eye = { 300.0f, 0.0f, 1000.0f };
    const float3 center = { 0.0f, 0.0f, -200.0f };
    const float3 forward = norm(center - eye);
    const float3 up = { 0.0f, 1.0f, 0.0f };
    const float3 right = norm(cross(forward, up));

    const float3 origin = center - right * (w / 2.0f) + up * (h / 2.0f - 1.0f);


    int imgSize = img.size();

    float3* imgArr; // this array contains all the pixels' colors on the host, size = imgSize
    float3* d_imgArr;   // this array contains all the pixels' colors on the device, size = imgSize
    float3* triangles;   
    float3* d_triangles;    // this array contains all the triangles, size = 4 * numTriangles because it stores the 3 points and its color


    // MEMORY ALLOCATION

    //cudaEventRecord(start);

    cudaMallocHost(&imgArr, imgSize * sizeof(float3));
    cudaMalloc(&d_imgArr, imgSize * sizeof(float3));    // 1920*1080 resolution: 6MB
    cudaMallocHost(&triangles, numTris * 5 * sizeof(float3));
    cudaMalloc(&d_triangles, numTris * 5 * sizeof(float3));   // 3 floats for point A, 3 for point B, 3 for point C, 3 for color (~24MB for 250000 triangles)
    
    /*cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&time, start, stop);
    std::cout << "cudaMalloc time: " << time << "ms" << std::endl;*/

    // DATA TYPE TRANSFORMATION
    auto timerS = high_resolution_clock::now();
    for (int i = 0; i < numTris; i++) {
        triangles[5 * i] = float3{ allTriangles[i].a.x, allTriangles[i].a.y, allTriangles[i].a.z };
        triangles[5 * i + 1] = float3{ allTriangles[i].b.x, allTriangles[i].b.y, allTriangles[i].b.z };
        triangles[5 * i + 2] = float3{ allTriangles[i].c.x, allTriangles[i].c.y, allTriangles[i].c.z };
        triangles[5 * i + 3] = float3{ allTriangles[i].color.x, allTriangles[i].color.y, allTriangles[i].color.z };
        triangles[5 * i + 4] = float3{ allTriangles[i].matDiff, allTriangles[i].matSpec, allTriangles[i].refrIdx };
    }
    auto timerE = high_resolution_clock::now();
    duration<double, std::milli> timerR = timerE - timerS;
    std::cout << "triangle transform time: " << timerR.count() << "ms" << std::endl;


    // MEMCOPY HOST TO DEVICE
    cudaEventRecord(start);

    //cudaMemcpy(d_imgArr, imgArr, imgSize * sizeof(float3), cudaMemcpyHostToDevice); CUERR
    cudaMemcpy(d_triangles, triangles, numTris * 5 * sizeof(float3), cudaMemcpyHostToDevice); CUERR

    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&time, start, stop);
    std::cout << "MemcpyH2D time: " << time << "ms\nBytes copied: " << inBytes << "\nBandwidth: " << inBytes * 1e-6 / time << "GB/s\n" << std::endl;


    // KERNEL CALL
    cudaEventRecord(start);

    cudaNaiveTraceKernel << <32768, 64 >> > (d_triangles, numTris, d_imgArr, imgSize, w, origin, right, up, eye, lightPos, lightCol); CUERR

    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&time, start, stop);
    std::cout << "Kernel time: " << time << "ms\n" << std::endl;


    // MEMCOPY DEVICE TO HOST
    cudaEventRecord(start);

    cudaMemcpy(imgArr, d_imgArr, imgSize * sizeof(float3), cudaMemcpyDeviceToHost); CUERR
    
    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&time, start, stop);
    std::cout << "MemcpyD2H time: " << time << "ms\nBytes copied: " << outBytes << "\nBandwidth: " << outBytes * 1e-6 / time << "GB/s\n" << std::endl;

    
    // writing pixel colors back to the image object
    auto writeBackStart = high_resolution_clock::now();
    for (int i = 0; i < imgSize; i++) img(i % w, i / w) = Color(imgArr[i].x, imgArr[i].y, imgArr[i].z); // i is the pixel number, i%w is the pixel's x coordinate whereas i/w is the pixel's y coordinate
    auto writeBackEnd = high_resolution_clock::now();
    duration<double, std::milli> writeBackTime = writeBackEnd - writeBackStart;
    std::cout << "Writeback time: " << writeBackTime.count() << "ms" << std::endl;


    //
    // CLEAN UP
    //
    cudaFree(d_imgArr);
    cudaFree(d_triangles);
    cudaFreeHost(imgArr);
    cudaFreeHost(triangles);
    cudaEventDestroy(start);
    cudaEventDestroy(stop);
    
}

// CUDA-accelerated ray tracing using KD-Trees
void cudaKDTrace(KDNode* tree, Image& img, Triangle* allTriangles, int* triangleIndices, int numNodes, int numTris, int numTriIndices, Point eye_, Point center_) {

    // performance metrics setup
    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);
    float time;
    unsigned int inBytes = (numTris * 5 + numNodes * 3) * sizeof(float3) + numTriIndices * sizeof(int) + numNodes * sizeof(int3);
    unsigned int outBytes = img.size() * sizeof(float3);
    
    cudaDeviceSetLimit(cudaLimitStackSize, 4096);
    size_t stackSize;
    cudaDeviceGetLimit(&stackSize, cudaLimitStackSize);


    // lights
    float3 lightPos = { -100.0f, 200.0f, 1200.0f };
    float3 lightCol = { 1.0f, 1.0f, 0.8f };

    int w = img.width();
    int h = img.height();

    // camera
    float3 eye = { eye_.x, eye_.y, eye_.z };
    float3 center = { center_.x, center_.y, center_.z };
    const float3 forward = norm(center - eye);
    const float3 up = { 0.0f, 1.0f, 0.0f };
    const float3 right = norm(cross(forward, up));

    const float3 origin = center - right * (w / 2.0f) + up * (h / 2.0f - 1.0f);

    int imgSize = img.size();


    // Image Pixels
    float3* imgArr; // this array contains all the pixels' colors on the host, size = imgSize
    float3* d_imgArr;   // this array contains all the pixels' colors on the device, size = imgSize

    // All Triangles
    float3* triangles;
    float3* d_triangles;    // this array contains all the triangles, size = 4 * numTriangles because it stores the 3 points and its color

    // Triangle Indices
    int* h_triIndices;
    int* d_triIndices;  // contains the indices used to access the triangles from the main array for every kdnode

    // KDNode Information
    float3* kdInfo;     // contains bounds (6 floats), tstart, tend and splitDist for each node
    int3* leafIndices;   // contains splitDim and triangleStart and end indices for each node
    float3* d_kdInfo;   // floats on device
    int3* d_leafIndices;    // splitDim and indices on device, use d_triangles[d_triIndices[d_leafIndices.y+i]] to access a specific triangle

    //
    // MEMORY ALLOCATION
    //

    //cudaEventRecord(start);

    // Image Pixels
    cudaMallocHost(&imgArr, imgSize * sizeof(float3));
    cudaMalloc(&d_imgArr, imgSize * sizeof(float3));    // 1920*1080 resolution: 6MB

    // All Triangles
    cudaMallocHost(&triangles, numTris * 5 * sizeof(float3));
    cudaMalloc(&d_triangles, numTris * 5 * sizeof(float3));   // 3 floats for point A, 3 for point B, 3 for point C, 3 for color (~24MB for 250000 triangles)
    
    // Triangle Indices
    cudaMallocHost(&h_triIndices, numTriIndices * sizeof(int));
    cudaMalloc(&d_triIndices, numTriIndices * sizeof(int));

    // KDNode Information
    cudaMallocHost(&kdInfo, numNodes * 3 * sizeof(float3)); // total of 9 floats per kdnode
    cudaMallocHost(&leafIndices, numNodes * sizeof(int3));  // total of 3 ints per kdnode
    cudaMalloc(&d_kdInfo, numNodes * 3 * sizeof(float3));
    cudaMalloc(&d_leafIndices, numNodes * sizeof(int3));

    /*cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&time, start, stop);
    std::cout << "cudaMalloc time: " << time << "ms" << std::endl;*/

    //
    // DATA TYPE TRANSFORMATION
    //
    auto timerS = high_resolution_clock::now();
    // Triangle Transformation
    for (int i = 0; i < numTris; i++) {
        triangles[5 * i] = float3{ allTriangles[i].a.x, allTriangles[i].a.y, allTriangles[i].a.z };
        triangles[5 * i + 1] = float3{ allTriangles[i].b.x, allTriangles[i].b.y, allTriangles[i].b.z };
        triangles[5 * i + 2] = float3{ allTriangles[i].c.x, allTriangles[i].c.y, allTriangles[i].c.z };
        triangles[5 * i + 3] = float3{ allTriangles[i].color.x, allTriangles[i].color.y, allTriangles[i].color.z };
        triangles[5 * i + 4] = float3{ allTriangles[i].matDiff, allTriangles[i].matSpec, allTriangles[i].refrIdx };
    }

    // Triangle Indices
    for (int i = 0; i < numTriIndices; i++) h_triIndices[i] = triangleIndices[i];

    //KD-Tree Transformation
    for (int i = 0; i < numNodes; i++) {
        kdInfo[3 * i] = float3{ tree[i].min_x, tree[i].max_x, tree[i].min_y };
        kdInfo[3 * i + 1] = float3{ tree[i].max_y, tree[i].min_z, tree[i].max_z };
        kdInfo[3 * i + 2] = float3{ tree[i].splitDist, tree[i].tstart, tree[i].tend };
        leafIndices[i] = int3{ tree[i].splitDim, tree[i].triangleStart, tree[i].triangleEnd };
    }

    auto timerE = high_resolution_clock::now();
    duration<double, std::milli> timerR = timerE - timerS;
    std::cout << "triangle & kd tree transform time: " << timerR.count() << "ms" << std::endl;

    //
    // MEMCOPY HOST TO DEVICE
    //
    cudaEventRecord(start);

    //cudaMemcpy(d_imgArr, imgArr, imgSize * sizeof(float3), cudaMemcpyHostToDevice); CUERR
    cudaMemcpy(d_triangles, triangles, numTris * 5 * sizeof(float3), cudaMemcpyHostToDevice); CUERR
    cudaMemcpy(d_triIndices, h_triIndices, numTriIndices * sizeof(int), cudaMemcpyHostToDevice); CUERR
    cudaMemcpy(d_leafIndices, leafIndices, numNodes * sizeof(int3), cudaMemcpyHostToDevice); CUERR
    cudaMemcpy(d_kdInfo, kdInfo, numNodes * 3 * sizeof(float3), cudaMemcpyHostToDevice); CUERR

    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&time, start, stop);
    std::cout << "MemcpyH2D time: " << time << "ms\nBytes copied: " << inBytes << "\nBandwidth: " << inBytes * 1e-6 / time << "GB/s\n" << std::endl;

    //
    // KERNEL CALL
    //
    cudaEventRecord(start);

    cudaKDTraceKernel << <32768, 64 >> > (d_triangles, d_imgArr, d_triIndices, d_leafIndices, d_kdInfo, numNodes, imgSize, w, origin, right, up, eye, lightPos, lightCol);

    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&time, start, stop);
    std::cout << "Kernel time: " << time << "ms\n" << std::endl;

    //
    // MEMCOPY DEVICE TO HOST
    // 
    cudaEventRecord(start);

    cudaMemcpy(imgArr, d_imgArr, imgSize * sizeof(float3), cudaMemcpyDeviceToHost); CUERR

    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&time, start, stop);
    std::cout << "MemcpyD2H time: " << time << "ms\nBytes copied: " << outBytes << "\nBandwidth: " << outBytes * 1e-6 / time << "GB/s\n" << std::endl;


    // writing pixel colors back to the image object
    auto writeBackStart = high_resolution_clock::now();
    for (int i = 0; i < imgSize; i++) img(i % w, i / w) = Color(imgArr[i].x, imgArr[i].y, imgArr[i].z); // i is the pixel number, i%w is the pixel's x coordinate whereas i/w is the pixel's y coordinate
    auto writeBackEnd = high_resolution_clock::now();
    duration<double, std::milli> writeBackTime = writeBackEnd - writeBackStart;
    std::cout << "Writeback time: " << writeBackTime.count() << "ms" << std::endl;

    //
    // CLEAN UP
    //
    cudaFree(d_imgArr);
    cudaFree(d_triangles);
    cudaFree(d_triIndices);
    cudaFree(d_kdInfo);
    cudaFree(d_leafIndices);
    cudaFreeHost(imgArr);
    cudaFreeHost(triangles);
    cudaFreeHost(h_triIndices);
    cudaFreeHost(kdInfo);
    cudaFreeHost(leafIndices);
    cudaEventDestroy(start);
    cudaEventDestroy(stop);


    return;
}
