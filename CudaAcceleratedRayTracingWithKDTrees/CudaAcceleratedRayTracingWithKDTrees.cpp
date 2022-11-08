// CudaAcceleratedRayTracingWithKDTrees.cpp: Definiert den Einstiegspunkt für die Anwendung.
//

#include "CudaAcceleratedRayTracingWithKDTrees.h"
#include <iostream>
#include <chrono>
#include "CudaRaytracing/CudaRaytracing.h"
#include "objparser.h"
#include "hit.h"
#include "ray.h"
#include "light.h"
#include "kdnode.h"
#include "vector.h"
#include "triangle.h"
#include "image.h"

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

static unsigned long long int raycount = 0;
static bool useKD = true;
static bool useCUDA = true;
static bool debug = false;

static int sd = 13; // number of splits/levels to build the kd tree with, gets converted to splitDim for circular splitting
static int recDepth = 10;   // maximum recursion depth for reflected rays

static KDNode root(-1000, 1000, -1000, 1000, -1000, 1000);

static Light light(Point(-100, 200, 1200), Color(1.0f, 1.0f, 0.8f));


Color trace(Ray ray, std::vector<Triangle>& triangles, int depth) {    // second argument takes vector storing all triangles

    raycount++;

    Triangle* triangleHit = NULL;

    Hit min_hit(std::numeric_limits<float>::infinity(), Vector());


    if (!useKD) {
        for (unsigned int i = 0; i < triangles.size(); ++i) {
            Hit hit(triangles.at(i).intersect(ray));
            if (hit.t < min_hit.t) {
                min_hit = hit;
            }
        }
    }
    else {
        min_hit = root.traverse(triangles, ray);
    }

    triangleHit = min_hit.hitobject;

    if (!triangleHit) return Color(0.0, 0.0, 0.0);  // BGColor
    if (debug) return triangleHit->color;   // for debugging - no shading


    // Phong Model

    float s = 512;  // shininess for specular lighting

    Point hitPoint = ray.at(min_hit.t); // coords of hit point
    Vector N = min_hit.N;   // normal of hit object
    Vector L = (light.position - hitPoint).normalized();  // vector from light to hit point
    Vector V = -ray.D;  // view direction
    Vector R = -L.reflect(N);   // light reflect direction

    // shading
    float Iambi = 0.1; // intensity of ambient lighting
    float Idiff;    // intensity of diffuse lighting at hit point
    float Ispec;    // intensity of specular lighting at hit point

    float matDiff = triangleHit->matDiff;   // material attribute diffuse
    float matSpec = triangleHit->matSpec;   // material attribute specular, 1.0 = mirror
    float matRefrIdx = triangleHit->refrIdx;   // material attribute refractive index

    Color col(0.0f, 0.0f, 0.0f);

    // REFRACTION
    if (matRefrIdx > 0 && depth < recDepth) {
        float cosi = N.dot(ray.D);
        float eta = matRefrIdx;
        Vector n = N;
        if (cosi < 0) { cosi = -cosi; eta = 1 / matRefrIdx; }
        else { n = -N; }
        if (cosi > 1) cosi = 1;
        float k = 1 - eta * eta * (1 - cosi * cosi);
        if (k < 0) {
            Vector bounceDir = -V.reflect(N).normalized();
            ray = Ray(hitPoint + bounceDir * 0.01f, bounceDir);
            col += trace(ray, triangles, depth + 1);
        }
        else {
            Vector refrDir = eta * ray.D + (eta * cosi - sqrtf(k)) * n;
            ray = Ray(hitPoint + refrDir * 0.01f, refrDir);
            col += trace(ray, triangles, depth + 1);
        }
    }
    else {
        if (matSpec < 1.0f && depth < recDepth) {  // don't need to compute the object's diffuse and ambient color if it's a 100% reflective mirror
            // shadow rays
            raycount++;
            min_hit.t = std::numeric_limits<float>::infinity();
            ray = Ray(hitPoint + L * 0.01f, L);
            if (!useKD) {
                for (unsigned int i = 0; i < triangles.size(); ++i) {
                    Hit hit(triangles.at(i).intersect(ray));
                    if (hit.t < min_hit.t && hit.hitobject->refrIdx == 0) {
                        min_hit = hit;
                        break;
                    }
                }
            }
            else {
                min_hit = root.shadTraverse(triangles, ray);
            }

            if (min_hit.t != std::numeric_limits<float>::infinity()) { Idiff = 0; Ispec = 0; }
            else { Idiff = std::fabs(N.dot(L)); Ispec = pow(std::fmax(0.0, V.dot(R)), s); }

            Color ambient = Iambi * light.color;
            Color diffuse = Idiff * matDiff * light.color;
            Color specular = Ispec * matSpec * light.color;

            col = triangleHit->color * (diffuse + ambient + specular);
        }
        else {
            Ispec = pow(std::fmax(0.0, V.dot(R)), s);
            col = Ispec * matSpec * light.color;
        }
        if (matSpec > 0.0f && depth < recDepth) {   // bouncing rays
            Vector bounceDir = -V.reflect(N).normalized();
            ray = Ray(hitPoint + bounceDir * 0.01f, bounceDir);
            col += matSpec * trace(ray, triangles, depth + 1);
        }
    }
    return col;
}

Color tripleIndexTest(Ray ray, std::vector<Triangle>& triangles, std::vector<int>& leaftris) {    // second argument takes vector storing all triangles

    Triangle* triangleHit = NULL;

    Hit min_hit(std::numeric_limits<float>::infinity(), Vector());


    if (!useKD) {


        for (unsigned int i = 0; i < triangles.size(); ++i) {
            Hit hit(triangles.at(i).intersect(ray));
            if (hit.t < min_hit.t) {
                min_hit = hit;
            }
        }

    }
    else {
        min_hit = root.tripleIndexTraverse(triangles, ray, leaftris);
    }

    triangleHit = min_hit.hitobject;



    if (!triangleHit) return Color(0.0, 0.0, 0.0);  // BGColor
    if (debug) return triangleHit->color;   // for debugging - no shading



    // Phong Shading

    float s = 32;  // shininess for specular lighting

    Point hitPoint = ray.at(min_hit.t); // coords of hit point

    Vector N = min_hit.N;   // normal of hit object

    Vector L = (light.position - hitPoint).normalized();  // vector from light to hit point

    Vector V = -ray.D;  // view direction

    Vector R = -L.reflect(N);   // light reflect direction


    float Iambi = 0.1; // intensity of ambient lighting

    float Idiff = std::fabs(N.dot(L)); // intensity of diffuse lighting at hit point

    float Ispec = pow(std::fmax(0.0, V.dot(R)), s);    // intensity of specular lighting at hit point


    Color ambient = Iambi * light.color;

    Color diffuse = Idiff * 1.0 * light.color;  // replace 1.0 with material attribute for diffuse lighting later

    Color specular = Ispec * 0.2 * light.color; // replace 0.2 with material attribute for specular reflection later

    return triangleHit->color * (diffuse + ambient + specular);
}


int main()
{
  
    // getting all triangles from the .obj files
    std::cout << "Parsing OBJs ..." << std::endl;
    std::vector<Triangle> triangles;
    std::vector<Triangle> dragon = OBJParser::parseOBJ("../../dragon.txt");
    std::vector<Triangle> armadillo = OBJParser::parseOBJ("../../armadillo.txt");
    std::vector<Triangle> buddha = OBJParser::parseOBJ("../../buddha.txt");
    std::vector<Triangle> teapot = OBJParser::parseOBJ("../../teapot.txt");
    std::vector<Triangle> blueDragon = dragon;
    std::vector<Triangle> shinyGoldDragon = dragon;
    std::vector<Triangle> secondBuddha = buddha;
    std::vector<Triangle> glasspot = teapot;
    std::cout << "DONE." << std::endl;

    // transform objects and put them in one vector
    std::cout << "Building Scene: " << std::endl;

    // environment
    Triangle reflBackgr1(Point(-931.0f, -170.0f, -999.0f), Point(930.0f, -170.0f, -999.0f), Point(929.0f, 510.0f, -999.0f), Color(1.0f, 1.0f, 0.0f), 0.0f, 1.0f, 0.0f);
    Triangle reflBackgr2(Point(-929.0f, -170.0f, -999.0f), Point(931.0f, 510.0f, -999.0f), Point(-930.0f, 510.0f, -999.0f), Color(1.0f, 1.0f, 0.0f), 0.0f, 1.0f, 0.0f);
    Triangle reflBorder1(Point(-961.0f, -201.0f, -1000.0f), Point(960.0f, -201.0f, -1000.0f), Point(959.0f, 540.0f, -1000.0f), Color(0.39608f, 0.26275f, 0.12941f), 1.0f, 0.0f, 0.0f);
    Triangle reflBorder2(Point(-959.0f, -201.0f, -1000.0f), Point(961.0f, 540.0f, -1000.0f), Point(-960.0f, 540.0f, -1000.0f), Color(0.39608f, 0.26275f, 0.12941f), 1.0f, 0.0f, 0.0f);
    Triangle ground1(Point(-961.0f, -200.0f, 1100.0f), Point(960.0f, -200.0f, 1100.0f), Point(959.0f, -200.0f, -1000.0f), Color(0.0f, 1.0f, 0.0f));
    Triangle ground2(Point(-959.0f, -200.0f, 1100.0f), Point(961.0f, -200.0f, -1000.0f), Point(-960.0f, -200.0f, -1000.0f), Color(0.0f, 1.0f, 0.0f));

    std::vector<Triangle> env = { reflBackgr1, reflBackgr2, reflBorder1, reflBorder2, ground1, ground2 };

    triangles.reserve(dragon.size() + blueDragon.size() + shinyGoldDragon.size() + armadillo.size() + teapot.size() + glasspot.size() + buddha.size() + secondBuddha.size() + env.size());

    // transform dragon
    for (int i = 0; i < dragon.size(); i++) {
        dragon[i].a *= 4;
        dragon[i].b *= 4;
        dragon[i].c *= 4;
        dragon[i].a.x -= 500;
        dragon[i].b.x -= 500;
        dragon[i].c.x -= 500;
        dragon[i].a.y -= 40;
        dragon[i].b.y -= 40;
        dragon[i].c.y -= 40;
        dragon[i].a.z -= 100;
        dragon[i].b.z -= 100;
        dragon[i].c.z -= 100;
    }
    // transform blue dragon
    for (int i = 0; i < blueDragon.size(); i++) {
        blueDragon[i].a *= 1;
        blueDragon[i].b *= 1;
        blueDragon[i].c *= 1;
        blueDragon[i].a.x -= 100;
        blueDragon[i].b.x -= 100;
        blueDragon[i].c.x -= 100;
        blueDragon[i].a.y -= 160;
        blueDragon[i].b.y -= 160;
        blueDragon[i].c.y -= 160;
        blueDragon[i].a.z += 350;
        blueDragon[i].b.z += 350;
        blueDragon[i].c.z += 350;
        blueDragon[i].color = Color(0.67843f, 0.84706f, 0.902f);
    }
    // transform gold dragon
    for (int i = 0; i < shinyGoldDragon.size(); i++) {
        shinyGoldDragon[i].a *= 1;
        shinyGoldDragon[i].b *= 1;
        shinyGoldDragon[i].c *= 1;
        shinyGoldDragon[i].a.x += 350;
        shinyGoldDragon[i].b.x += 350;
        shinyGoldDragon[i].c.x += 350;
        shinyGoldDragon[i].a.y -= 160;
        shinyGoldDragon[i].b.y -= 160;
        shinyGoldDragon[i].c.y -= 160;
        shinyGoldDragon[i].a.z += 350;
        shinyGoldDragon[i].b.z += 350;
        shinyGoldDragon[i].c.z += 350;
        shinyGoldDragon[i].color = Color(1.0f, 0.843f, 0.0f);
        shinyGoldDragon[i].matSpec = 0.4f;
    }
    // transform armadillo
    for (int i = 0; i < armadillo.size(); i++) {
        armadillo[i].a *= 4;
        armadillo[i].b *= 4;
        armadillo[i].c *= 4;
        armadillo[i].a.x += 450;
        armadillo[i].b.x += 450;
        armadillo[i].c.x += 450;
        armadillo[i].a.y += 16;
        armadillo[i].b.y += 16;
        armadillo[i].c.y += 16;
        armadillo[i].a.z -= 200;
        armadillo[i].b.z -= 200;
        armadillo[i].c.z -= 200;
    }
    // transform buddha
    for (int i = 0; i < buddha.size(); i++) {
        buddha[i].a *= 2000;
        buddha[i].b *= 2000;
        buddha[i].c *= 2000;
        buddha[i].a.x += 70;
        buddha[i].b.x += 70;
        buddha[i].c.x += 70;
        buddha[i].a.y -= 300;
        buddha[i].b.y -= 300;
        buddha[i].c.y -= 300;
        buddha[i].a.z += 200;
        buddha[i].b.z += 200;
        buddha[i].c.z += 200;
        buddha[i].color = Color(1.0f, 0.843f, 0.0f);
        buddha[i].matSpec = 0.2f;
    }
    // transform second buddha
    for (int i = 0; i < secondBuddha.size(); i++) {
        secondBuddha[i].a *= 500;
        secondBuddha[i].b *= 500;
        secondBuddha[i].c *= 500;
        secondBuddha[i].a.x += 70;
        secondBuddha[i].b.x += 70;
        secondBuddha[i].c.x += 70;
        secondBuddha[i].a.y -= 225;
        secondBuddha[i].b.y -= 225;
        secondBuddha[i].c.y -= 225;
        secondBuddha[i].a.z += 450;
        secondBuddha[i].b.z += 450;
        secondBuddha[i].c.z += 450;
        secondBuddha[i].color = Color(0.753f, 0.753f, 0.753f);
        secondBuddha[i].matSpec = 0.2f;
        //secondBuddha[i].refrIdx = 1.52;
    }
    // transform teapot
    for (int i = 0; i < teapot.size(); i++) {
        teapot[i].a *= 23;
        teapot[i].b *= 23;
        teapot[i].c *= 23;
        teapot[i].a.x -= 70;
        teapot[i].b.x -= 70;
        teapot[i].c.x -= 70;
        teapot[i].a.y -= 20;
        teapot[i].b.y -= 20;
        teapot[i].c.y -= 20;
        teapot[i].a.z -= 50;
        teapot[i].b.z -= 50;
        teapot[i].c.z -= 50;
        teapot[i].color = Color(0.6627f, 0.6627f, 0.6627f);
        teapot[i].matSpec = 0.3f;
    }
    // transform glass teapot
    for (int i = 0; i < glasspot.size(); i++) {
        glasspot[i].a *= 23;
        glasspot[i].b *= 23;
        glasspot[i].c *= 23;
        glasspot[i].a.x += 50;
        glasspot[i].b.x += 50;
        glasspot[i].c.x += 50;
        glasspot[i].a.y -= 20;
        glasspot[i].b.y -= 20;
        glasspot[i].c.y -= 20;
        glasspot[i].a.z += 350;
        glasspot[i].b.z += 350;
        glasspot[i].c.z += 350;
        glasspot[i].color = Color(0.0f, 1.0f, 1.0f);
        glasspot[i].matSpec = 0.2f;
        glasspot[i].refrIdx = 1.52f;
    }



    // add models and environment to the scene
    triangles.insert(triangles.end(), dragon.begin(), dragon.end());
    triangles.insert(triangles.end(), blueDragon.begin(), blueDragon.end());
    triangles.insert(triangles.end(), shinyGoldDragon.begin(), shinyGoldDragon.end());
    triangles.insert(triangles.end(), armadillo.begin(), armadillo.end());
    triangles.insert(triangles.end(), teapot.begin(), teapot.end());
    triangles.insert(triangles.end(), glasspot.begin(), glasspot.end());
    triangles.insert(triangles.end(), buddha.begin(), buddha.end());
    triangles.insert(triangles.end(), secondBuddha.begin(), secondBuddha.end());
    triangles.insert(triangles.end(), env.begin(), env.end());
    
    std::cout << "Total number of Triangles: " <<  triangles.size() << std::endl;

    // find root bounds
    root.getInitialBounds(triangles);

    // print root bounds
    root.printBounds();

    // initializing image object
    Image img(1920, 1080);

    int w = img.width();
    int h = img.height();

    std::cout << "Resolution: " << w << "x" << h << std::endl;

    // initializing camera
    Point eye = Point(0.0f, 0.0f, 1000.0f);
    Point center = Point(0.0f, 0.0f, -200.0f);
    const Vector forward = (center - eye).normalized();
    const Vector up = Vector(0, 1, 0);
    const Vector right = forward.cross(up).normalized();
    const Point origin = center - (w / 2.0) * right + (h / 2.0 - 1) * up;

    // initializing kd tree
    int treeSize = pow(2, sd + 1) - 1;
    std::vector<KDNode> tree(treeSize);

    // separate triangles array containing all the leaves' triangles which can be accessed via the indices stored in the corresponding node
    std::vector<int> leafTriangles;

    // CUDA ray tracing
    if (useCUDA) {
        auto traceWithCUDAStart = high_resolution_clock::now();
        cudaNaiveTrace(triangles.data(), img, triangles.size());
        auto traceWithCUDAEnd = high_resolution_clock::now();
        duration<double, std::milli> traceTimeWithCUDA = traceWithCUDAEnd - traceWithCUDAStart;

        std::cout << "Execution time (tracing with CUDA): " << traceTimeWithCUDA.count() << "ms" << std::endl;
        std::cout << "______________________________________________________________" << std::endl;

        img.write_png("../CUDA.png");
    }

    if (useKD) {

        //// building kd tree recursively

        //std::cout << "Building KD Tree...";
        //auto buildTreeStart = high_resolution_clock::now();
        //root.makeTree(triangles, sd);
        //auto buildTreeEnd = high_resolution_clock::now();
        //std::cout << " DONE." << std::endl;
        //duration<double, std::milli> buildTime = buildTreeEnd - buildTreeStart;
        //root.printTree(1);
        //std::cout << "Execution time (building kd tree): " << buildTime.count() << "ms" << std::endl;


        // building cuda compatible kd tree 

        std::cout << "Building CUDA-compatible KD Tree...";
        auto cudabuildTreeStart = high_resolution_clock::now();
        root.makeTreeCUDA(triangles, tree, leafTriangles, 1, sd);
        auto cudabuildTreeEnd = high_resolution_clock::now();
        std::cout << " DONE." << std::endl;
        duration<double, std::milli> cudabuildTime = cudabuildTreeEnd - cudabuildTreeStart;

        //DEBUG print KDTree
        //for (int i = 0; i < tree.size(); i++) std::cout << "Node with index " << i << " has " << tree[i].numTris << " triangles." << std::endl;

        std::cout << "Execution time (building cuda kd tree vector): " << cudabuildTime.count() << "ms" << std::endl;


        // CUDA-accelerated ray tracing using KD-Trees
        if (useCUDA) {
            auto traceWithCUDAKDStart = high_resolution_clock::now();
            cudaKDTrace(tree.data(), img, triangles.data(), leafTriangles.data(), treeSize, triangles.size(), leafTriangles.size(), eye, center);
            img.write_png("../CUDAKD2.png");
            //eye.x = -300.0f;
            //cudaKDTrace(tree.data(), img, triangles.data(), leafTriangles.data(), treeSize, triangles.size(), leafTriangles.size(), eye, center);
            //img.write_png("../CUDAKD1.png");
            //eye.x = 300.0f;
            //cudaKDTrace(tree.data(), img, triangles.data(), leafTriangles.data(), treeSize, triangles.size(), leafTriangles.size(), eye, center);
            //img.write_png("../CUDAKD3.png");
            auto traceWithCUDAKDEnd = high_resolution_clock::now();
            duration<double, std::milli> traceTimeWithCUDAKD = traceWithCUDAKDEnd - traceWithCUDAKDStart;

            std::cout << "Execution time (tracing with CUDA and KD tree): " << traceTimeWithCUDAKD.count() << "ms" << std::endl;
            std::cout << "______________________________________________________________" << std::endl;

        }


        // ray tracing with KD usage
        raycount = 0;
        auto traceWithKDStart = high_resolution_clock::now();
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                Color total;
                Point pixel;
                Ray ray(eye, (pixel - eye).normalized());

                // super sampling / anti aliasing
                int aa = 1;
                for (int i = 1; i <= aa; i++) {
                    for (int j = 1; j <= aa; j++) {
                        pixel = origin + right * (x - 0.5f + i * 1.0f / (aa + 1)) - up * (y - 0.5f + j * 1.0f / (aa + 1));
                        ray = Ray(eye, (pixel - eye).normalized());
                        root.getInterval(ray);
                        total += trace(ray, triangles, 0);
                    }
                }
                total /= (aa * aa);

                total.clamp();
                img(x, y) = total;
            }
        }
        auto traceWithKDEnd = high_resolution_clock::now();
        duration<double, std::milli> traceTimeWithKD = traceWithKDEnd - traceWithKDStart;

        std::cout << "Execution time (tracing with kd tree): " << traceTimeWithKD.count() << "ms" << std::endl;
        std::cout << "Total number of rays traced: " << raycount << std::endl;
        std::cout << "______________________________________________________________" << std::endl;

        img.write_png("../KD.png");



        // clean up
        root.cleanUpTree();

    }



    // naive ray tracing (no KD usage, no CUDA)
    raycount = 0;
    useKD = false;
    auto traceNoKDStart = high_resolution_clock::now();
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            Color total;
            Point pixel;
            Ray ray(eye, (pixel - eye).normalized());

            // super sampling / anti aliasing
            int aa = 1;
            for (int i = 1; i <= aa; i++) {
                for (int j = 1; j <= aa; j++) {
                    pixel = origin + right * (x - 0.5f + i * 1.0f / (aa + 1)) - up * (y - 0.5f + j * 1.0f / (aa + 1));
                    ray = Ray(eye, (pixel - eye).normalized());
                    total += trace(ray, triangles, 0);
                }
            }
            total /= (aa * aa);

            total.clamp();
            img(x, y) = total;
        }
    }
    auto traceNoKDEnd = high_resolution_clock::now();
    duration<double, std::milli> traceTimeNoKD = traceNoKDEnd - traceNoKDStart;

    std::cout << "Execution time (naive tracing): " << traceTimeNoKD.count() << "ms" << std::endl;
    std::cout << "Total number of rays traced: " << raycount << std::endl;
    std::cout << "______________________________________________________________" << std::endl;

    img.write_png("../naive.png");
    
    return 0;
}





