#ifndef KDNODE_H
#define KDNODE_H

#include "triangle.h"
#include "ray.h"
#include "hit.h"
#include <vector>
#include <tuple>

class KDNode
{
public:
    KDNode() {
        for (int i = 0; i < 6; i++) bounds[i] = 0;
        front = nullptr;
        back = nullptr;
        isLeaf = false;
    }
    KDNode(float MIN_X, float MAX_X, float MIN_Y, float MAX_Y, float MIN_Z, float MAX_Z) : min_x(MIN_X), max_x(MAX_X), min_y(MIN_Y), max_y(MAX_Y), min_z(MIN_Z), max_z(MAX_Z), triangleStart(0), triangleEnd(0), isLeaf(false), front(nullptr), back(nullptr)
    { }
    ~KDNode() {
        clearTris();
    }

    union {
        float bounds[6];

        struct {
            float min_x;   // B
            float max_x;   // O
            float min_y;   // U
            float max_y;   // N
            float min_z;   // D
            float max_z;   // S
        };

    };


    float tstart;  // interval during which
    float tend;    // a ray is inside the box
    float splitDist;   // distance from the start of the front node
    int splitDim; // dimension along which to split (0 for x, 1 for y, 2 for z)
    
    // save starting and end index and store triangles array separately
    int triangleStart;
    int triangleEnd;



    bool isLeaf;

    KDNode* front;  // child 1
    KDNode* back;   // child 2

    std::vector<int> triangles;    // only for leaves, array of triangle indices

    //DEBUG
    int numTris;



    void getInterval(Ray ray) {  // computes the interval [tstart:tend] during which the ray is inside the box/kdnode

        if (ray.D.x == 0 && (ray.O.x < min_x || ray.O.x > max_x)) { tstart = -1; tend = -1; return; }
        if (ray.D.y == 0 && (ray.O.y < min_y || ray.O.y > max_y)) { tstart = -1; tend = -1; return; }   // ray is parallel
        if (ray.D.z == 0 && (ray.O.z < min_z || ray.O.z > max_z)) { tstart = -1; tend = -1; return; }


        float invRdx = 1 / ray.D.x;
        float invRdy = 1 / ray.D.y;
        float invRdz = 1 / ray.D.z;

        float t1;
        float t2;
        float temp;


        // getting the x interval
        tstart = (min_x - ray.O.x) * invRdx;
        tend = (max_x - ray.O.x) * invRdx;
        if (tstart > tend) {
            temp = tstart;
            tstart = tend;
            tend = temp;
        }
        if (tstart < 0) tstart = 0; // t can't be negative, we are potentially inside the box
        if (tend < 0) return;   // box is behind eye in x


        // y interval
        t1 = (min_y - ray.O.y) * invRdy;
        t2 = (max_y - ray.O.y) * invRdy;

        if (t1 > t2) {
            temp = t1;
            t1 = t2;
            t2 = temp;
        }
        if (t1 < 0) t1 = 0;
        if (t2 < 0){  // box is behind eye in y
            tend = t2;
            return;
        }  
        if (t1 > tstart) tstart = t1;   // checking
        if (t2 < tend) tend = t2;       // for overlaps


        // z interval
        t1 = (min_z - ray.O.z) * invRdz;
        t2 = (max_z - ray.O.z) * invRdz;

        if (t1 > t2) {
            temp = t1;
            t1 = t2;
            t2 = temp;
        }
        if (t1 < 0) t1 = 0;
        if (t2 < 0) { tend = t2; return; }   // box is behind eye in z
        if (t1 > tstart) tstart = t1;   // checking
        if (t2 < tend) tend = t2;       // for overlaps

    }




    //    // split() sets the geometry for KDNode childen front and back by splitting them in the split dimension
    //    int split(std::vector<Triangle>* allTriangles){
    //        front = new KDNode(this->min_x, this->max_x, this->min_y, this->max_y, this->min_z, this->max_z);
    //        back = new KDNode(this->min_x, this->max_x, this->min_y, this->max_y, this->min_z, this->max_z);
    //        getSplit(allTriangles);
    //        float temp = bounds[splitDim*2] + splitDist;    // splitDim is either 0, 1 or 2 encoding x, y and z dimensions
    //                                                                // bounds[splitDim*2] gets us the minimum value for that dimension, e.g.
    //        front->bounds[splitDim*2+1] = temp;                     // 0*2 = 0 -> bounds[0] = min_x; 1*2 = 2 -> bounds[2] = min_y; 2*2 = 4 -> bounds[4] = min_z
    //        back->bounds[splitDim*2] = temp;                        // whereas bounds[splitDim*2+1] gets us the max value
    //        return 0;
    //    }


    // clean up, deletes triangles array and sets the pointer to be the nullptr
    int clearTris() {
        try {
            triangles.clear();
            triangles.shrink_to_fit();
            return 1;
        }
        catch (...) {
            return 0;
        }
    }


    // gets initial bounds for root node from most outer points in the scene
    void getInitialBounds(std::vector<Triangle> triangles) {
        float minx = std::numeric_limits<float>::max(), miny = std::numeric_limits<float>::max(), minz = std::numeric_limits<float>::max();
        float maxx = std::numeric_limits<float>::lowest(), maxy = std::numeric_limits<float>::lowest(), maxz = std::numeric_limits<float>::lowest();
        float offset = 0.0000001f;
        for (int i = 0; i < triangles.size(); i++) {
            if (triangles[i].min_x() < minx) minx = triangles[i].min_x();
            if (triangles[i].max_x() > maxx) maxx = triangles[i].max_x();
            if (triangles[i].min_y() < miny) miny = triangles[i].min_y();
            if (triangles[i].max_y() > maxy) maxy = triangles[i].max_y();
            if (triangles[i].min_z() < minz) minz = triangles[i].min_z();
            if (triangles[i].max_z() > maxz) maxz = triangles[i].max_z();
        }
        min_x = minx - offset;
        max_x = maxx + offset;
        min_y = miny - offset;
        max_y = maxy + offset;
        min_z = minz - offset;
        max_z = maxz + offset;
    }

    // prints out bounds
    void printBounds() {
        std::cout << "min x: " << min_x << std::endl;
        std::cout << "max x: " << max_x << std::endl;
        std::cout << "min y: " << min_y << std::endl;
        std::cout << "max y: " << max_y << std::endl;
        std::cout << "min z: " << min_z << std::endl;
        std::cout << "max z: " << max_z << std::endl;
    }

    // recursive version of split
    int makeTree(std::vector<Triangle>& allTriangles, int sd) {

        splitDim = sd % 3;
        getCoverage(allTriangles);

        //DEBUG
        numTris = triangles.size();

        if (sd == 0) { 
            isLeaf = true;
            return 1;
        }
        else {

            try {
                front = new KDNode(this->min_x, this->max_x, this->min_y, this->max_y, this->min_z, this->max_z);
                back = new KDNode(this->min_x, this->max_x, this->min_y, this->max_y, this->min_z, this->max_z);

                // BENCHMARK getSplit vs halfSplit
                getSplit(allTriangles);

                clearTris();

                float temp = bounds[splitDim * 2] + splitDist;    // splitDim is either 0, 1 or 2 encoding x, y and z dimensions
                                                                 // bounds[splitDim*2] gets us the minimum value for that dimension, e.g.
                front->bounds[splitDim * 2 + 1] = temp;              // 0*2 = 0 -> bounds[0] = min_x; 1*2 = 2 -> bounds[2] = min_y; 2*2 = 4 -> bounds[4] = min_z
                back->bounds[splitDim * 2] = temp;                 // whereas bounds[splitDim*2+1] gets us the max value


                front->makeTree(allTriangles, sd - 1);
                back->makeTree(allTriangles, sd - 1);
            }
            catch (...) {
                std::cout << "ERROR BUILDING TREE" << std::endl;
            }
        }

        return 0;
    }

    // CUDA version of split
    int makeTreeCUDA(std::vector<Triangle>& allTriangles, std::vector<KDNode>& tree, std::vector<int>& leafTriangles, unsigned int index, int sd) {

        splitDim = sd % 3;
        getCoverage(allTriangles);

        //DEBUG
        numTris = triangles.size();
        

        if (sd == 0) { 
            isLeaf = true;
            // fill leafTriangles array here 
            triangleStart = leafTriangles.size();
            leafTriangles.insert(leafTriangles.end(), triangles.begin(), triangles.end());
            triangleEnd = leafTriangles.size()-1;

            tree.at(index - 1) = *this;
            return 1;
        }
        else {

            try {
                front = new KDNode(this->min_x, this->max_x, this->min_y, this->max_y, this->min_z, this->max_z);
                back = new KDNode(this->min_x, this->max_x, this->min_y, this->max_y, this->min_z, this->max_z);

                // BENCHMARK getSplit vs halfSplit
                getSplit(allTriangles);

                clearTris();

                float temp = bounds[splitDim * 2] + splitDist;    // splitDim is either 0, 1 or 2 encoding x, y and z dimensions
                                                                 // bounds[splitDim*2] gets us the minimum value for that dimension, e.g.
                front->bounds[splitDim * 2 + 1] = temp;              // 0*2 = 0 -> bounds[0] = min_x; 1*2 = 2 -> bounds[2] = min_y; 2*2 = 4 -> bounds[4] = min_z
                back->bounds[splitDim * 2] = temp;                 // whereas bounds[splitDim*2+1] gets us the max value

                tree.at(index - 1) = *this;


                front->makeTreeCUDA(allTriangles, tree, leafTriangles, index*2, sd - 1);
                back->makeTreeCUDA(allTriangles, tree, leafTriangles, index*2+1, sd - 1);
            }
            catch (...) {
                std::cout << "ERROR BUILDING TREE" << std::endl;
                if (triangles.size() == 0) std::cout << "no triangles in the array!" << std::endl;
            }
        }

        return 0;
    }

    // makes already existing kd tree cuda compatible by storing it in an array
    int makeTreeCUDAfromTree(std::vector<KDNode>& tree, unsigned index) {

        tree.at(index - 1) = *this;
        if (front == nullptr && back == nullptr) return 1;
        else {
            front->makeTreeCUDAfromTree(tree, index * 2);
            back->makeTreeCUDAfromTree(tree, index * 2 + 1);
        }
        return 0;
    }

    // clean up and delete tree
    void cleanUpTree() {
        if (isLeaf) {
            return;
        }
        else{
            front->cleanUpTree();
            back->cleanUpTree();
            delete front;
            delete back;
            front = nullptr;
            back = nullptr;
            return;
        }
    }

    // old debug print
    int printTree(int nodeNum) {

        std::cout << "Node with number " << nodeNum - 1 << " has " << numTris << " triangles." << std::endl;
        if (front == nullptr && back == nullptr) return 1;
        else {
            front->printTree(nodeNum * 2);
            back->printTree(nodeNum * 2 + 1);
        }
        return 0;
    }

    // getCoverage() determines which triangles we need to intersect with when hitting a leaf node with our ray
    // allTriangles points to the array storing all triangle objects whereas triangles, which gets returned is an array of pointers to exactly
    // those triangle objects that fall within the bounds of the kdnode
    std::vector<int> getCoverage(std::vector<Triangle>& allTriangles) {

        int count = 0;

        if (triangles.size() == 0) {   // this is to avoid filling the array with multiples of the same pointer when calling getCoverage() more than once
            /*for (unsigned int i = 0; i < allTriangles.size(); i++) {
                if ((allTriangles.at(i).max_x() >= this->min_x && allTriangles.at(i).min_x() <= this->max_x) && (allTriangles.at(i).max_y() >= this->min_y && allTriangles.at(i).min_y() <= this->max_y) && (allTriangles.at(i).max_z() >= this->min_z && allTriangles.at(i).min_z() <= this->max_z)) {
                    count++;
                }
            }*/




            try {
                for (unsigned int i = 0; i < allTriangles.size(); i++) {
                    if ((allTriangles.at(i).max_x() >= this->min_x && allTriangles.at(i).min_x() <= this->max_x) && (allTriangles.at(i).max_y() >= this->min_y && allTriangles.at(i).min_y() <= this->max_y) && (allTriangles.at(i).max_z() >= this->min_z && allTriangles.at(i).min_z() <= this->max_z)) {
                        triangles.push_back(i);
                    }
                }
            }
            catch (...) {
                std::cout << "ERROR GETTING TRIANGLE COVERAGE" << std::endl;
            }
        }
        return triangles;
    }






    int partition(std::vector<Triangle>& allTriangles, int low, int high) {

        float pivot;
        if (splitDim == 0) pivot = allTriangles[triangles.at(high)].max_x();
        else if (splitDim == 1) pivot = allTriangles[triangles.at(high)].max_y();
        else if (splitDim == 2) pivot = allTriangles[triangles.at(high)].max_z();

        int i = low - 1;

        for (int j = low; j < high; j++) {
            if (splitDim == 0) {     // x dimension
                if (allTriangles[triangles.at(j)].max_x() < pivot) {
                    //std::cout<<"x"<<std::endl;

                    i++;
                    int temp = triangles.at(i);
                    triangles.at(i) = triangles.at(j);
                    triangles.at(j) = temp;

                }
            }
            else if (splitDim == 1) {    // y dimension
                if (allTriangles[triangles.at(j)].max_y() < pivot) {
                    //std::cout<<"y"<<std::endl;

                    i++;
                    int temp = triangles.at(i);
                    triangles.at(i) = triangles.at(j);
                    triangles.at(j) = temp;

                }
            }
            else if (splitDim == 2) {    // z dimension
                if (allTriangles[triangles.at(j)].max_z() < pivot) {
                    //std::cout<<"z"<<std::endl;

                    i++;
                    int temp = triangles.at(i);
                    triangles.at(i) = triangles.at(j);
                    triangles.at(j) = temp;

                }
            }
        }

        int temp = triangles.at(high);
        triangles.at(high) = triangles.at(i + 1);
        triangles.at(i + 1) = temp;
        return i + 1;
    }

    void quicksort(std::vector<Triangle>& allTriangles, int low, int high) {
        if (low < high) {
            int pi = partition(allTriangles, low, high);
            quicksort(allTriangles, low, pi - 1);
            quicksort(allTriangles, pi + 1, high);
        }
    }

    // sorts triangles by split dimension using quick sort
    std::vector<int> sortTriangles(std::vector<Triangle>& allTriangles) {

        quicksort(allTriangles, 0, triangles.size() - 1);

        return triangles;
    }


    // getSplit() computes the split distance using the median in the split dimension from the triangles array to ensure
    // half the triangles become part of the front node and the other half becomes part of the back node
    // in order to do that we first need to sort the triangles array by x, y or z values in ascending order
    float getSplit(std::vector<Triangle>& allTriangles) {

        // sort triangles array
        sortTriangles(allTriangles);


        //calculate splitDist from median and current bounds
        if (splitDim == 0) {
            if (triangles.size() % 2 == 1) splitDist = allTriangles[triangles.at(triangles.size() / 2)].max_x() - min_x;
            else splitDist = (allTriangles[triangles.at(triangles.size() / 2)].max_x() + allTriangles[triangles.at(triangles.size() / 2 + 1)].max_x()) / 2 - min_x;
        }
        else if (splitDim == 1) {
            if (triangles.size() % 2 == 1) splitDist = allTriangles[triangles.at(triangles.size() / 2)].max_y() - min_y;
            else splitDist = (allTriangles[triangles.at(triangles.size() / 2)].max_y() + allTriangles[triangles.at(triangles.size() / 2 + 1)].max_y()) / 2 - min_y;
        }
        else if (splitDim == 2) {
            if (triangles.size() % 2 == 1) splitDist = allTriangles[triangles.at(triangles.size() / 2)].max_z() - min_z;
            else splitDist = (allTriangles[triangles.at(triangles.size() / 2)].max_z() + allTriangles[triangles.at(triangles.size() / 2 + 1)].max_z()) / 2 - min_z;
        }

        return splitDist;
    }

    // naive half split
    float halfSplit() {
        if (splitDim == 0) splitDist = (max_x - min_x) / 2;
        if (splitDim == 1) splitDist = (max_y - min_y) / 2;
        if (splitDim == 2) splitDist = (max_z - min_z) / 2;
        return splitDist;
    }


    // traversal function
    Hit traverse(std::vector<Triangle>& allTriangles, Ray ray) {

        if (tstart > 0 && tstart > tend) return Hit::NO_HIT();  // ray didn't hit KD node
        if (tend < 0) return Hit::NO_HIT(); // box is behind eye


        if (isLeaf) {

            //if (triangles->size()==0) return Hit::NO_HIT(); // if an empty box happens to be a leaf we need this line, only happens when doing halfSplit

            Hit min_hit(std::numeric_limits<float>::infinity(), Vector());

            // compute closest hit
            for (unsigned int i = 0; i < triangles.size(); ++i) {
                Hit hit(allTriangles[triangles.at(i)].intersect(ray));
                if (hit.t < min_hit.t) {
                    min_hit = hit;
                }
            }

            return min_hit;

            // relevant for overlapping primitives in different bounding boxes
            /*if (min_hit.t <= tend && min_hit.t >= tstart) return min_hit;
            else return Hit::NO_HIT();*/

        }

        float t = (bounds[2 * splitDim] - ray.O.data[splitDim] + splitDist) / ray.D.data[splitDim];   // timesteps in splitDim direction to splitting plane



        if (ray.D.data[splitDim] < 0) {  // swap front and back if the ray comes from the opposite direction along the split dimension
            if (t <= tstart) { // only intersect front node
                front->tstart = tstart;
                front->tend = tend;
                return front->traverse(allTriangles, ray);
            }
            else if (t >= tend) {  // only intersect back node
                back->tstart = tstart;
                back->tend = tend;
                return back->traverse(allTriangles, ray);
            }
            else {
                back->tstart = tstart;
                back->tend = t;
                front->tstart = t;
                front->tend = tend;
                Hit hit = back->traverse(allTriangles, ray);
                if (hit.t <= t) return hit;
                else return front->traverse(allTriangles, ray);
            }
        }
        else {
            if (t <= tstart) { // only intersect back node
                back->tstart = tstart;
                back->tend = tend;
                return back->traverse(allTriangles, ray);
            }
            else if (t >= tend) {  // only intersect front node
                front->tstart = tstart;
                front->tend = tend;
                return front->traverse(allTriangles, ray);
            }
            else {
                front->tstart = tstart;
                front->tend = t;
                back->tstart = t;
                back->tend = tend;
                Hit hit = front->traverse(allTriangles, ray);
                if (hit.t <= t) return hit;
                else return back->traverse(allTriangles, ray);
            }
        }
    }

    // traversal function for shadow rays
    Hit shadTraverse(std::vector<Triangle>& allTriangles, Ray ray) {

        if (tstart > 0 && tstart > tend) return Hit::NO_HIT();  // ray didn't hit KD node
        if (tend < 0) return Hit::NO_HIT(); // box is behind eye


        if (isLeaf) {

            //if (triangles->size()==0) return Hit::NO_HIT(); // if an empty box happens to be a leaf we need this line, only happens when doing halfSplit

            Hit min_hit(std::numeric_limits<float>::infinity(), Vector());

            // compute closest hit
            for (unsigned int i = 0; i < triangles.size(); ++i) {
                Hit hit(allTriangles[triangles.at(i)].intersect(ray));
                if (hit.t < min_hit.t && hit.hitobject->refrIdx == 0) {
                    min_hit = hit;
                    break;
                }
            }

            return min_hit;

            // relevant for overlapping primitives in different bounding boxes
            /*if (min_hit.t <= tend && min_hit.t >= tstart) return min_hit;
            else return Hit::NO_HIT();*/

        }

        float t = (bounds[2 * splitDim] - ray.O.data[splitDim] + splitDist) / ray.D.data[splitDim];   // timesteps in splitDim direction to splitting plane



        if (ray.D.data[splitDim] < 0) {  // swap front and back if the ray comes from the opposite direction along the split dimension
            if (t <= tstart) { // only intersect front node
                front->tstart = tstart;
                front->tend = tend;
                return front->shadTraverse(allTriangles, ray);
            }
            else if (t >= tend) {  // only intersect back node
                back->tstart = tstart;
                back->tend = tend;
                return back->shadTraverse(allTriangles, ray);
            }
            else {
                back->tstart = tstart;
                back->tend = t;
                front->tstart = t;
                front->tend = tend;
                Hit hit = back->shadTraverse(allTriangles, ray);
                if (hit.t <= t) return hit;
                else return front->shadTraverse(allTriangles, ray);
            }
        }
        else {
            if (t <= tstart) { // only intersect back node
                back->tstart = tstart;
                back->tend = tend;
                return back->shadTraverse(allTriangles, ray);
            }
            else if (t >= tend) {  // only intersect front node
                front->tstart = tstart;
                front->tend = tend;
                return front->shadTraverse(allTriangles, ray);
            }
            else {
                front->tstart = tstart;
                front->tend = t;
                back->tstart = t;
                back->tend = tend;
                Hit hit = front->shadTraverse(allTriangles, ray);
                if (hit.t <= t) return hit;
                else return back->shadTraverse(allTriangles, ray);
            }
        }
    }


    // traversal using triple indexing for triangles
    Hit tripleIndexTraverse(std::vector<Triangle>& allTriangles, Ray ray, std::vector<int>& leaftris) {

        if (tstart > 0 && tstart > tend) return Hit::NO_HIT();  // ray didn't hit KD node
        if (tend < 0) return Hit::NO_HIT(); // box is behind eye


        if (isLeaf) {

            //if (triangles->size()==0) return Hit::NO_HIT(); // if an empty box happens to be a leaf we need this line, only happens when doing halfSplit

            Hit min_hit(std::numeric_limits<float>::infinity(), Vector());

            // compute closest hit
            for (unsigned int i = triangleStart; i <= triangleEnd; ++i) {
                Hit hit(allTriangles[leaftris[i]].intersect(ray));
                if (hit.t < min_hit.t) {
                    min_hit = hit;
                }
            }

            return min_hit;

            // relevant for overlapping primitives in different bounding boxes
            /*if (min_hit.t <= tend && min_hit.t >= tstart) return min_hit;
            else return Hit::NO_HIT();*/

        }

        float t = (bounds[2 * splitDim] - ray.O.data[splitDim] + splitDist) / ray.D.data[splitDim];   // timesteps in splitDim direction to splitting plane



        if (ray.D.data[splitDim] < 0) {  // swap front and back if the ray comes from the opposite direction along the split dimension
            if (t <= tstart) { // only intersect front node
                front->tstart = tstart;
                front->tend = tend;
                return front->tripleIndexTraverse(allTriangles, ray, leaftris);
            }
            else if (t >= tend) {  // only intersect back node
                back->tstart = tstart;
                back->tend = tend;
                return back->tripleIndexTraverse(allTriangles, ray, leaftris);
            }
            else {
                back->tstart = tstart;
                back->tend = t;
                front->tstart = t;
                front->tend = tend;
                Hit hit = back->tripleIndexTraverse(allTriangles, ray, leaftris);
                if (hit.t <= t) return hit;
                else return front->tripleIndexTraverse(allTriangles, ray, leaftris);
            }
        }
        else {
            if (t <= tstart) { // only intersect back node
                back->tstart = tstart;
                back->tend = tend;
                return back->tripleIndexTraverse(allTriangles, ray, leaftris);
            }
            else if (t >= tend) {  // only intersect front node
                front->tstart = tstart;
                front->tend = tend;
                return front->tripleIndexTraverse(allTriangles, ray, leaftris);
            }
            else {
                front->tstart = tstart;
                front->tend = t;
                back->tstart = t;
                back->tend = tend;
                Hit hit = front->tripleIndexTraverse(allTriangles, ray, leaftris);
                if (hit.t <= t) return hit;
                else return back->tripleIndexTraverse(allTriangles, ray, leaftris);
            }
        }






    }



};




#endif // KDNODE_H
