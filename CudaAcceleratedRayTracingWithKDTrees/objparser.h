#ifndef OBJPARSER_H
#define OBJPARSER_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <limits>
#include "object.h"
#include "triangle.h"

class OBJParser
{
public:
    OBJParser();

    static std::vector<Triangle> parseOBJ(string path) {
        ifstream objf;
        string line;

        int v = 0;    //number of vertex coords
        int vt = 0;   //number of texture coords
        int vn = 0;   //number of normals
        int f = 0;    //number of triangles

        objf.open(path);


        if (!objf.is_open()) {
            std::cout << "Couldn't open file!" << std::endl;
            return {};
        }
        else {

            while (std::getline(objf, line, ' ')) {   // first run to determine the size of our arrays

                if (line == "v") {    // vertices
                    v++;
                }
                else if (line == "vt") {  // texcoords
                    vt++;
                }
                else if (line == "vn") { // normals
                    vn++;
                }
                else if (line == "f") {  // triangles
                    f++;
                }
                std::getline(objf, line, '\n');   // ignore lines starting with different letters

            }

            objf.clear();
            objf.seekg(0);  // resetting file pointer


            // DEBUG
            //std::cout << "vs counted: " << v << std::endl;
            //std::cout << "fs counted: " << f << std::endl;




            //initializing arrays to store attributes
            std::vector<Point> vertices;
            std::vector<Point> textures;    // texcoords 2D, fill z values with 0, subject to change
            std::vector<Vector> normals;
            std::vector<Triangle> triangles;

            int vindices[3];    // indices of vertex coords stored in the vertices array
            //int vtindex;
            //int nindex;


            string split;  // split line


            while (std::getline(objf, split, ' ')) {   // second run to fill the arrays with data

                if (split == "v") {   //vertices

                    std::getline(objf, split, ' ');
                    float x = std::stof(split);
                    //x = x * 5 - 500;

                    std::getline(objf, split, ' ');
                    float y = std::stof(split);
                    //y = y * 5;

                    std::getline(objf, split, '\n');
                    float z = std::stof(split);
                    //z = z * 5 - 300;

                    vertices.push_back(Point(x, y, z));
                }
                else if (split == "vt") {  // texcoords
                    std::getline(objf, split, ' ');
                    float x = std::stof(split);

                    std::getline(objf, split, '\n');
                    float y = std::stof(split);

                    textures.push_back(Vector(x, y, 0));
                }
                else if (split == "vn") { // normals
                    std::getline(objf, split, ' ');
                    float x = std::stof(split);

                    std::getline(objf, split, ' ');
                    float y = std::stof(split);

                    std::getline(objf, split, '\n');
                    float z = std::stof(split);

                    normals.push_back(Vector(x, y, z));
                }
                else if (split == "f") {  // triangles

                    /*  use this for .obj files with texture coords and normals
                    for (int i=0; i<3; i++){    // getting vertex coords, texture coords and normals for 1st, 2nd and 3rd vertex of the triangle

                        std::getline(objf, split, ' ');
                        vindices[i] = std::stoi(split)-1;   // index for vertex coords


                        std::getline(objf, split, ' ');
                        vtindex = std::stoi(split)-1;   // index for texture coords

                        if(i==2) std::getline(objf, split, '\n');
                        else std::getline(objf, split, ' ');
                        nindex = std::stoi(split)-1;    // index for normals



                    }

                    triangles->push_back(Triangle(vertices->at(vindices[0]), vertices->at(vindices[1]), vertices->at(vindices[2])));

                    */

                    std::getline(objf, split, ' ');
                    vindices[0] = std::stoi(split) - 1;

                    std::getline(objf, split, ' ');
                    vindices[1] = std::stoi(split) - 1;

                    std::getline(objf, split, '\n');
                    vindices[2] = std::stoi(split) - 1;


                    //std::cout << vertices->at(vindices[0]) << std::endl;

                    triangles.push_back(Triangle(vertices.at(vindices[0]), vertices.at(vindices[1]), vertices.at(vindices[2])));


                }
                else {
                    std::getline(objf, split, '\n');    // ignore lines starting with different letters
                }
            }



            // clean up
            objf.close();

            return triangles;

        }

    }
};

#endif // OBJPARSER_H