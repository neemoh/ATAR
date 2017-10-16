//
// Created by nima on 13/07/17.
//

// Nima: written from the test_vhacd.cpp code in Extra modules of the Bullet
// library


/* Copyright (c) 2011 Khaled Mamou (kmamou at gmail dot com)
 All rights reserved.


 Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

 3. The names of the contributors may not be used to endorse or promote products derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "VHACDGen.h"
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string.h>
#include "VHACD/inc/VHACD.h"

using namespace VHACD;

class MyCallback : public IVHACD::IUserCallback {
public:
    MyCallback(void) {}
    ~MyCallback(){};
    void Update(const double overallProgress, const double stageProgress, const double operationProgress,
                const char* const stage, const char* const operation)
    {
        using namespace std;
        cout << setfill(' ') << setw(3) << (int)(overallProgress + 0.5) << "% "
             << "[ " << stage << " " << setfill(' ') << setw(3) << (int)(stageProgress + 0.5) << "% ] "
             << operation << " " << setfill(' ') << setw(3) << (int)(operationProgress + 0.5) << "%" << endl;
    };
};

class MyLogger : public IVHACD::IUserLogger {
public:
    MyLogger(void) {}
    MyLogger(const std::string& fileName) { OpenFile(fileName); }
    ~MyLogger(){};
    void Log(const char* const msg)
    {
        if (m_file.is_open()) {
            m_file << msg;
            m_file.flush();
        }
    }
    void OpenFile(const std::string& fileName)
    {
        m_file.open(fileName.c_str());
    }

private:
    std::ofstream m_file;
};

struct Material {

    float m_diffuseColor[3];
    float m_ambientIntensity;
    float m_specularColor[3];
    float m_emissiveColor[3];
    float m_shininess;
    float m_transparency;
    Material(void)
    {
        m_diffuseColor[0] = 0.5f;
        m_diffuseColor[1] = 0.5f;
        m_diffuseColor[2] = 0.5f;
        m_specularColor[0] = 0.5f;
        m_specularColor[1] = 0.5f;
        m_specularColor[2] = 0.5f;
        m_ambientIntensity = 0.4f;
        m_emissiveColor[0] = 0.0f;
        m_emissiveColor[1] = 0.0f;
        m_emissiveColor[2] = 0.0f;
        m_shininess = 0.4f;
        m_transparency = 0.5f;
    };
};
struct Parameters {
    unsigned int m_oclPlatformID;
    unsigned int m_oclDeviceID;
    std::string m_fileNameIn;
    std::string m_fileNameOut;
    std::string m_fileNameLog;
    bool m_run;
    IVHACD::Parameters m_paramsVHACD;
    Parameters(void)
    {
        m_run = true;
        m_oclPlatformID = 0;
        m_oclDeviceID = 0;
        m_fileNameIn = "";
        m_fileNameOut = "output.obj";
        m_fileNameLog = "log.txt";
    }
};


bool LoadOBJ(const std::string& fileName,  std::vector<float>& points,  std::vector<int>& triangles, IVHACD::IUserLogger& logger);
bool SaveOBJ(std::ofstream& fout, const double* const& points, const int*
const& triangles, const unsigned int& nPoints,
             const unsigned int& nTriangles, const Material& material, IVHACD::IUserLogger& logger, int convexPart, int vertexOffset);

int DecomposeObj(const std::string file_name)
{
    using namespace std;
    // --input camel.off --output camel_acd.obj --log log.txt
    // --resolution 1000000 --depth 20 --concavity 0.0025
    // --planeDownsampling 4 --convexhullDownsampling 4 --alpha 0.05 --beta 0.05 --gamma 0.00125
    // --pca 0 --mode 0 --maxNumVerticesPerCH 256 --minVolumePerCH 0.0001
    // --convexhullApproximation 1 --oclDeviceID 2

    // set parameters
    Parameters params;
    MyCallback myCallback;
    params.m_fileNameLog = "vhacd_log";
    MyLogger myLogger(params.m_fileNameLog);
    params.m_paramsVHACD.m_logger = &myLogger;
    params.m_paramsVHACD.m_callback = &myCallback;
    if (!params.m_run) {
        return 0;
    }


    params.m_fileNameIn                             = file_name;
    params.m_paramsVHACD.m_resolution               = 1000000;
    params.m_paramsVHACD.m_depth                    = 20;
    params.m_paramsVHACD.m_concavity                = 0.0025;
    params.m_paramsVHACD.m_planeDownsampling        = 4;
    params.m_paramsVHACD.m_convexhullDownsampling   = 4;
    params.m_paramsVHACD.m_alpha                    = 0.05;
    params.m_paramsVHACD.m_beta                     = 0.05;
    params.m_paramsVHACD.m_gamma                    = 0.00125;
    params.m_paramsVHACD.m_pca                      = 0;
    params.m_paramsVHACD.m_mode                     = 0;
    params.m_paramsVHACD.m_maxNumVerticesPerCH      = 256;
    params.m_paramsVHACD.m_minVolumePerCH           = 0.0001;
    params.m_paramsVHACD.m_convexhullApproximation  = 1;
//    params.m_paramsVHACD.m_oclAcceleration          =
//    params.m_oclPlatformID
//    params.m_oclDeviceID

    params.m_fileNameOut = AddHACDToName(file_name) ;



    std::ostringstream msg;

    msg << "+ OpenCL (OFF)" << std::endl;

    msg << "+ Parameters" << std::endl;
    msg << "\t input                                       " << params.m_fileNameIn << endl;
    msg << "\t resolution                                  " << params.m_paramsVHACD.m_resolution << endl;
    msg << "\t max. depth                                  " << params.m_paramsVHACD.m_depth << endl;
    msg << "\t max. concavity                              " << params.m_paramsVHACD.m_concavity << endl;
    msg << "\t plane down-sampling                         " << params.m_paramsVHACD.m_planeDownsampling << endl;
    msg << "\t convex-hull down-sampling                   " << params.m_paramsVHACD.m_convexhullDownsampling << endl;
    msg << "\t alpha                                       " << params.m_paramsVHACD.m_alpha << endl;
    msg << "\t beta                                        " << params.m_paramsVHACD.m_beta << endl;
    msg << "\t gamma                                       " << params.m_paramsVHACD.m_gamma << endl;
    msg << "\t pca                                         " << params.m_paramsVHACD.m_pca << endl;
    msg << "\t mode                                        " << params.m_paramsVHACD.m_mode << endl;
    msg << "\t max. vertices per convex-hull               " << params.m_paramsVHACD.m_maxNumVerticesPerCH << endl;
    msg << "\t min. volume to add vertices to convex-hulls " << params.m_paramsVHACD.m_minVolumePerCH << endl;
    msg << "\t convex-hull approximation                   " << params.m_paramsVHACD.m_convexhullApproximation << endl;
    msg << "\t OpenCL acceleration                         " << params.m_paramsVHACD.m_oclAcceleration << endl;
    msg << "\t OpenCL platform ID                          " << params.m_oclPlatformID << endl;
    msg << "\t OpenCL device ID                            " << params.m_oclDeviceID << endl;
    msg << "\t output                                      " << params.m_fileNameOut << endl;
    msg << "\t log                                         " << params.m_fileNameLog << endl;
    msg << "+ Load mesh" << std::endl;
    myLogger.Log(msg.str().c_str());

    cout << msg.str().c_str();

    // load mesh
     std::vector<float> points;
     std::vector<int> triangles;
    std::string fileExtension;
    GetFileExtension(params.m_fileNameIn, fileExtension);
    if (fileExtension == ".OBJ") {
        if (!LoadOBJ(params.m_fileNameIn, points, triangles, myLogger))
            return -1;
    }
    else {
        myLogger.Log("Format not supported!\n");
        return -1;
    }

    // run V-HACD
    IVHACD* interfaceVHACD = CreateVHACD();

    bool res = interfaceVHACD->Compute(&points[0], 3,
                                       (unsigned int)points.size() / 3,
                                       &triangles[0], 3,
                                       (unsigned int)triangles.size() / 3,
                                       params.m_paramsVHACD);
    if (res) {
        // save output
        unsigned int nConvexHulls = interfaceVHACD->GetNConvexHulls();
        msg.str("");
        msg << "+ Generate output: " << nConvexHulls << " convex-hulls " << endl;
        myLogger.Log(msg.str().c_str());
        ofstream foutCH(params.m_fileNameOut.c_str());
        IVHACD::ConvexHull ch;
        if (foutCH.is_open()) {
            Material mat;
            int vertexOffset = 1;//obj wavefront starts counting at 1...
            for (unsigned int p = 0; p < nConvexHulls; ++p) {
                interfaceVHACD->GetConvexHull(p, ch);


                SaveOBJ(foutCH, ch.m_points, ch.m_triangles, ch.m_nPoints,
                        ch.m_nTriangles, mat, myLogger, p, vertexOffset);
                vertexOffset+=ch.m_nPoints;
                msg.str("");
                msg << "\t CH[" << setfill('0') << setw(5) << p << "] "
                    << ch.m_nPoints << " V, " << ch.m_nTriangles << " T" << endl;
                myLogger.Log(msg.str().c_str());
            }
            foutCH.close();
        }
    }
    else {
        myLogger.Log("Decomposition cancelled by user!\n");
    }


    interfaceVHACD->Clean();
    interfaceVHACD->Release();

    return 0;
}


void GetFileExtension(const std::string& fileName, std::string& fileExtension)
{
    size_t lastDotPosition = fileName.find_last_of(".");
    if (lastDotPosition == std::string::npos) {
        fileExtension = "";
    }
    else {
        fileExtension = fileName.substr(lastDotPosition, fileName.size());
        std::transform(fileExtension.begin(), fileExtension.end(), fileExtension
                .begin(), ::toupper);
    }
}


bool LoadOBJ(const std::string& fileName,  std::vector<float>& points,  std::vector<int>& triangles, IVHACD::IUserLogger& logger)
{
    const unsigned int BufferSize = 1024;
    FILE* fid = fopen(fileName.c_str(), "r");

    if (fid) {
        char buffer[BufferSize];
        int ip[4];
        float x[3];
        char* pch;
        char* str;
        while (!feof(fid)) {
            if (!fgets(buffer, BufferSize, fid)) {
                break;
            }
            else if (buffer[0] == 'v') {
                if (buffer[1] == ' ') {
                    str = buffer + 2;
                    for (int k = 0; k < 3; ++k) {
                        pch = strtok(str, " ");
                        if (pch)
                            x[k] = (float)atof(pch);
                        else {
                            return false;
                        }
                        str = NULL;
                    }
                    points.push_back(x[0]);
                    points.push_back(x[1]);
                    points.push_back(x[2]);
                }
            }
            else if (buffer[0] == 'f') {

                pch = str = buffer + 2;
                int k = 0;
                while (pch) {
                    pch = strtok(str, " ");
                    if (pch) {
                        ip[k++] = atoi(pch) - 1;
                    }
                    else {
                        break;
                    }
                    str = NULL;
                }
                if (k == 3) {
                    triangles.push_back(ip[0]);
                    triangles.push_back(ip[1]);
                    triangles.push_back(ip[2]);
                }
                else if (k == 4) {
                    triangles.push_back(ip[0]);
                    triangles.push_back(ip[1]);
                    triangles.push_back(ip[2]);

                    triangles.push_back(ip[0]);
                    triangles.push_back(ip[2]);
                    triangles.push_back(ip[3]);
                }
            }
        }
        fclose(fid);
    }
    else {
        logger.Log("File not found\n");
        return false;
    }
    return true;
}


bool SaveOBJ(std::ofstream& fout, const double* const& points, const int*
const& triangles, const unsigned int& nPoints,
             const unsigned int& nTriangles, const Material& material, IVHACD::IUserLogger& logger, int convexPart, int vertexOffset)
{
    if (fout.is_open()) {

        fout.setf(std::ios::fixed, std::ios::floatfield);
        fout.setf(std::ios::showpoint);
        fout.precision(6);
        size_t nV = nPoints * 3;
        size_t nT = nTriangles * 3;

        fout << "o convex_" << convexPart << std::endl;

        if (nV > 0) {
            for (size_t v = 0; v < nV; v += 3) {
                fout << "v " << points[v + 0] << " " << points[v + 1] << " " << points[v + 2] << std::endl;
            }
        }
        if (nT > 0) {
            for (size_t f = 0; f < nT; f += 3) {
                fout << "f "
                     << triangles[f + 0]+vertexOffset << " "
                     << triangles[f + 1]+vertexOffset << " "
                     << triangles[f + 2]+vertexOffset << " " << std::endl;
            }
        }
        return true;
    }
    else {
        logger.Log("Can't open file\n");
        return false;
    }
}

std::string AddHACDToName(const std::string &fileName) {

    size_t last_dot_position = fileName.find_last_of(".");

    if (last_dot_position == std::string::npos) {
        return("");
    }
    else {
        std::string file_name_no_extension = fileName.substr(0,
                                                           last_dot_position);

        std::stringstream out_name;
        out_name << file_name_no_extension << "_hacd.obj";
        return out_name.str();
    }
}

