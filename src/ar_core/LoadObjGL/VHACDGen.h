//
// Created by nima on 13/07/17.
//

#ifndef ATAR_VHACDGEN_H
#define ATAR_VHACDGEN_H

#include <iostream>
#include <vector>

int DecomposeObj(const std::string file_name);

void GetFileExtension(const std::string& fileName, std::string& fileExtension);

std::string AddHACDToName(const std::string& fileName);


#endif //ATAR_VHACDGEN_H
