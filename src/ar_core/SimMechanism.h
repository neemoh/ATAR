//
// Created by nima on 11/11/17.
//

#ifndef ATAR_SIMMECHANISM_H
#define ATAR_SIMMECHANISM_H

// This class contains a vector of SimObjects* constrained by a vector of
// btHingeConstraint*

#include <vector>
#include "SimObject.h"

class SimMechanism {

public:
    ~SimMechanism(){
        for(auto i:sim_objects_)
            delete i;
        for(auto i:constraints_)
            delete i;}

    std::vector<SimObject*> GetSimObjects(){return sim_objects_;};

    std::vector<btHingeConstraint*> GetConstraints(){return constraints_;};

protected:
    std::vector<SimObject*> sim_objects_;

    std::vector<btHingeConstraint*> constraints_;

};


#endif //ATAR_SIMMECHANISM_H
