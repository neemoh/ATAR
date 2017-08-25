//
// Created by nima on 24/08/17.
//

#ifndef ATAR_PERFORMANCEEVALUATORS_H
#define ATAR_PERFORMANCEEVALUATORS_H


#include <kdl/frames.hpp>

class SteadyHandPerfEval
{

    void Increment();
    void GetEvaluation();

private:
    void CalculatePoseError(const KDL::Frame desired, const KDL::Frame
    current){

        KDL::Frame error = desired * current.Inverse();

        pos_error_norm_sum = error.p.Norm();

//        ori_error_norm_sum = error.M.GetRotAngle();
    };

private:
    double pos_error_norm_sum;
    double ori_error_norm_sum;

};
#endif //ATAR_PERFORMANCEEVALUATORS_H
