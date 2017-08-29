//
// Created by nima on 24/08/17.
//

#ifndef ATAR_PERFORMANCEEVALUATORS_H
#define ATAR_PERFORMANCEEVALUATORS_H


#include <kdl/frames.hpp>
#include <tuple>

//#define MAX(a, b) ((a > b) ? a : b)

class SteadyHandPerfEval
{
public:

    SteadyHandPerfEval(uint n_session){
        this->n_session=n_session;
    };

    void Increment(const KDL::Frame &desired, const KDL::Frame &current){

        counter++;

        auto pose_err_norm = CalculatePoseError(desired, current);

        // add to the sum
        pos_error_norm_sum += std::get<0>(pose_err_norm);
        ori_error_norm_sum += std::get<1>(pose_err_norm);

        //add to sum of squared
        pos_error_norm_sum_sqr += pow(std::get<0>(pose_err_norm), 2);
        ori_error_norm_sum_sqr += pow(std::get<1>(pose_err_norm), 2);

        // check max
        pos_err_max = MAX(std::get<0>(pose_err_norm), pos_err_max);
        ori_err_max = MAX(std::get<1>(pose_err_norm), ori_err_max);

    }


    double GetPerformanceAndReset(const double time){

        double pos_rms, ori_rms, pos_mean, ori_mean, pos_max, ori_max;

        GetEvaluation(pos_rms, ori_rms, pos_mean, ori_mean,
                                pos_max, ori_max);

        std::cout << "pos_rms: " << pos_rms
                  << ", pos_max: "<< pos_max
                  << ", ori_rms: "<< ori_rms
                  << ", repetition_time: "<< time << std::endl;

        double pos_rms_ideal = 0.001;
        double ori_rms_ideal = 20*M_PI/180;
        double pos_max_ideal = 0.0028;
        double time_ideal = 64;

        double pos_rms_worst = 0.0024;
        double ori_rms_worst = 57*M_PI/180;
        double pos_max_worst = 0.004;
        double time_worst = 121;

        // calculate the magnitude
        double normalized_pos_rms=
        MAX(0, MIN(1,
                   (pos_rms_worst - pos_rms)/ (pos_rms_worst - pos_rms_ideal)));

        double normalized_ori_rms=
        MAX(0, MIN(1,
                   (ori_rms_worst - ori_rms)/ (ori_rms_worst - ori_rms_ideal)));
        double normalized_pos_max=
        MAX(0, MIN(1,
                   (pos_max_worst - pos_max)/ (pos_max_worst - pos_max_ideal)));
        double normalized_time=
        MAX(0, MIN(1,
                   (time_worst - time)/ (time_worst - time_ideal)));

        return (normalized_pos_rms + normalized_ori_rms +
            normalized_pos_max+ normalized_time) / 4.0;

    }

    double GetHapticAssistanceActivation(const std::vector<double>
                                         &perf_history) {


        // calculate the magnitude
        double magnitude=0;

        return magnitude;

    }


    // resets the history
    void Reset(){

        pos_error_norm_sum = 0.0;
        ori_error_norm_sum = 0.0;
        pos_error_norm_sum_sqr = 0.0;
        ori_error_norm_sum_sqr = 0.0;
        pos_err_max = 0.0;
        ori_err_max = 0.0;
        counter = 0;
    }

private:
    std::tuple<double, double>
    CalculatePoseError(const KDL::Frame &desired, const KDL::Frame &current){


        //double pos_error_norm = error.p.Norm();
        double pos_error_norm = (desired.p - current.p).Norm();

        // calculate the orientation error as the absolute angle of rotation
        // to the desired pose
        KDL::Vector axis;
        double r,p,y;
        (desired.M * current.M.Inverse()).GetRPY(r,p,y);
        double ori_error_norm_rpy = sqrt(r*r + p*p + y*y);

        return std::make_tuple(pos_error_norm, ori_error_norm_rpy);

    };


    // calculates and returns the metrics and resest the history;
    void GetEvaluation(double &pos_rms, double &ori_rms,
                       double &pos_mean, double &ori_mean,
                       double &pos_max,double &ori_max){

        pos_rms = sqrt(pos_error_norm_sum_sqr / double(counter));
        ori_rms = sqrt(ori_error_norm_sum_sqr / double(counter));

        pos_mean = (pos_error_norm_sum / double(counter));
        ori_mean = (ori_error_norm_sum / double(counter));

        pos_max = pos_err_max;
        ori_max = ori_err_max;

        Reset();
    }


private:
    double pos_error_norm_sum;
    double ori_error_norm_sum;
    double pos_error_norm_sum_sqr;
    double ori_error_norm_sum_sqr;
    double pos_err_max;
    double ori_err_max;
    uint counter=0;
    uint n_session;

};
#endif //ATAR_PERFORMANCEEVALUATORS_H
