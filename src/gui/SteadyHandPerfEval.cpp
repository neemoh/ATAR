//
// Created by nima on 29/08/17.
//

#include <iostream>
#include <algorithm>
#include "SteadyHandPerfEval.h"


#define MAX(a, b) ((a > b) ? a : b)
#define MIN(a, b) ((a < b) ? a : b)




SteadyHandPerfEval::SteadyHandPerfEval(
        uint n_session,
        double last_sess_performance,
        double last_last_sess_performance
    ) {


    this->n_session=n_session;
    last_session_perf_mean = last_sess_performance;

    assist_feed_forward_curr =
            HapticAssistanceFeedForwardTerm(n_session, last_sess_performance);
    double assist_feed_forward_last = 0;
    if(n_session>2)
        assist_feed_forward_last =
                HapticAssistanceFeedForwardTerm(n_session-1, last_last_sess_performance);

    intrasession_delta = assist_feed_forward_curr - assist_feed_forward_last;

    last_rep_assistance_level = assist_feed_forward_curr;
    std::cout << "Assist feed forward last  = " << assist_feed_forward_last
    << ", Assist feed forward curr = " << assist_feed_forward_curr <<
                                                                    std::endl;

}




void SteadyHandPerfEval::Increment(const KDL::Frame &desired, const
KDL::Frame &current){

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


double SteadyHandPerfEval::GetPerformanceAndReset(const double time){

    double pos_rms, ori_rms, pos_mean, ori_mean, pos_max, ori_max;

    GetEvaluation(pos_rms, ori_rms, pos_mean, ori_mean,
                  pos_max, ori_max);

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

    double perf = (2*normalized_pos_rms + normalized_ori_rms +
        2*normalized_pos_max+ 2*normalized_time) / 7.0;

    std::cout << "pos_rms: " << pos_rms
              << ", pos_max: "<< pos_max
              << ", ori_rms: "<< ori_rms
              << ", repetition_time: "<< time
              << ", perf: "<< perf << std::endl;

    return perf;

}

double SteadyHandPerfEval::GetHapticAssistanceActivation(const
                                                         std::vector<double>
                                                         &perf_history) {

    double lambda = 0.8;
    // first element of perf_history is the mean perf of last session
    double n_reps_done = perf_history.size();

    double perf_mean_current = 0.0;
    if(n_reps_done>0)
        perf_mean_current = std::accumulate(perf_history.begin(),
                                               perf_history.end(), 0.0f)/n_reps_done;

    double perf = (8*last_session_perf_mean + 2*n_reps_done* perf_mean_current)/
            (8+2*n_reps_done);

    double tfd = 1- lambda*perf - (1-lambda)*last_rep_assistance_level;

    double assistance_act =
            MAX(0, assist_feed_forward_curr - intrasession_delta*(2*tfd-1));

    std::cout << "-------- Perf mean curr session= " << perf_mean_current <<
                                                                    std::endl;
    std::cout << "Perf mean incl last session= " << perf << ", TFD = " << tfd
              << ", Assistance activation = " << assistance_act << std::endl;

    last_rep_assistance_level = assistance_act;
    return assistance_act;

}


// resets the history
void SteadyHandPerfEval:: Reset(){

    pos_error_norm_sum = 0.0;
    ori_error_norm_sum = 0.0;
    pos_error_norm_sum_sqr = 0.0;
    ori_error_norm_sum_sqr = 0.0;
    pos_err_max = 0.0;
    ori_err_max = 0.0;
    counter = 0;
}

std::tuple<double, double>
SteadyHandPerfEval::CalculatePoseError(const KDL::Frame &desired, const
KDL::Frame &current){


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
void SteadyHandPerfEval::GetEvaluation(double &pos_rms, double &ori_rms,
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

double SteadyHandPerfEval::HapticAssistanceFeedForwardTerm(uint session,
                                                           double past_perf) {
    double sessions_max = 10;
    double sessions_min = 6;
    double sessions_full_assis = 2;

    double ff = 1;
    if(session > sessions_full_assis) {
        ff = 1 - (double(session) - sessions_full_assis)
                 / (sessions_max - past_perf * (sessions_max - sessions_min)
                    - sessions_full_assis);
    }

    return ff;
}

