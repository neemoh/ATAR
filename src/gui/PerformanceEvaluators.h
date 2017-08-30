//
// Created by nima on 24/08/17.
//

#ifndef ATAR_PERFORMANCEEVALUATORS_H
#define ATAR_PERFORMANCEEVALUATORS_H


#include <kdl/frames.hpp>
#include <tuple>
#include <vector>


class SteadyHandPerfEval
{
public:

    SteadyHandPerfEval(uint n_session, double last_sess_performance);

    void Increment(const KDL::Frame &desired, const KDL::Frame &current);

    double GetPerformanceAndReset(const double time);

    double GetHapticAssistanceActivation(const std::vector<double>
                                         &current_perf_history);
    // resets the history
    void Reset();

private:
    std::tuple<double, double>
    CalculatePoseError(const KDL::Frame &desired, const KDL::Frame &current);

    // calculates and returns the metrics and resest the history;
    void GetEvaluation(double &pos_rms, double &ori_rms,
                       double &pos_mean, double &ori_mean,
                       double &pos_max,double &ori_max);

    double HapticAssistanceFeedForwardTerm(uint session, double past_perf);

private:
    double pos_error_norm_sum;
    double ori_error_norm_sum;
    double pos_error_norm_sum_sqr;
    double ori_error_norm_sum_sqr;
    double pos_err_max;
    double ori_err_max;

    uint counter=0;
    double last_rep_assistance_level;
    double last_session_perf_mean;
    uint n_session;

    double assist_feed_forward_curr;
    double intrasession_delta;

};
#endif //ATAR_PERFORMANCEEVALUATORS_H
