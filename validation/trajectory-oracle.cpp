#include <boost/numeric/odeint/stepper/controlled_runge_kutta.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta_dopri5.hpp>
#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <vector>

namespace odeint = boost::numeric::odeint;
unsigned long long trajectories = 0, attempts = 0, checks = 0, mismatches = 0;
void check(bool equal)
{
    ++checks;
    if(!equal) ++mismatches;
}
struct linear_system
{
    double rate;
    template<class State, class Deriv>
    void operator()(const State& state, Deriv& derivative, double time) const
    {
        for(std::size_t i = 0; i < state.size(); ++i)
            derivative[i] = rate * (i + 1) * state[i] + time + 0.1 * i;
    }
};
template<class State>
State state_at(double value)
{
    State state;
    state[0] = value;
    state[1] = -0.75 * value + 0.1;
    return state;
}
template<>
std::vector<double> state_at<std::vector<double> >(double value)
{
    return std::vector<double>{value, -0.75 * value + 0.1};
}
template<class State>
void run(double rate, double value, double start, double step, double tolerance)
{
    typedef odeint::runge_kutta_dopri5<State> error_stepper;
    typedef odeint::controlled_runge_kutta<error_stepper> controlled;
    typedef odeint::default_error_checker<double, typename error_stepper::algebra_type,
        typename error_stepper::operations_type> checker;
    const checker error(tolerance, tolerance);
    controlled out_of_place(error), reference(error), in_place(error);
    State input = state_at<State>(value), inplace_state = input;
    double time = start, reference_time = start, inplace_time = start;
    double dt = step, reference_dt = step, inplace_dt = step;
    linear_system system = {rate};
    ++trajectories;
    for(unsigned i = 0; i < 40 && std::abs(time - start) < 0.5; ++i)
    {
        ++attempts;
        State output = input, reference_output = input;
        State derivative = input, output_derivative = input;
        system(input, derivative, reference_time);
        const auto result = out_of_place.try_step(system, input, time, output, dt);
        const auto expected = reference.try_step(system, input, derivative, reference_time,
            reference_output, output_derivative, reference_dt);
        const auto inplace_result = in_place.try_step(system, inplace_state, inplace_time, inplace_dt);
        check(result == expected);
        check(result == inplace_result);
        check(time == reference_time);
        check(time == inplace_time);
        check(dt == reference_dt);
        check(dt == inplace_dt);
        for(std::size_t j = 0; j < input.size(); ++j)
        {
            check(output[j] == reference_output[j]);
            check((result == odeint::success ? output[j] : input[j]) == inplace_state[j]);
        }
        if(result == odeint::success) input = output;
    }
}
int main()
{
    const double rates[] = {-4.0, -1.0, -0.125, 0.0, 0.125, 1.0, 4.0};
    const double values[] = {-10.0, -0.25, 0.0, 0.25, 10.0};
    const double starts[] = {-1.0, 0.0, 0.5};
    const double steps[] = {1e-6, 1e-3, 0.025, 0.25, 2.0};
    const double directions[] = {-1.0, 1.0};
    const double tolerances[] = {1e-5, 1e-9};
    for(double rate : rates)
        for(double value : values)
            for(double start : starts)
                for(double step : steps)
                    for(double direction : directions)
                        for(double tolerance : tolerances)
                        {
                            run<std::array<double, 2> >(rate, value, start, direction * step, tolerance);
                            run<std::vector<double> >(rate, value, start, direction * step, tolerance);
                        }
    std::cout << "trajectories=" << trajectories << " attempts=" << attempts
              << " checks=" << checks << " mismatches=" << mismatches << "\n";
    return mismatches ? 1 : 0;
}
