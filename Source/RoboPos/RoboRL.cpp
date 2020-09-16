#include "StdAfx.h"

#include <algorithm>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <string>
#include <random>
#include <gsl/gsl_vector.h>

#include "rl.hpp" //https://github.com/HerveFrezza-Buet/RLlib
//#include "rlAlgo.hpp"

#include "RoboRL.h"
#include "RoboRLSim.h"
#include "RoboMovesStore.h"
#include "RoboMovesTarget.h"

using namespace std::placeholders;
using namespace Robo;
using namespace RoboMoves;

namespace rl_problem {
//------------------------------------------------------
template <typename P, typename D>
class GaussFunc final
{
protected:
    P center{};
    D width{};
public:
    GaussFunc() {}
    template <typename P, typename D>
    GaussFunc(const P &c, D w) : center(c), width(w) {}
    GaussFunc(const GaussFunc<P, D> &gf) {}
    GaussFunc<P, D>& operator=(const GaussFunc<P, D> &gf)
    {
        if (*this != gf)
        {
            center = gf.center;
            width = gf.width;
        }
        return *this;
    }
};
//------------------------------------------------------
template <typename ...> class RGD;

/// For Q-function
template <typename S, typename A>
class RGD<S, A> : public rl::gsl::TD<S, A>
{
    template <typename fctQ, typename fctGRAD_Q>
    RGD(gsl_vector* theta,
        double gamma_coef,
        double alpha_coef,
        const fctQ &fct_q,
        const fctGRAD_Q &fct_grad_q) :
        TD<S, A>(theta, gamma_coef, alpha_coef, fct_q, fct_grad_q)
    {}
    RGD(const RGD<S, A> &cp) : TD<S, A>(cp) {}
    ~RGD() /*override */ {}

    //RGD<S,A>& operator=(const RGD<S,A> &cp) override;

    //double td_error(const S &s, const A &a, double r, const S &s_, const A &a_) override
    //{ return r + gamma * q(theta, s_, a_) - q(theta, s, a); }
    //double td_error(const S &s, const A &a, double r) override
    //{ return r - q(theta, s, a); }
    //
    //void learn(const S &s, const A &a, double r, const S &s_, const A &a_) override
    //{ this->td_update(s, a, this->td_error(s, a, r, s_, a_)); }
    //void learn(const S &s, const A &a, double r) override
    //{ this->td_update(s, a, this->td_error(s, a, r)); }
};

/// For Value Function
template <typename S>
class RGD<S> : public rl::gsl::TD<S>
{
    template <typename fctV, typename fctGRAD_V>
    RGD(gsl_vector* theta,
        double gamma_coef,
        double alpha_coef,
        const fctV &fct_v,
        const fctGRAD_V &fct_grad_v) :
        TD<S>(theta, gamma_coef, alpha_coef, fct_v, fct_grad_v)
    {}
    RGD(const RGD<S> &cp) : TD<S>(cp) {}
    virtual ~RGD() /*override */ {}

    //TD<STATE>& operator=(const TD<STATE>& cp);
    //double td_error(const STATE& s, double r, const STATE& s_) override
    //{
    //    return r + gamma * v(theta, s_) - v(theta, s);
    //}
    //double td_error(const STATE& s, double r) override
    //{
    //    return r - v(theta, s);
    //}
    //
    //void learn(const STATE& s, double r, const STATE& s_) override
    //{
    //    this->td_update(s, this->td_error(s, r, s_));
    //}
    //void learn(const STATE& s, double r) override
    //{
    //    this->td_update(s, this->td_error(s, r));
    //}
};
//------------------------------------------------------
class Qfunction final
{
public:
    static constexpr int N_EPISODES = 400; // 10000;
    static constexpr int MAX_EPISODE_DURATION = 1000;
    static constexpr int EPISODE_FRAME_PERIOD = 10;
    //static constexpr int MIN_V = -50;
    static constexpr int PHI_DIRECT_DIMENSION = 4/*musclesCount*/ + 2/*jointsCount*/ * 3/*p_v_a*/ * 2/*x_y*/;

    static constexpr double paramALPHA = 0.05;
    static constexpr double paramGAMMA = 0.95;
    static constexpr double paramEPSILONT = 0.1;// ObservationRobo::EpsVel;
    static constexpr double paramSIGMOID_COEF = 0.1;
    static constexpr double paramETA_NOISE = 0.;
    static constexpr double paramOBSERVATION_NOISE = 1e-4;
    static constexpr double paramPRIOR_VAR = 0.3162278; // sqrt(1e-1);
    static constexpr double paramRANDOM_AMPLITUDE = 1e-1;
    static constexpr double paramUT_ALPHA = 1e-2;
    static constexpr double paramUT_BETA = 2;
    static constexpr double paramUT_KAPPA = 0;
    static constexpr double paramUSE_LINEAR_EVALUATION = false; // a MLP is not a linear architecture

    using TransferF = double(double);
    using Transfer = std::function<double(double)>;
    using Input = rl::gsl::mlp::Input<S, A, void(gsl_vector*, const S&, const A&)>;
    using Hidden1 = rl::gsl::mlp::Hidden<Input, Transfer>;
#define LAYER2
#ifdef LAYER2
    using Hidden2 = rl::gsl::mlp::Hidden<Hidden1, Transfer>;
    using Output = rl::gsl::mlp::Output<Hidden2, TransferF>;
#else
    using Output = rl::gsl::mlp::Output<Hidden1, TransferF>;
#endif

    //std::mt19937  rand_gen;
    Transfer sigmoid;
    Input    input_layer;
    Hidden1  hidden_layer_1;
#ifdef LAYER2
    Hidden2  hidden_layer_2;
#endif
    Output   q_parametrized;
    gsl_vector *theta;

    Qfunction(/*double alpha, double gamma, double Csigmoid*/) :
        //rand_gen(std::random_device{}()),
        //paramALPHA(alpha),
        //paramGAMMA(gamma),
        //paramSIGMOID_COEF(Csigmoid),
        // setup the Q-function approximator as a ANN-RBFs
        sigmoid(std::bind(rl::transfer::tanh, std::placeholders::_1, paramSIGMOID_COEF)),
        input_layer(rl::gsl::mlp::input<S, A>(phi_direct, PHI_DIRECT_DIMENSION)),
        hidden_layer_1(rl::gsl::mlp::hidden(input_layer, /*1*/5, sigmoid)),
#ifdef LAYER2
        hidden_layer_2(rl::gsl::mlp::hidden(hidden_layer_1, /*5*/3, sigmoid)),
        q_parametrized(rl::gsl::mlp::output(hidden_layer_2, rl::transfer::identity)),
#else
        q_parametrized(rl::gsl::mlp::output(hidden_layer_1, rl::transfer::identity)),
#endif
        theta(gsl_vector_alloc(q_parametrized.size))
    {
        gsl_vector_set_zero(theta);
    }
    ~Qfunction()
    {
        gsl_vector_free(theta);
    }

    friend std::istream& operator>>(std::istream &s, Qfunction &q)
    {
        char c;
        s >> c/*{*/ /*>> sigmoid*/ >> q.input_layer >> q.hidden_layer_1;
#ifdef LAYER2
        s >> q.hidden_layer_2;
#endif
        s >> q.q_parametrized >> q.theta >> c/*}*/;
        return s;
    }
    friend std::ostream& operator<<(std::ostream &s, const Qfunction &q)
    {
        s << '{' /*<< sigmoid << ' '*/ << q.input_layer << ' ' << q.hidden_layer_1 << ' ';
#ifdef LAYER2
        s << q.hidden_layer_2 << ' ';
#endif
        s << q.q_parametrized << ' ' << q.theta << '}' << std::endl;
        return s;
    }

    auto getQfunc() const
    {
        return std::bind(//&Output::operator(),
                         q_parametrized,
                         theta,
                         std::placeholders::_1,
                         std::placeholders::_2) /*Q(s,a)*/;
    }
    auto getGradQfunc() const
    {
        return [](S /*s*/, A /*a*/) { return 0.; } /*grad Q(s,a)*/;
    }
};

//------------------------------------------------------
// display the length
void displayTestEpLength(int step, int episode, int length)
{
    std::cout << "\r Step " << std::setw(4) << std::setfill('0') << step
        << " : " << std::setfill('.') << std::setw(4) << episode + 1
        << " length = " << std::setw(10) << std::setfill(' ') << length
        << std::flush;
}

//------------------------------------------------------
// display the mean length
void displayTestEpMeanLength(int step, double mean_length)
{
    std::cout << "\r Step " << std::setw(4) << std::setfill('0') << step
        << " : mean length = "
        << std::setw(10) << std::setfill(' ') << std::setprecision(2) << mean_length
        << std::endl;
}

//------------------------------------------------------
template <
    typename SIMULATOR,
    typename POLICY,
    typename RANDOM_GENERATOR,
    int NB_LENGTH_SAMPLES,
    int MAX_EPISODE_LENGTH
>
void test_iteration(SIMULATOR simulator, const POLICY& policy, int step, RANDOM_GENERATOR& gen)
{
    double mean_length = 0.;
    for (int episode = 0; episode < NB_LENGTH_SAMPLES; ++episode)
    {
        Simulator::phase_type start_phase;
        start_phase.random(gen);
        simulator.setPhase(start_phase);
        // generate an episode and get its length
        int length = rl::episode::run(simulator, policy, MAX_EPISODE_LENGTH);
        displayTestEpLength(step, episode, length);
        // mean update
        mean_length += length;
    }
    mean_length /= NB_LENGTH_SAMPLES;
    displayTestEpMeanLength(step, mean_length);
}

//------------------------------------------------------
template <
    typename Simulator,
    typename Critic,
    typename LearningPolicy,
    typename TestPolicy,
    typename Q,
    typename V,
    typename RandGen
>
void make_experiment(Simulator &simulator, Critic &critic, const Q &q, const V &v,
                     LearningPolicy &learning_policy, TestPolicy &test_policy,
                     RandGen &rand, Robo::StateTrajectories &trajs)
{
    //std::cout << std::endl << std::endl;
    for (int episode = 0, frame = 0; episode < Qfunction::N_EPISODES; ++episode)
    {
        std::cout << "running episode " << std::setw(6) << episode + 1 << "/" << Qfunction::N_EPISODES << "    \r" << std::flush;

        simulator.restart();
        auto actual_episode_length = rl::episode::learn(simulator, learning_policy, critic, Qfunction::MAX_EPISODE_DURATION);
        //store.insert(Record{ robo.position(), start, robo.position(), controls, robo.trajectory() });

        if (episode % Qfunction::EPISODE_FRAME_PERIOD == 0)
        {
            //StateTrajectory visited;
            //// Let us run an episode with a greedy policy and mark the states as visited.
            //VisitNotifier<S> visit_notifier(visited);

            simulator.restart();
            rl::episode::run(simulator,
                             test_policy,
                             //visit_notifier,
                             //[](S s, A a, Reward r, S s_) -> S { return s; },
                             //[](S s, A a, Reward r)       -> S { return s; },
                             Qfunction::MAX_EPISODE_DURATION);
            trajs.push_back(simulator.trajectory());

            //Cliff::draw_visited("rllib", frame++, v,
            //                    [&visit_notifier](S s) -> bool { return visit_notifier.visited[s]; },
            //                    MIN_V, 0);
        }
    }
}

} // end namespace rl_problem

//------------------------------------------------------
namespace RoboPos {
void RoboRL(Store&, RoboI&, TargetI&, StateTrajectories&);
}

//------------------------------------------------------
void RoboPos::RoboRL(Store &/*store !!!*/, RoboI &robo, TargetI &target, StateTrajectories &show_trajs)
{
    using namespace rl_problem;
    std::random_device rd;
    std::mt19937  rand_gen(rd());

    auto actionBegin = rl::enumerator<A>(0);
    auto actionEnd = rl::enumerator<A>(robo.musclesCount());

    Qfunction Q;

    std::string qfn("qnn.dat");
    if (boost::filesystem::exists(qfn))
    {
        std::ifstream fin(qfn);
        fin >> Q;
    }

    auto q = Q.getQfunc();
    //auto gq = Q.getGradQfunc();

    // display the structure of our MLP...
    std::cout << std::endl;
    Q.q_parametrized.displayParameters(std::cout);
    std::cout << std::endl;

    std::function<double(S)> v = [&actionBegin, &actionEnd, &q](S s) -> double {
        return rl::max(std::bind(q, s, std::placeholders::_1),
                       actionBegin,
                       actionEnd);
    }; // V(s) = max_a q(s,a)

    //auto      q = std::bind(q_parametrized, theta, _1, _2);
    //auto critic = rl::gsl::q_learning<S, A>(theta,
    //                                        paramGAMMA, paramALPHA,
    //                                        action_begin, action_end,
    //                                        q_parametrized,
    //                                        grad_q_parametrized);

    //auto critic = rl::gsl::q_learning<S,A>(Q.theta,
    //                                       Q.paramGAMMA,
    //                                       Q.paramALPHA,
    //                                       actionBegin,
    //                                       actionEnd,
    //                                       q,
    //                                       gq);

    //auto phi = [](gsl_vector *phi, const S& s, const A& a) { return; };

    auto critic = rl::gsl::ktd_q<S, A>(Q.theta,
                                       Q.q_parametrized, actionBegin, actionEnd,
                                       Q.paramGAMMA,
                                       Q.paramETA_NOISE,
                                       Q.paramOBSERVATION_NOISE,
                                       Q.paramPRIOR_VAR,
                                       Q.paramRANDOM_AMPLITUDE,
                                       Q.paramUT_ALPHA,
                                       Q.paramUT_BETA,
                                       Q.paramUT_KAPPA,
                                       Q.paramUSE_LINEAR_EVALUATION,
                                       rand_gen);

    // ==============================
    boost::this_thread::interruption_point();
    // ==============================
    double epsilont = 0.25; // Qfunction::paramEPSILONT;
    auto random_policy = rl::policy::random(rl::enumerator<A>(1), actionEnd, rand_gen);
    //auto learning_policy = rl::policy::epsilon_greedy(q, epsilont, actionBegin, actionEnd, rand_gen);
    auto test_policy = rl::policy::greedy(q, actionBegin, actionEnd);

    auto learning_policy1 = [&rand_gen, q, &epsilont, actionBegin, actionEnd](const S &s)
        -> typename std::remove_reference<decltype(*actionBegin)>::type {
        std::bernoulli_distribution dis(epsilont);
        if (dis(rand_gen))
        {
            typename std::remove_reference<decltype(*actionBegin)>::type selected_value;
            std::sample(actionBegin, actionEnd, &selected_value, 1, rand_gen);
            return selected_value;
        }
        // ==============================
        boost::this_thread::interruption_point();
        // ==============================
        return rl::argmax(std::bind(q, s, std::placeholders::_1), actionBegin, actionEnd).first;
    };

    //S s(Point{}, 3, 0);
    //auto aaa = learning_policy(s);

    auto temparature = 1.;
    // Температура достаточно важный параметр функции SoftMax, она показывает насколько детерменирована последовательность
    // Более высокие значения дают более шумный и стохастический выход, низкие могут повторять вход с незначительными изменениями
    auto learning_policy = rl::policy::softmax(q, temparature, actionBegin, actionEnd, rand_gen);

    std::ofstream fqlog("qnn.log", std::ios::app);
    fqlog << std::string(5, '-') << ' ' << Utils::now() << ' ' << std::string(25, '-') << std::endl;
    Point::w = 6;
    Point::prec = 2;

    //for (auto &goal : target.coords())
    {
        robo.reset();
        Point base = robo.position();
        Point pos = robo.position();
        Point goal = *std::/*min*/max_element(target.coords().begin(),
                                              target.coords().end(),
                                              [&pos](const Point &a, const Point &b) { return (boost_distance(a, pos) < boost_distance(b, pos)); });

        rl_problem::SimRobo simulator(robo, goal);
        ////Store &store, RoboI &robo
        //make_experiment(simulator, 
        //                critic, 
        //                learning_policy,
        //                test_policy,
        //                q, 
        //                v, 
        //                rand_gen,
        //                show_trajs);
        bool stop = false;
        Robo::distance_t d = boost_distance(base, goal);

#define RL_DUMP
#ifdef RL_DUMP
        auto Now = Utils::now();
        std::ofstream mid_dist("mid-dist-" + Now + ".plt");
        mid_dist << "plot '-' using 1:2 with lines" << std::endl;

        std::ofstream mid_rw("mid-rw-" + Now + ".plt");
        mid_rw << "plot '-' using 1:2 with lines" << std::endl;

        std::ofstream fgoal("mid-goal-" + Now + ".plt");
        fgoal << "plot '-' using 1:2 with lines" << std::endl;

        unsigned time_rw = 0, time_dst = 0, time_g = 0, count_dst = 0;
        double rws = 0., dists = 0., dist_g = 0.;
        auto plot_printer = [](auto &stream, unsigned &time, double item) {
            stream << time++ << '\t' << item << std::endl;
        };
#endif
        unsigned traj_count = 0, all_count = 0;
        //while (boost_distance(base, pos) < 0.0001 && !stop)
        for (unsigned i = 0; i < 100 && !stop; ++i)
        {
            for (int episode = 0/*, frame = 0*/; episode < Qfunction::N_EPISODES; ++episode)
            {
                ++all_count;
                std::vector<int> controls;
                controls.reserve(Qfunction::MAX_EPISODE_DURATION);

                std::cout << "running episode " << std::setw(6) << episode + 1 << "/" << Qfunction::N_EPISODES << "    \r" << std::flush;

                simulator.restart();
                controls.clear();
                // episode
                unsigned length = 0;
                S s = simulator.sense();
                A a = random_policy(s); //learning_policy(s);
                try
                {
                    do
                    {
                        ++length;
                        //sa = rl::episode::adaptation(simulator, policy, critic, sa.first, sa.second);
                        simulator.timeStep(a);
                        // ------------------
                        controls.push_back(a);
#ifdef RL_DUMP
                        rws += simulator.reward();
#endif
                        // ------------------
                        S next = simulator.sense();
                        A a_next = learning_policy(next); //(length < 10) ? random_policy(s) : learning_policy(s);
                        critic.learn(s, a, simulator.reward(), next, a_next);
                        // ==============================
                        boost::this_thread::interruption_point();
                        // ==============================
                        a = a_next;
                        s = next;
                    } while (length != Qfunction::MAX_EPISODE_DURATION);
#ifdef RL_DUMP
                    plot_printer(mid_rw, time_rw, rws / length);
#endif
                }
                catch (boost::thread_interrupted&)
                {
                    CINFO("WorkingThread interrupted!");
                    stop = true;
                    break;
                }
                catch (rl::exception::Terminal&)
                {
                    critic.learn(s, a, simulator.last_reward());
                    controls.push_back(a);
#ifdef RL_DUMP
                    rws += simulator.last_reward();
                    plot_printer(mid_rw, time_rw, rws / length);
#endif
                }
                pos = robo.position();
                double dd = boost_distance(pos, goal);
#ifdef RL_DUMP
                dist_g += dd;
#endif
                if (d > dd) //simulator.trajectory().size() == 100)
                {
                    d = dd;
                    show_trajs.push_back(simulator.trajectory());
                    int j = 0;
                    auto qlog_printer = [&j, &fqlog](auto c) {
                        if (c)
                        {
                            std::cout << j << ':' << c << ' ';
                            fqlog << j << ':' << c << ' ';
                        }
                        ++j;
                    };
                    std::for_each(controls.begin(), controls.end(), qlog_printer);
                    fqlog << std::endl;

                    std::cout << std::string(40, ' ') << std::endl;
                    //std::cout  << "    \r" << std::flush;
                    //controls.clear();

                    Point prev = simulator.trajectory().begin()->spec();
                    Robo::distance_t traj_dist = 0.;
                    for (auto &p : simulator.trajectory())
                    {
                        //std::cout << p.spec() << std::endl;
                        traj_dist += boost_distance(prev, p.spec());
                        prev = p.spec();
                    }

                    traj_count++;
                    fqlog <<
                        "goal=" << goal <<
                        " pos=" << pos <<
                        " dist=" << std::setw(12) << std::setprecision(5) << d <<
                        " traj_dist=" << std::setw(12) << std::setprecision(5) << traj_dist << 
                        ' ' << std::setw(10) << simulator.trajectory().size() <<
                        " " << std::setw(10) << traj_count << '/' << all_count
                        << std::endl;
#ifdef RL_DUMP
                    ++count_dst;
                    dists += traj_dist;
#endif
                }

                if (0)//(episode + 1) % Qfunction::EPISODE_FRAME_PERIOD == 0)
                {
                    simulator.restart();
                    rl::episode::run(simulator, test_policy, Qfunction::MAX_EPISODE_DURATION);

                    Robo::distance_t traj_dist = 0.;
                    if (simulator.trajectory().size() > 2)
                    {
                        show_trajs.push_back(simulator.trajectory());
                        Point prev = show_trajs.back().begin()->spec();
                        for (auto &hit : show_trajs.back())
                        {
                            traj_dist += boost_distance(prev, hit.spec());
                            prev = hit.spec();
                        }
                        // -------------------------------------
                        //boost::this_thread::interruption_point();
                        // -------------------------------------
                        pos = robo.position();
                        std::cout <<
                            "goal=" << goal <<
                            " pos=" << pos <<
                            " dist=" << boost_distance(pos, goal) <<
                            " traj_dist=" << traj_dist << ' ' << show_trajs.back().size()
                            << std::endl;
                        // -------------------------------------
                        fqlog <<
                            "goal=" << goal <<
                            " pos=" << pos <<
                            " dist=" << boost_distance(pos, goal) <<
                            " traj_dist=" << traj_dist << ' ' << show_trajs.back().size()
                            << std::endl;
                    }
                }
            }

#ifdef RL_DUMP
            plot_printer(mid_dist, time_dst, dists / count_dst);
            plot_printer(fgoal, time_g, dist_g / Qfunction::N_EPISODES);
            count_dst = 0;
#endif
        } // while goal
        //break;
    }
    
    std::ofstream fout(qfn);
    fout << Q;
}

