#pragma once

//#include "LAPACK"
#include <cvm.h>

#include "StdAfx.h"

#include "Robo.h"
#include "RoboMovesStore.h"


namespace RoboPos
{
class ApproxI
{
protected:
    static constexpr int MLN = 1000000;

    using Pt = std::pair<double, double>;
    using Row = std::vector<double>;
    using M2I = std::function<size_t(Robo::muscle_t)>;

public:
    //virtual bool learn(/*store*/) = 0;
    virtual Point predict(const Robo::Control&) const = 0;
    virtual bool  clarify(const Robo::Control&, Point) = 0;
    virtual ~ApproxI() {}

    Row substract(const Row &a, const Row &b) const
    {
        if (a.size() != b.size())
            throw std::logic_error("Invalid Row");

        Row res(a.size());
        for (auto i = 0u; i < a.size(); ++i)
        {
            res[i] = a[i] - b[i];
        }
        return res;
    }
    double sq_norm(const Row &row) const
    {
        double res = 0;
        for (double x : row)
            res += x * x;
        return res;
    }
};


class ApproxStore : public ApproxI
{
    RoboMoves::Store &_store;
    cvm::rmatrix equation;
    // Q * X = Y

    const double _t = 1000 * MLN, _n = 1000 * MLN;
    const double _c_k = 17; // калибровочный

    M2I _m2i;
    // x - aргументы функции,
    // y - значения функции
    const size_t _max_controls_size;
    // X row len = _max_controls_size;
    // X n_rows = Y n_rows= _store.size()
    // Y row len = 2;

    std::vector<Row> _X;
    std::vector<Pt> _Y;

    Row q_l1;
    Row q_l2;

    double noize(int i = 0) const { return 12.; }
    Row convertRow(const Robo::Control&) const;

    double K_function(const Row &x) const
    {
        double _sq_norm = sq_norm(x);
        return _c_k * (_sq_norm * std::log(_sq_norm / _t) + _n);
    }
    double y_function(const Row &x, const Row &q) const
    {
        double res = 0;
        for (auto c : _store)
        {
            for (auto i = 0u; i < _max_controls_size; ++i)
                res += q[i] * K_function(substract(x, _X[i]));
        }
        return res;
    }

    void linSystQ(Row &q, bool i);

public:
    ApproxStore(RoboMoves::Store &store, size_t max_controls_size, M2I &m2i) :
        _store(store), _max_controls_size(max_controls_size), _m2i(m2i)
    {}

    Point predict(const Robo::Control&) const;
    bool  clarify(const Robo::Control&, Point);
    bool  learn();

};
}
