#include "StdAfx.h"

//#include <Eigen/Eigen>

#include "Robo.h"
#include "RoboMovesTarget.h"
#include "RoboMovesStore.h"
#include "RoboLearnMoves.h"
#include "Test/Test.h"

using namespace Robo;
using namespace RoboPos;
using namespace RoboMoves;

//using d_vector = std::vector<double>;
//
//struct colvec {};
//struct matrix {};
//
//struct normal
//{
//    double scale;
//    double translate;
//};
//
////Eigen::col
//
//colvec vec2colvec(d_vector &vec)
//{
//    int length = vec.size();
//    colvec A(length);
//    for (int i = 0; i < length; i++)
//        A(i) = vec[i];
//    return A;
//}
//
//matrix vec2mat(vector<vector<double> >&vec)
//{
//    int col = vec.size();
//    int row = vec[0].size();
//
//    matrix A(row, col);
//    for (int i = 0; i < col; i++)
//        for (int j = 0; j < row; j++)
//            A(j, i) = vec[i][j];
//    return A;
//}
//
//
//normal getScale(vector<double> &vec, double newmin, double newmax)
//{
//    double max, min;
//    for (int i = 0; i < vec.size(); i++)
//
//        if (i == 0)
//        {
//            max = vec[i];
//            min = vec[i];
//        }
//        else
//        {
//            if (vec[i] > max)
//                max = vec[i];
//            if (vec[i] < min)
//                min = vec[i];
//        }
//    normal result;
//    result.translate = (max - min) / 2 - max;
//    result.scale = (max - min) / (newmax - newmin);
//    return result;
//}
//
//colvec BatchGradientDescent(vector<vector<double> >&vecX, vector<double>& vecY)
//{
//    int nfeatures = vecX.size() + 1;
//    int nsamples = vecX[0].size();
//    //add X0 = 1.
//    vector<double> tempvec;
//    for (int i = 0; i < vecX[0].size(); i++)
//        tempvec.push_back(1.0);
//
//    vecX.insert(vecX.begin(), tempvec);
//
//    /*
//    //try to normalize vecX
//    //X0 should not be normalized.
//    //first calculate the scale and translate
//    vector<normal> nor;
//    normal tpnor;
//    tpnor.scale = 1.0;
//    tpnor.translate = 0.0;
//    nor.push_back(tpnor);
//    for(int i = 1; i<nfeatures; i++) {
//        tpnor = getScale(vecX[i], -1.0, 1.0);
//        nor.push_back(tpnor);
//    }
//    //now do scale and translate
//    for(int i=1; i<nfeatures; i++)
//        for(int j=0; j<nsamples; j++){
//            vecX[i][j] += nor[i].translate;
//            vecX[i][j] /= nor[i].scale;
//        }
//      cout<<nor[1].translate<<", "<<nor[1].scale<<endl;
//    */
//
//    //change vecX and vecY into matrix or vector
//    colvec y = vec2colvec(vecY);
//    matrix x = vec2mat(vecX);
//    //set learning rate;
//    double lrate = 0.0001;
//    //build theta vector, and randomize initial values
//    colvec theta(nfeatures);
//    for (int i = 0; i < nfeatures; i++)
//        theta(i) = 0.0;
//    colvec thetatemp;
//    int counter = 0;
//    while (1)
//    {
//        rowvec thetaT = theta.t();
//        colvec descent(nfeatures);
//        for (int j = 0; j < nfeatures; j++)
//        {
//            double sum = 0.0;
//            for (int i = 0; i < nsamples; i++)
//            {
//                rowvec xit = x.row(i);
//                colvec xi = xit.t();
//                double h = as_scalar(thetaT * xi);
//                double errortemp = y(i) - h;
//                errortemp *= xi(j);
//                sum += errortemp;
//            }
//            sum *= lrate;
//            descent(j) = sum;
//        }
//        thetatemp = theta + descent;
//        cout << "************** round " << counter << endl << theta << endl;
//
//        int converge = 0;
//        for (int i = 0; i < nfeatures; i++)
//            if (fabs(theta(i) - thetatemp(i)) / theta(i) > 0.00001)
//                break;
//            else ++converge;
//        if (converge == nfeatures || ++counter > 100000)
//            return thetatemp;
//        theta = thetatemp;
//    }
//}
//
//colvec StochasticGradientDescent(vector<vector<double> >&vecX, vector<double>& vecY)
//{
//    int nfeatures = vecX.size() + 1;
//    int nsamples = vecX[0].size();
//    //add X0 = 1.
//    vector<double> tempvec;
//    for (int i = 0; i < vecX[0].size(); i++)
//        tempvec.push_back(1.0);
//    vecX.insert(vecX.begin(), tempvec);
//
//    /*
//    //try to normalize vecX
//    //X0 should not be normalized
//    //first calculate the scale and translate
//    vector<normal> nor;
//    normal tpnor;
//    tpnor.scale = 1.0;
//    tpnor.translate = 0.0;
//    nor.push_back(tpnor);
//    for(int i = 1; i<nfeatures; i++)
//    {
//        tpnor = getScale(vecX[i], -1.0, 1.0);
//        nor.push_back(tpnor);
//    }
//    //now do scale and translate
//    for(int i=1; i<nfeatures; i++){
//        for(int j=0; j<nsamples; j++){
//            vecX[i][j] += nor[i].translate;
//            vecX[i][j] /= nor[i].scale;
//        }
//    }
//      cout<<nor[1].translate<<", "<<nor[1].scale<<endl;
//    */
//
//    //change vecX and vecY into matrix or vector
//    colvec y = vec2colvec(vecY);
//    matrix x = vec2mat(vecX);
//
//    //set learning rate
//    double lrate = 0.0001;
//    //build theta vector, and randomize initial values
//    colvec theta(nfeatures);
//    colvec thetatemp(nfeatures);
//
//    for (int i = 0; i < nfeatures; i++)
//        theta(i) = 0.;
//
//    int counter = 0;
//    while (1)
//    {
//        thetatemp = theta;
//        colvec descent(nfeatures);
//        for (int j = 0; j < nfeatures; j++)
//        {
//            int i = 0;
//            for (i = 0; i < nsamples; i++)
//            {
//                double thetaj = theta(j);
//                rowvec thetaT = theta.t();
//                rowvec xit = x.row(i);
//                colvec xi = xit.t();
//                double h = as_scalar(thetaT * xi);
//                double errortemp = y(i) - h;
//                errortemp *= xi(j);
//                theta(j) += lrate * errortemp;
//                if (fabs(theta(j) - thetaj) / theta(j) <= 0.00001)
//                    break;
//            }
//            cout << "j = " << j << ", i = " << i << endl;
//        }
//        cout << "************** round " << counter << endl << theta << endl;
//
//        int converge = 0;
//        for (int i = 0; i < nfeatures; i++)
//        {
//            if (fabs(theta(i) - thetatemp(i)) / theta(i) > 0.00001)
//                break;
//            else ++converge;
//        }
//        if (converge == nfeatures || ++counter > 100000)
//            return theta;
//    }
//}
//
//colvec NormalEquation(vector<vector<double> >&vecX, vector<double>& vecY)
//{
//    //add X0 = 1.
//    vector<double> tempvec;
//    for (int i = 0; i < vecX[0].size(); i++)
//        tempvec.push_back(1.0);
//    vecX.insert(vecX.begin(), tempvec);
//    colvec y = vec2colvec(vecY);
//    matrix x = vec2mat(vecX);
//    matrix xT = x.t();
//    matrix temp;
//    temp = xT * x;
//    temp = pinv(temp);
//    colvec result = temp * xT * y;
//    return result;
//}

//#include "dlib/optimization.h"
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

/*!  схлопнуть управления в вектор, упорядоченный по номеру мускула,
*    сливая управления для одного мускула(в разное время запуска), удлинняя длительности
*/
void create_actuators_vector(OUT std::vector<Actuator> &v, IN const Control &c, IN const muscle_t n_muscles)
{
    v.resize(n_muscles);
    for (auto &a : c)
        if (v[a.muscle].muscle == MInvalid)
            v[a.muscle] = a;
        else
        {
            v[a.muscle].lasts += a.lasts;
            v[a.muscle].start += a.start;
            v[a.muscle].lasts /= 2;
            v[a.muscle].start /= 2;
        }
    //create_actuators_vector(inits, inits_controls, n_muscles);
    //create_actuators_vector(lower, lower_controls, n_muscles);
    //create_actuators_vector(upper, upper_controls, n_muscles);
    //Control inits(inits_controls);
    //const Control &lower = lower_controls;
    //const Control &upper = upper_controls;
}

//------------------------------------------------------------------------------
/// расположить управления в вектор, упорядоченный по номеру мускула (НЕ сливая управления для одного мускула по длительности)
void layout_controls(OUT std::vector<Actuator> &v, IN const Control &c, IN const muscle_t n_muscles)
{
    v.resize(n_muscles);
    for (auto &a : c)
    {
        muscle_t m = a.muscle;
        while (v[m].muscle != MInvalid)
            m += n_muscles;
        v[m] = a;
    }
}
/// дополнить векторы управлений до одинаковой максимальной длины
void layout_controls_continue(std::vector<Actuator> &v, const size_t max_sz, const muscle_t n_muscles)
{
    v.resize(max_sz);
    muscle_t m = 0;
    frames_t last_start = 0;
    for (auto &a : v)
    {
        if (a.muscle == MInvalid)
        {
            a.muscle = m % n_muscles;
            a.start = last_start;
            a.lasts = 0;
        }
        last_start = std::max(last_start, a.start + a.lasts);
        ++m;
    }
}

//------------------------------------------------------------------------------
void RoboPos::LearnMoves::gradientControls(IN const Point   &aim, IN  double delta/*distance(lo.hit, hi.hit)*/,
                                           IN const Control &inits_controls,
                                           IN const Control &lower_controls,
                                           IN const Control &upper_controls,
                                           OUT      Control &controls)
{
    std::vector<Actuator> inits, lower, upper;
    layout_controls(inits, inits_controls, robo_nmuscles);
    layout_controls(lower, lower_controls, robo_nmuscles);
    layout_controls(upper, upper_controls, robo_nmuscles);

    auto max_sz = std::max({ inits.size(), lower.size(), lower.size() });
    layout_controls_continue(inits, max_sz, robo_nmuscles);
    layout_controls_continue(lower, max_sz, robo_nmuscles);
    layout_controls_continue(upper, max_sz, robo_nmuscles);

    tcout << " inits: "; for (auto &i : inits) tcout << i << " "; tcout << std::endl;
    tcout << " lower: "; for (auto &i : lower) tcout << i << " "; tcout << std::endl;
    tcout << " upper: "; for (auto &i : upper) tcout << i << " "; tcout << std::endl;

    const int int_normalizer = int(delta / _target.precision()); /* velosity */

    for (muscle_t mo = 0; mo < max_sz; mo += RoboI::musclesPerJoint)
    {
        muscle_t mc = mo + 1;
        //auto mo = RoboI::muscleByJoint(joint, true);
        //auto mc = RoboI::muscleByJoint(joint, false);

        auto start_mo_i = (MInvalid != inits[mo].muscle) ? (inits[mo].start) : 0;
        auto start_mc_i = (MInvalid != inits[mc].muscle) ? (inits[mc].start) : 0;

        auto last_mo_i = (MInvalid != inits[mo].muscle) ? (inits[mo].lasts) : 0;
        auto last_mo_l = (MInvalid != lower[mo].muscle) ? (lower[mo].lasts - last_mo_i) : 0;
        auto last_mo_g = (MInvalid != upper[mo].muscle) ? (upper[mo].lasts - last_mo_i) : 0;
    
        auto last_mc_i = (MInvalid != inits[mc].muscle) ? (inits[mc].lasts) : 0;
        auto last_mc_l = (MInvalid != lower[mc].muscle) ? (lower[mc].lasts - last_mc_i) : 0;
        auto last_mc_g = (MInvalid != upper[mc].muscle) ? (upper[mc].lasts - last_mc_i) : 0;

        // last_mo_l - дельта длительности работы открывающего мускула относительно данного управления - координаты hit < aim
        // last_mo_g - дельта длительности работы открывающего мускула относительно данного управления - координаты hit > aim
        // last_mc_l - дельта длительности работы ЗАКРЫВАЮЩЕГО мускула относительно данного управления - координаты hit < aim
        // last_mc_g - дельта длительности работы ЗАКРЫВАЮЩЕГО мускула относительно данного управления - координаты hit > aim
        
        // -----------------------------------------------
        int direction_o = 0;
        int direction_c = 0;
        if (last_mo_l > 0 || last_mo_g > 0)
        {
            distance_t d = 0.;
            if (last_mo_l + last_mo_g)
            {
                d = delta / (last_mo_l + last_mo_g);
                //// direction_o = ((d_d * int_normalizer) / (last_mo_l + last_mo_g));
                direction_o = int(d * int_normalizer);
                //direction_o = int_normalizer / (last_mo_l + last_mo_g);
            }
            if (direction_o == 0)
            {
                // BACKWARD MOVEMENT
                if (d < 0.)
                { ++direction_c; /* = (direction_o) ? (direction_o + 1) : 50; */ }
                else if (d > 0.)
                { --direction_o; /* = (direction_c < 0) ? -1 : 1; */ }
            }
        }
        
        if (last_mc_l > 0 || last_mc_g > 0)
        {
            double  d = 0.;
            if (last_mc_l + last_mc_g)
            {
                d = delta / (last_mc_l + last_mc_g);
                // direction_c = ((d_d * int_normalizer) / (last_mc_l + last_mc_g));
                direction_c = int(d * int_normalizer);
                //direction_c = int_normalizer / (last_mc_l + last_mc_g);
            }
        
            if (direction_c == 0)
            {
                // BACKWARD MOVEMENT
                if (d < 0.)
                { ++direction_o; /* = (direction_o) ? (direction_o + 1) : 50; */ }
                else if (d > 0.)
                { --direction_c; /* = (direction_c < 0) ? -1 : 1; */ }
            }
        }
        
        frames_t last_o = last_mo_i;
        frames_t last_c = last_mc_i;
        frames_t start_o = start_mo_i;
        frames_t start_c = start_mc_i;
        
        auto f_dir = [](int direction, frames_t init_val, frames_t &forward, frames_t &backward) {
            if (direction < 0 || direction <= init_val)
                forward -= direction;
            else // last_mo_i < direction
            {
                forward = 0;
                backward += (direction - init_val);
            }
        };
        f_dir(direction_o, last_mo_i, last_o, last_c);
        f_dir(direction_c, last_mc_i, last_c, last_o);
        f_dir(direction_o, start_mo_i, start_o, start_c);
        f_dir(direction_c, start_mc_i, start_c, start_o);

        tcout << " mo=" << mo << " direction_o=" << direction_o << " last_o=" << last_o << " start_o=" << start_o << std::endl;
        tcout << " mc=" << mc << " direction_c=" << direction_c << " last_c=" << last_c << " start_c=" << start_c << std::endl;
        
        //if (last_o != last_c)
        {
            if (last_o) controls.append({ mo, start_o, last_o });
            if (last_c) controls.append({ mc, start_c, last_c });
        }
    }
    controls.order(robo_nmuscles);
}

//------------------------------------------------------------------------------
void RoboPos::LearnMoves::gradientControlsNew(IN const Point &aim, IN  double delta,
                                              IN const Control &inits_controls,
                                              IN const Control &lower_controls,
                                              IN const Control &upper_controls,
                                              OUT      Control &controls)
{
    std::vector<Actuator> inits, lower, upper;
    layout_controls(inits, inits_controls, robo_nmuscles);
    layout_controls(lower, lower_controls, robo_nmuscles);
    layout_controls(upper, upper_controls, robo_nmuscles);

    auto max_sz = std::max({ inits.size(), lower.size(), lower.size() });
    layout_controls_continue(inits, max_sz, robo_nmuscles);
    layout_controls_continue(lower, max_sz, robo_nmuscles);
    layout_controls_continue(upper, max_sz, robo_nmuscles);

    auto lows = Strategy::get(lower_controls);
    auto inis = Strategy::get(inits_controls);
    auto upps = Strategy::get(upper_controls);

    if (lows != upps) CWARN("incorrect strategies (lower=" << lows.number() << " & upper=" << upps.number() << " )");
    if (inis != upps) CWARN("incorrect strategies (inits=" << inis.number() << " & upper=" << upps.number() << " )");
    if (inis != upps) CWARN("incorrect strategies (inits=" << inis.number() << " & lower=" << lows.number() << " )");

    //tcout << "delta=" << delta << std::endl;
    const double d = (delta / _target.precision()); /* velosity */
    const int delta_normalized = int(d * 5);

    frames_t min_start = SIZE_MAX;
    for (muscle_t m = 0; m < max_sz/*robo_nmuscles*/; ++m)
    {
        frames_t start;
        int64_t lasts = 0;

        if (upper[m].muscle != MInvalid && inits[m].muscle != MInvalid && lower[m].muscle != MInvalid)
        //if (upper[m].muscle == inits[m].muscle && lower[m].muscle == inits[m].muscle)
        {
            start = (lower[m].start + inits[m].start + upper[m].start) / 3;
            int64_t denom = ((lower[m].lasts - inits[m].lasts) + (upper[m].lasts - inits[m].lasts));
            if (denom != 0)
                lasts = delta_normalized / denom;
            lasts = inits[m].lasts + lasts;
            if (lasts <= 0)
            {
                muscle_t mo = _robo.muscleOpposite(m);
                if (mo > m)
                {
                    inits[mo].lasts += 4;
                }
                else
                {
                    auto it = br::find(controls, mo);
                    if (it != controls.end())
                        it->lasts += 4;
                }
                continue;
            }
        }
        //else if (upper[m].muscle == lower[m].muscle)
        else if (upper[m].muscle != MInvalid && lower[m].muscle != MInvalid)
        {
            start = (lower[m].start + upper[m].start) / 3;
            int64_t denom = (upper[m].lasts - lower[m].lasts) / 3;
            if (denom != 0)
                lasts = delta_normalized / denom;
            lasts = lower[m].lasts + lasts;
            if (lasts <= 0)
            {
                muscle_t mo = _robo.muscleOpposite(m);
                if (mo > m)
                {
                    inits[mo].lasts += 4;
                }
                else
                {
                    auto it = br::find(controls, mo);
                    if (it != controls.end())
                        it->lasts += 4;
                }
                continue;
            }
        }
        else if (inits[m].muscle != MInvalid)
        {
            start = inits[m].start;
            lasts = inits[m].lasts;
        }
        //else if (upper[m].muscle == MInvalid && inits[m].muscle == MInvalid && lower[m].muscle == MInvalid)
        //{
        //    start = 0;
        //    lasts = Utils::random<int>(1,20);
        //}
        else continue;
        controls.append({ m, start, frames_t(lasts) });
        if (min_start > start)
            min_start = start;
    }
    for (auto &c : controls)
        c.start -= min_start;
}

//------------------------------------------------------------------------------
Robo::distance_t RoboPos::LearnMoves::testStage3(IN const Point &aim)
{
    annealing = bg::distance(_target.min(), _target.max()) / 10.;
    annealing = 0.11;// new_distance / 5;
    CINFO("NEW aim=" << aim << " annealing=" << annealing);

    distance_t distance, new_distance;
    new_distance = distance = bg::distance(_base_pos, aim);
    // -----------------------------------------------
    int i_admix = 0, all_admixes_tried = 0;
    do
    {
        Record  rec;
        Control lower_controls, upper_controls;
        distance_t delta; // distance between points in function space
        if (!weightedMeanULAdjs(aim, &rec, lower_controls, upper_controls, delta))
        {
            CINFO("weightedMeanULAdjs FAIL");
            break;
        }
        // -----------------------------------------------
        Control controls;
        gradientControls(aim, delta,
                         rec.controls/*inits*/,
                         lower_controls,
                         upper_controls,
                         controls);
        // -----------------------------------------------
        new_distance = actionRobo(aim, controls);
        //++_gradient_wmeans_complexity;
        updateReachedStat(Admix::GradWMeans);
        // -----------------------------------------------
        if (less(distance, new_distance))
        {
            auto ann1 = sqrt(distance); // annealing * 0.9;// 
            if (ann1 < annealing)
            {
                annealing = ann1;
                CINFO(" annealing=" << annealing << " d2=" << distance * 2 << " d=" << (distance));
            }
            distance = new_distance;
            all_admixes_tried = 0;
            if (check_precision(distance, aim))
                break;
        }
        else if (all_admixes_tried == 3)
        {
            CINFO(aim << " testStage3 FAIL");
            break;
        }
        // -----------------------------------------------
        if (!less(distance, new_distance)) // если не приблизились основным алгоритмом
        {
            const int n_admixes = sizeof(admixes) / sizeof(*admixes);
            new_distance = (this->*(admixes[i_admix]))(aim); // используем алгоритмы-примеси
            i_admix = ++i_admix % n_admixes;
            all_admixes_tried++;

            if (less(distance, new_distance))
            {
                distance = new_distance;
                if (check_precision(distance, aim))
                    break;
            }
            if (new_distance > (distance + side3))
            {
                CINFO("FAIL Admix=" << i_admix << " d=" << new_distance << " > side=" << distance);
                //break;
            }
        }
        CDEBUG(_T("prec: ") << new_distance);
    } while (true);
    // -----------------------------------------------
    CINFO(_T(" precision: ") << distance << std::endl);
    return distance;
}

//------------------------------------------------------------------------------
const Record* RoboPos::LearnMoves::gradientClothestRecord(IN const adjacency_ptrs_t &range,
                                                          IN const Point            &aim,
                                                          IN const HitPosRelToAim   *pHitPosPred,
                                                          IN OUT   visited_t        *pVisited)
{
    const Record *pRecMin = nullptr;
    distance_t d_min = DBL_MAX;
    //CINFO("aim " << aim);
    for (const auto &pRec : range)
    {
        bool visited = false;
        if (pVisited)
        {
            const RecordHasher rh;
            size_t h = rh(*pRec);
            visited = (pVisited->find(h) == pVisited->end());
        }
        bool endPointCondition = (pHitPosPred) ? (*pHitPosPred)(*pRec) : true;

        distance_t d_curr = bg::distance(pRec->hit, aim);
        //CDEBUG("d_curr=" << d_curr);
        //CINFO("hit " << pRec->hit);
        if (!visited && endPointCondition && d_curr < d_min)
        {
            pRecMin = pRec;
            d_min = d_curr;
        }
    }
    if (pRecMin)
        CINFO("d=" << d_min << " hit=" << pRecMin->hit << " c=" << pRecMin->controls)
    else
        CINFO("d=- hit=- c=-")
    return pRecMin;
}

//------------------------------------------------------------------------------
bool RoboPos::LearnMoves::gradientSomeClothestRecords(IN  const Point &aim,
                                                      OUT Record *pRecClose,
                                                      OUT Record *pRecLower,
                                                      OUT Record *pRecUpper,
                                                      IN OUT visited_t *pVisited)
{
    const Point min(aim.x - annealing, aim.y - annealing),
                max(aim.x + annealing, aim.y + annealing);
    // ------------------------------------------------
    adjacency_ptrs_t range;
    _store.adjacencyRectPoints<adjacency_ptrs_t, ByP>(range, min, max);
    // ------------------------------------------------
    const Record *pRec = gradientClothestRecord(range, aim, NULL, pVisited);
    if (!pRec) { return false; }
    // ------------------------------------------------
    //HitPosRelToAim cmp_l = [aim](const Record &p) { return (p.hit.x < aim.x) && (p.hit.y < aim.y); };
    //HitPosRelToAim cmp_g = [aim](const Record &p) { return (p.hit.x > aim.x) && (p.hit.y > aim.y); };
    const Point ort = rotate(aim, pRec->hit, -90.);
    HitPosRelToAim cmp_l = [aim, ort](const Record &p) { return (onLineSide(aim, ort, p.hit) == LineSide::Left); };
    HitPosRelToAim cmp_g = [aim, ort](const Record &p) { return (onLineSide(aim, ort, p.hit) == LineSide::Right); };
    // ------------------------------------------------
    if (pVisited)
    {
        RecordHasher rh;
        size_t h = rh(*pRec);
        pVisited->insert(h);
    }
    else range.remove(pRec);
    // ===========
    *pRecClose = *pRec; 
    // ===========
    //test::Test::plotStoreAdj(range, aim, pRec->hit);
    if (pRecLower && pRecUpper)
    {
        pRec = gradientClothestRecord(range, aim, &cmp_l, pVisited);
        if (!pRec) { return false; }
        // ===========
        *pRecLower = *pRec;
        // ===========
        // ------------------------------------------------
        pRec = gradientClothestRecord(range, aim, &cmp_g, pVisited);
        if (!pRec) { return false; }
        // ===========
        *pRecUpper = *pRec;
        // ===========
    }
    return true;
}
//------------------------------------------------------------------------------
distance_t RoboPos::LearnMoves::gradientMethod(IN const Point &aim)
{
    LearnMoves::visited_t visited;
    distance_t distance, new_distance;
    new_distance = distance = bg::distance(_base_pos, aim);
    // -----------------------------------------------
    do
    {
        Record rec_close, rec_lower, rec_upper;
        if (!gradientSomeClothestRecords(aim,
                                         &rec_close,
                                         &rec_lower,
                                         &rec_upper,
                                         &visited))
        {
            CINFO("gradientSomeClothestRecords FAIL");
            break; /* FAIL */
        }
        distance_t delta = bg::distance(rec_upper.hit, rec_lower.hit);
        // -----------------------------------------------
        Control controls;
        gradientControls(aim, delta,
                         rec_close.controls/*inits*/,
                         rec_lower.controls,
                         rec_upper.controls,
                         controls);
        // -----------------------------------------------
        //auto hit = predict(controls);
        //auto d = bg::distance(aim, hit);
        //if (less(distance, d))
        {
            new_distance = actionRobo(aim, controls);
            //++_gradient_points_complexity;
            updateReachedStat(Admix::GradPoints);
            //CINFO(" predict=" << d << " hit=" << new_distance);
        }
        // -----------------------------------------------
        if (less(distance, new_distance))
        {
            distance = new_distance;
            if (check_precision(distance, aim))
                break;
        }
        else
        {
            CINFO("gradientMethod FAIL");
            break; /* FAIL */
        }
        CDEBUG(_T("prec: ") << new_distance);
    } while (true);
    // -----------------------------------------------
    CINFO(_T("gradientMethod precision: ") << distance << std::endl);
    return distance;
}
//------------------------------------------------------------------------------

