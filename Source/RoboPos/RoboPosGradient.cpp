#include "StdAfx.h"

//#include <Eigen/Eigen>

#include "Robo.h"
#include "RoboMovesTarget.h"
#include "RoboMovesStore.h"
#include "RoboLearnMoves.h"

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
void create_actuators_vector(OUT std::vector<Actuator> &v, IN const Control &c, IN muscle_t n)
{
    v.resize(n);
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
}

const int _max_n_controls = 20;
//------------------------------------------------------------------------------
void layout_controls(OUT std::vector<Actuator> &v, IN const Control &c, IN muscle_t n)
{
    if (c.size() > _max_n_controls)
        CERROR("layout_controls: controls " << c.size() << " is too long " << _max_n_controls);

    v.resize(_max_n_controls);
    for (auto &a : c)
        for (int j = a.muscle; v[j].muscle == MInvalid; j += n)
        {
            if (j > _max_n_controls)
                CERROR("layout_controls: 20 < j=" << j);
            v[j] = a;
        }
}


//------------------------------------------------------------------------------
void RoboPos::LearnMoves::gradientControls(IN const Point   &aim, IN  double delta,
                                           IN const Control &inits_controls,
                                           IN const Control &lower_controls,
                                           IN const Control &upper_controls,
                                           OUT      Control &controls)
{
    const auto n_muscles = _robo.musclesCount();
    std::vector<Actuator> inits, lower, upper;

    //create_actuators_vector(inits, inits_controls, n_muscles);
    //create_actuators_vector(lower, lower_controls, n_muscles);
    //create_actuators_vector(upper, upper_controls, n_muscles);
    layout_controls(inits, inits_controls, n_muscles);
    layout_controls(lower, lower_controls, n_muscles);
    layout_controls(upper, upper_controls, n_muscles);

    //Control inits(inits_controls);
    //const Control &lower = lower_controls;
    //const Control &upper = upper_controls;

    if (getStrategy(lower_controls) != getStrategy(upper_controls))
        CWARN("incorrect strategies (lower=" << getStrategy(lower_controls) << " & upper=" << getStrategy(upper_controls) << " )");
    if (getStrategy(inits_controls) != getStrategy(upper_controls))
        CWARN("incorrect strategies (inits=" << getStrategy(inits_controls) << " & upper=" << getStrategy(upper_controls) << " )");
    if (getStrategy(inits_controls) != getStrategy(lower_controls))
        CWARN("incorrect strategies (inits=" << getStrategy(inits_controls) << " & lower=" << getStrategy(lower_controls) << " )");

    //inits_controls

    //tcout << "delta=" << delta << std::endl;
    const double d = (delta / _target.precision()); /* velosity */
    const int delta_normalized = int(d * 5);    

    frames_t min_start = SIZE_MAX;
    auto sz = std::max(lower_controls.size(), upper_controls.size());
    sz = std::max(inits_controls.size(), sz);
    for (muscle_t m = 0; m < sz/*n_muscles*/; ++m)
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

    //for (joint_t joint = 0; joint < _robo.jointsCount(); ++joint)
    //{
    //    auto mo = RoboI::muscleByJoint(joint, true);
    //    auto mc = RoboI::muscleByJoint(joint, false);
    //    
    //    //auto it_mo_i = br::find(inits_controls, mo);
    //    //auto it_mo_l = br::find(lower_controls, mo);
    //    //auto it_mo_g = br::find(upper_controls, mo);
    //    //
    //    //auto it_mc_i = br::find(inits_controls, mc);
    //    //auto it_mc_l = br::find(lower_controls, mc);
    //    //auto it_mc_g = br::find(upper_controls, mc);
    //    //
    //    //int last_mo_i = (it_mo_i != inits_controls.end()) ? (it_mo_i->lasts) : 0U;
    //    //int last_mo_l = (it_mo_l != lower_controls.end()) ? (it_mo_l->lasts - last_mo_i) : 0U;
    //    //int last_mo_g = (it_mo_g != upper_controls.end()) ? (it_mo_g->lasts - last_mo_i) : 0U;
    //    //
    //    //int last_mc_i = (it_mc_i != inits_controls.end()) ? (it_mc_i->lasts) : 0U;
    //    //int last_mc_l = (it_mc_l != lower_controls.end()) ? (it_mc_l->lasts - last_mc_i) : 0U;
    //    //int last_mc_g = (it_mc_g != upper_controls.end()) ? (it_mc_g->lasts - last_mc_i) : 0U;
    //
    //    int last_mo_i = (MInvalid != inits[mo].muscle) ? (inits[mo].lasts) : 0;
    //    int last_mo_l = (MInvalid != lower[mo].muscle) ? (lower[mo].lasts - last_mo_i) : 0;
    //    int last_mo_g = (MInvalid != upper[mo].muscle) ? (upper[mo].lasts - last_mo_i) : 0;
    //
    //    int last_mc_i = (MInvalid != inits[mo].muscle) ? (inits[mo].lasts) : 0;
    //    int last_mc_l = (MInvalid != lower[mo].muscle) ? (lower[mo].lasts - last_mc_i) : 0;
    //    int last_mc_g = (MInvalid != upper[mo].muscle) ? (upper[mo].lasts - last_mc_i) : 0;
    //    // -----------------------------------------------
    //    int direction_o = 0;
    //    int direction_c = 0;
    //    // last_mo_l - дельта длительности работы открывающего мускула относительно данного управления - координаты hit < aim
    //    // last_mo_g - дельта длительности работы открывающего мускула относительно данного управления - координаты hit > aim
    //    // last_mc_l - дельта длительности работы ЗАКРЫВАЮЩЕГО мускула относительно данного управления - координаты hit < aim
    //    // last_mc_g - дельта длительности работы ЗАКРЫВАЮЩЕГО мускула относительно данного управления - координаты hit > aim
    //    if (last_mo_l/* > 0*/ || last_mo_g/* > 0*/)
    //    {
    //        double d = 0.;
    //        if (last_mo_l + last_mo_g)
    //        {
    //            //d = delta / (last_mo_l + last_mo_g);
    //            //// direction_o = ((d_d * int_normalizer) / (last_mo_l + last_mo_g));
    //            //direction_o = int(d * int_normalizer);
    //            direction_o = int_normalizer / (last_mo_l + last_mo_g);
    //        }
    //    
    //        if (direction_o == 0)
    //        {
    //            // BACKWARD MOVEMENT
    //            if (d < 0.)
    //            { ++direction_c; /* = (direction_o) ? (direction_o + 1) : 50; */ }
    //            else if (d > 0.)
    //            { --direction_o; /* = (direction_c < 0) ? -1 : 1; */ }
    //        }
    //    }
    //    
    //    if (last_mc_l/* > 0 */|| last_mc_g /*> 0*/)
    //    {
    //        double  d = 0.;
    //        if (last_mc_l + last_mc_g)
    //        {
    //            //d = delta / (last_mc_l + last_mc_g);
    //            //// direction_c = ((d_d * int_normalizer) / (last_mc_l + last_mc_g));
    //            //direction_c = int(d * int_normalizer);
    //            direction_c = int_normalizer / (last_mc_l + last_mc_g);
    //        }
    //    
    //        if (direction_c == 0)
    //        {
    //            // BACKWARD MOVEMENT
    //            if (d < 0.)
    //            { ++direction_o; /* = (direction_o) ? (direction_o + 1) : 50; */ }
    //            else if (d > 0.)
    //            { --direction_c; /* = (direction_c < 0) ? -1 : 1; */ }
    //        }
    //    }
    //    
    //    frames_t last_o = 0;
    //    frames_t last_c = 0;
    //    
    //    if (direction_o < 0 || (direction_o >= 0 && last_mo_i > direction_o))
    //    { last_o += (last_mo_i - direction_o); }
    //    else if ((direction_o >= 0 && last_mo_i < direction_o))
    //    {
    //        last_o = 0U;
    //        last_c += (last_mo_i + (direction_o - last_mo_i));
    //    }
    //    
    //    if (direction_c < 0 || (direction_c >= 0 && last_mc_i > direction_c))
    //    { last_c += (last_mc_i - direction_c); }
    //    else if ((direction_c >= 0 && last_mc_i <= direction_c))
    //    {
    //        last_c = 0U;
    //        last_o += (last_mc_i + (direction_c - last_mc_i));
    //    }
    //    
    //    if (last_o != last_c)
    //    {
    //        frames_t start_o = (last_o > last_c) ? 0U : (last_c + 1U);
    //        frames_t start_c = (last_o < last_c) ? 0U : (last_o + 1U);
    //    
    //        if (last_o) controls.append({ mo, start_o, last_o });
    //        if (last_c) controls.append({ mc, start_c, last_c });
    //    }
    //}
}

void RoboPos::LearnMoves::gradientControlsNew(IN const Point &aim, IN  double d_d,
                                              IN const Control &inits_controls,
                                              IN const Control &lower_controls,
                                              IN const Control &upper_controls,
                                              OUT      Control &controls)
{

}

//------------------------------------------------------------------------------
size_t RoboPos::LearnMoves::testStage3(IN const Point &aim)
{
    size_t gradient_complexity = 0U;
    // -----------------------------------------------
    _robo.reset();
    Point base_pos = _robo.position();
    Point pos; // hand_position
    // -----------------------------------------------
    distance_t distance = boost_distance(base_pos, aim),
        new_distance = distance;
    // -----------------------------------------------
    int i = 0, n = sizeof(admixes) / sizeof(*admixes);
    do
    {
        // -----------------------------------------------
        Record  rec;
        Control lower_controls, upper_controls;
        distance_t delta; // distance betwiin points in function space
        if (!weightedMeanULAdjs(aim, &rec, lower_controls, upper_controls, delta))
        {
            CINFO("weightedMeanULAdjs FAIL");
            break;
        }
        const Control &inits_controls = rec.controls;
        // -----------------------------------------------
        distance_t d = boost_distance(rec.hit, aim);
        if (_target.precision() > d)
        {
            CINFO("precision " << _target.precision() << " reached " << d);
            break;
        }
        // -----------------------------------------------
        if (new_distance > d)
        { new_distance = d; }
        else
        {
            gradient_complexity += (this->*(admixes[i]))(aim, pos);
            i = ++i % n;

            d = bg::distance(pos, aim);
            if (_target.precision() > d)
            {
                CINFO("precision " << _target.precision() << " reached " << d);
                break;
            }
            else if (new_distance > d)
                continue;
        }
        // -----------------------------------------------
        Control controls;
        gradientControls(aim, delta,
                         inits_controls,
                         lower_controls,
                         upper_controls,
                         controls);
        // -----------------------------------------------
        if (actionRobo(aim, controls, pos))
            ++gradient_complexity;
        // -----------------------------------------------
        d = boost_distance(pos, aim);
        // -----------------------------------------------
        if (new_distance > d)
        { new_distance = d; }
        if (d > side3)
        {
            /* FAIL */
            CINFO("gradientMethod_admixture FAIL d=" << d << " > side3=" << side3);
            break;
        }
        // -----------------------------------------------
        if (distance > new_distance)
        { distance = new_distance; }
        // -----------------------------------------------
    } while (_target.precision() < distance);
    // -----------------------------------------------
    CINFO(aim << _T(" precision: ") << distance << std::endl << 
          _T("gradient admix complexity: ") << gradient_complexity << std::endl);
    // -----------------------------------------------
    return gradient_complexity;
}
//------------------------------------------------------------------------------
const Record* RoboPos::LearnMoves::gradientClothestRecord(IN const adjacency_ptrs_t &range,
                                                          IN const Point            &aim,
                                                          IN const HitPosRelToAim   *pHitPosPred,
                                                          IN OUT   visited_t        *pVisited)
{
    const Record *pRecMin = NULL;
    // ------------------------
    size_t h;
    double dm;
    RecordHasher rh;
    // ------------------------
    for ( const auto &pRec : range )
    {
        if (pVisited)
            h = rh(*pRec);
        // ------------------------
        double dr = boost_distance(pRec->hit, aim);
        if ((!pVisited || pVisited->find(h) == pVisited->end())
            && (!pHitPosPred || (*pHitPosPred) (*pRec, aim))
            && (!pRecMin || dr < dm))
        {
            pRecMin = pRec;
            dm = dr;
        }
    }
    // ------------------------
    return pRecMin;
}
//------------------------------------------------------------------------------
bool RoboPos::LearnMoves::gradientSomeClothestRecords(IN  const Point &aim,
                                                      OUT Record *pRecClose,
                                                      OUT Record *pRecLower,
                                                      OUT Record *pRecUpper,
                                                      IN OUT visited_t *pVisited)
{
    if (!pRecClose) return false;
    // ------------------------------------------------
    Point min(aim.x - side3, aim.y - side3),
          max(aim.x + side3, aim.y + side3);
    // ------------------------------------------------
    adjacency_ptrs_t range;
    _store.adjacencyRectPoints<adjacency_ptrs_t, ByP>(range, min, max);
    // ------------------------------------------------
    HitPosRelToAim cmp_l = [](const Record &p, const Point &aim) { return  (p.hit.x < aim.x) && (p.hit.y < aim.y); };
    HitPosRelToAim cmp_g = [](const Record &p, const Point &aim) { return  (p.hit.x > aim.x) && (p.hit.y > aim.y); };
    // ------------------------------------------------
    const Record *pRec = gradientClothestRecord(range, aim, NULL, pVisited);
    if (!pRec) { return false; }

    RecordHasher rh;
    size_t h = rh(*pRec);
    pVisited->insert(h);
    // ===========
    *pRecClose = *pRec;
    // ===========
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
size_t RoboPos::LearnMoves::gradientMethod(IN const Point &aim, OUT Point &pos)
{
    size_t gradient_complexity = 0;
    // -----------------------------------------------
    std::set<size_t> visited;
    // -----------------------------------------------
    double distance = boost_distance(_base_pos, aim);
    double new_distance = 0.;
    // -----------------------------------------------
    _robo.reset();
    do
    {
        Record  rec_close, rec_lower, rec_upper;
        if (!gradientSomeClothestRecords(aim, &rec_close,
                                         &rec_lower,
                                         &rec_upper,
                                         &visited))
        { 
            /* FAIL */
            CINFO("gradientSomeClothestRecords FAIL");
            break;
        }
        Point hit = rec_close.hit;

        new_distance = boost_distance(hit, aim);
        if (distance > new_distance)
            distance = new_distance;

        if (_target.precision() > new_distance)
        {
            CINFO("precision " << _target.precision() << " reached " << new_distance);
            break;
        }

        Control inits_controls{ rec_close.controls };
        distance_t delta = boost_distance(rec_upper.hit, rec_lower.hit);
        // -----------------------------------------------
        Control controls;
        gradientControls(aim, delta,
                         inits_controls,
                         rec_lower.controls,
                         rec_upper.controls,
                         controls);
        // -----------------------------------------------
        if (actionRobo(aim, controls, hit))
        { ++gradient_complexity; }
        // -----------------------------------------------
        double d = boost_distance(hit, aim);
        // -----------------------------------------------
        if (_target.precision() > d)
        {
            CINFO("precision " << _target.precision() << " reached " << d);
            break;
        }
        // -----------------------------------------------
        else if (new_distance > d)
        { new_distance = d; }
        // -----------------------------------------------
        else
        {
            //visited.clear();
            //rundownFull(aim, hit);
            //
            //d = boost_distance(hit, aim);
            //if (_target.precision() > d)
            //{
            //    CINFO("precision " << _target.precision() << " reached " << d);
            //    break;
            //}
            //else if (new_distance > d)
            //{ continue; }
            //else
            {
                /* FAIL */
                CINFO("gradientMethod FAIL");
                break;
            } // end else
        } // end else
        // -----------------------------------------------
        if (distance > new_distance)
            distance = new_distance;
        // -----------------------------------------------
        CDEBUG(_T("prec: ") << distance);
    } while (_target.precision() < distance);
    // -----------------------------------------------
    CINFO(aim << _T(" precision: ") << distance << std::endl << 
          _T("grad_complex ") << gradient_complexity << std::endl);
    // -----------------------------------------------
    //auto p = _store.getClosestPoint(aim, side3);
    //if (!p.first)
    //    throw std::runtime_error{ "gradientMethod_admixture: Empty adjacency" };
    //pos = p.second.hit;

    return gradient_complexity;
}
//------------------------------------------------------------------------------

