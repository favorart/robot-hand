﻿#include "RoboMovesStore.h"
#include "RoboPosApprox.h"

using namespace RoboMoves;
using namespace RoboPos;
using namespace Utils;
using namespace Eigen;


int main()
{
    std::cout << std::setprecision(3) << std::fixed << std::showpos << std::left;
    //Store store("Hand-v3-robo-moves-2018.04.26-10.10.bin");
    
    //MatrixXd X(7,2);
    //X <<
    //    0., 0.,
    //    0., 1.,
    //    1., 0.,
    //    1., 1.,
    //    0., 2.,
    //    2., 0.,
    //    2., 2.;

    MatrixXd X(81, 6);
    X <<1,	0,	285,	2,	0,  81,
        0,	0,	 70,	3,	0, 329,
        1,	0,	247,	3,	0, 436,
        0,	0,	 73,	2,	0, 163,
        0,	0,	 94,	3,	0, 112,
        1,	0,	247,	3,	0, 378,
        0,	0,	473,	3,	0, 549,
        0,	0,	349,	3,	0, 472,
        1,	0,	247,	3,	0, 325,
        1,	0,	 73,	2,	0, 163,
        1,	0,	247,	3,	0, 163,
        1,	0,	247,	3,	0,  81,
        1,	0,	247,	2,	0, 499,
        1,	0,	247,	2,	0, 325,
        1,	0,	247,	2,	0, 277,
        0,	0,	331,	3,	0, 436,
        1,	0,	247,	2,	0, 196,
        1,	0,	214,	3,	0, 163,
        1,	0,	247,	2,	0,  73,
        0,	0,	290,	3,	0,  73,
        1,	0,	214,	3,	0,  94,
        0,	0,	225,	2,	0, 135,
        1,	0,	214,	3,	0,  70,
        1,	0,	214,	2,	0, 499,
        1,	0,	214,	2,	0, 135,
        1,	0,	 70,	3,	0, 390,
        1,	0,	214,	2,	0, 112,
        1,	0,	214,	2,	0,  81,
        1,	0,	318,	3,	0, 234,
        0,	0,	455,	3,	0, 490,
        1,	0,	214,	2,	0,  73,
        0,	0,	421,	3,	0, 623,
        1,	0,	186,	3,	0, 436,
        1,	0,	461,	2,	0, 325,
        0,	0,	308,	3,	0, 278,
        1,	0,	186,	3,	0, 325,
        1,	0,	247,	3,	0, 112,
        1,	0,	186,	3,	0, 196,
        0,	0,	207,	3,	0, 405,
        1,	0,	186,	3,	0, 163,
        1,	0,	186,	3,	0, 135,
        1,	0,	163,	2,	0, 234,
        1,	0,	186,	3,	0, 112,
        0,	0,	 81,	2,	0, 163,
        1,	0,	186,	3,	0,  94,
        0,	0,	194,	2,	0, 135,
        0,	0,	225,	2,	0, 234,
        1,	0,	186,	3,	0,  70,
        1,	0,	186,	2,	0, 499,
        0,	0,	308,	2,	0, 135,
        1,	0,	112,	3,	0, 448,
        1,	0,	135,	2,	0,  81,
        1,	0,	186,	2,	0, 436,
        1,	0,	112,	3,	0, 318,
        1,	0,	186,	2,	0, 234,
        1,	0,	163,	3,	0, 601,
        1,	0,	163,	3,	0, 520,
        0,	0,	 94,	2,	0, 112,
        1,	0,	163,	3,	0, 325,
        1,	0,	407,	3,	0,  70,
        1,	0,	163,	3,	0, 234,
        0,	0,	349,	3,	0,  94,
        0,	0,	194,	3,	0,  73,
        1,	0,	163,	3,	0, 135,
        1,	0,	163,	3,	0, 112,
        1,	0,	163,	3,	0,  94,
        0,	0,	 94,	2,	0, 378,
        1,	0,	163,	3,	0,  73,
        0,	0,	 70,	2,	0,  70,
        1,	0,	163,	2,	0, 499,
        0,	0,	331,	3,	0,  70,
        1,	0,	163,	2,	0, 378,
        1,	0,	135,	3,	0, 315,
        1,	0,	163,	2,	0, 163,
        1,	0,	163,	2,	0, 135,
        1,	0,	163,	2,	0, 112,
        1,	0,	112,	3,	0,  73,
        1,	0,	135,	3,	0, 572,
        1,	0,	135,	3,	0, 511,
        1,	0,	135,	3,	0, 277,
        1,	0,	135,	3,	0, 234;


    //MatrixXd Y(7, 2);
    //Y <<
    //    0., +0.,
    //    1., +5.,
    //    1., -5.,
    //    0., +0.,
    //    1., +9.,
    //    1., -9.,
    //    0., +0.;

    MatrixXd Y(81, 2);
    Y <<
         5.583,	 1.773,	
         4.040, -5.128,	
         7.988,	 4.969,	
        -6.846,	 5.368,	
        -9.683, -3.534,	
         7.378,	 4.418,	
         4.156, -1.342,	
         1.076, -1.245,	
         6.855,	 5.416,	
         9.582,	 3.606,	
         5.814,	 1.312,	
         5.622,	 1.647,	
         5.573,	 1.766,	
         5.573,	 1.766,	
         5.573,	 1.766,	
        -1.597, -1.100,	
         5.573,	 1.766,	
         5.828,	 1.328,	
         5.573,	 1.766,	
        -7.385,	 7.355,	
         5.657,	 1.620,	
        -5.927,	 4.227,	
         5.625,	 1.690,	
         5.590,	 1.778,	
         5.590,	 1.778,	
         7.179, -4.418,	
         5.590,	 1.778,	
         5.590,	 1.778,	
         6.145,	 9.418,	
         2.563, -1.299,	
         5.590,	 1.778,	
         4.134, -1.341,	
         8.190,	 7.151,	
         5.587,	 1.776,	
        -8.134, -1.366,	
         6.801,	 7.374,	
         5.674,	 1.542,	
         5.664,	 7.665,	
         3.063, -9.560,	
         5.492,	 9.693,	
         5.377,	 1.133,	
         4.122,	 9.832,	
         5.310,	 1.243,	
        -9.011,	 5.857,	
         5.266,	 1.322,	
        -4.849,	 2.839,	
        -5.927,	 4.227,	
         5.224,	 1.406,	
         5.175,	 1.513,	
        -7.100,	 9.298,	
         9.219, -3.042,	
         2.958,	 5.997,	
         5.175,	 1.513,	
         5.504, -2.949,	
         5.175,	 1.513,	
         9.831, -3.762,	
         9.839, -3.703,	
        -1.248,	 6.772,	
         6.549, -1.078,	
         5.623,	 1.688,	
         5.237, -4.311,	
        -7.592,	 8.596,	
        -4.850,	 2.231,	
         4.428,	 4.315,	
         4.327,	 5.910,	
         4.261,	 7.053,	
        -1.248,	 6.772,	
         4.201,	 8.187,	
        -6.403,	 5.276,	
         4.122,	 9.832,	
        -7.353,	 9.165,	
         4.122,	 9.832,	
         5.903, -2.156,	
         4.122,	 9.832,	
         4.122,	 9.832,	
         4.122,	 9.832,	
         2.186,	 1.677,	
         1.051, -1.394,	
         1.050, -1.395,	
         5.127, -1.813,	
         4.392, -1.318;	
    
    //MatrixXd pred(7, 2);
    //pred <<
    //    0.0, 0.5,
    //    0.5, 1.0,
    //    0.5, 0.5,
    //    0.0, 1.5,
    //    1.5, 0.0,
    //    1.5, 1.5,
    //    1.5, 1.7;


    MatrixXd pred(7, 6);
    pred << 
           1,    0,    280,    2,    0,  81, // [1,    0,    285,    2,    0,  81],    // [ 5.583,     1.773]
           0,    0,     77,    3,    0, 329, // [0,    0,     70,    3,    0, 329],    // [ 4.040,    -5.128]
           1,    0,    240,    3,    0, 436, // [1,    0,    247,    3,    0, 436],    // [ 7.988,     4.969]
           0,    0,     73,    2,    0, 163, // [0,    0,     73,    2,    0, 163],    // [-6.846,     5.368]
           0,    0,     90,    3,    0, 112, // [0,    0,     94,    3,    0, 112],    // [-9.683,    -3.534]
           1,    0,    249,    3,    0, 378, // [1,    0,    247,    3,    0, 378],    // [ 7.378,     4.418]
           0,    0,    473,    3,    0, 549; // [0,    0,    473,    3,    0, 549],    // [ 4.156,    -1.342]
    
    try
    {
        Approx approx(X, Y);

        MatrixXd x = Map<MatrixXd, 0, OuterStride<>>(X.data(), 7, 6, OuterStride<>(X.rows()));
        
        VectorXi indices = VectorXi::LinSpaced(x.rows(), 0, x.rows());
        std::random_shuffle(indices.data(), indices.data() + x.rows());
        x = indices.asPermutation() * x;

        auto rez = approx.predict(x);
        //std::cout << rez << std::endl;
        //auto rez = approx.predict(x);
        std::cout << rez << std::endl;
        std::cout << std::endl;

        //auto rez = approx.predict(X);
        //std::cout << rez << std::endl;
        //std::cout << std::endl;
        //
        //rez = approx.predict(X);
        //std::cout << rez << std::endl;
        //std::cout << std::endl;

        for (auto i = 0; i < 7; ++i)
        {
            VectorXd v{ X.row(i) };
            //std::cout << v << std::endl;
            Point p = approx.predict(v);
            std::cout << p.x << ' ' << p.y << " - " << Y.row(i) << std::endl;
        }
    }
    catch (const std::exception &e)
    {
        CERROR(e.what());
    }
    
    std::getchar();
    return 0;
}

