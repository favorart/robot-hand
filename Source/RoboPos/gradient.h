#pragma once

//#include <string>
//#include <iostream>
//#include <iomanip>
//#include <sstream>
//#include <fstream>
//
//#include <vector> // used to import the stl vector class
//#include <fstream> // used to create an object to read and write fils
//#include <string> // used to import the string object
//#include <iostream> // used to take input output
//using namespace std; // to void writing the scope resolution operator to access the std namespace

#include "Eigen/Eigen"

//#include "Robo.h"
//#include "RoboPhysics.h"

namespace gradient {
/*@
*  The basic idea is to reduce the cost function by descending down the gradient of the cost function
*  with respect to one of the variables in the cost funciton. 
* 
*  cost = 1/N * sigma ((y - mx + b) ** 2)
* 
*  the partial differentiation of cost with respect to x and b should give us a direction
*  from the reseult to descent our gradient accordingly to plot the best curve to fit through
*  the data points
*/
class Descent
{
public:
    using residual = double; ///< невязка
    using value = double;
    using vector = std::vector<double>;
    using matrix = Eigen::MatrixXd;

    /// reads the csv file and stores it in the vector containing a vector of array with two elements
    /// first element being the value of y axis for a specific point and second being the x axis for a specific point
    //void generate(/*string*/);
    /// just a member function to display the data
    //void display();

    /// iterates the dataset a certain number of times and does gradient descent
    residual batch_train(value*, value*, value); ///< takes in a m and b value and the learning rate
private:
    /// gradient descent needs to happen multiple times, as we are getting better and better at
    /// plotting the line to best fit our dataset, hence we do gradient descent 100000 amount of times
    const int iteration_of_same_data_set = 10000;
    /// goes through one iteration and applies gradient descent
    residual descent(value*, value*, value); ///< takes in a m and b value and the learning rate 
    matrix points;
};
} // end gradient
