#include <random>
#include "gradient.h"

using namespace std;
using namespace gradient;

//void gradient::Descent::generate()
//{
//    string fileName;
//    ifstream file; // ifstream object declared
//    file.open(fileName); // file pointer pointing to the file
//    string out; // strings in each line of a file will be assigned to out
//    // loop till file.oef() doesn't return a false
//    while (!file.eof())
//    {
//        vector temp;
//        int i = 0;
//        string yString = "";
//        string xString = "";
//        getline(file, out); // a single line from the file is read and put inside the string object called out
//        while (out[i] != ',')
//        {
//            yString += out[i];
//            ++i;
//        }
//        ++i; // this is done to avoid the ',' as the previous while loop breaks
//            // at ','
//        while (out[i])
//        { // while out[i] is not NULL
//            xString += out[i];
//            ++i;
//        }
//        temp.push_back(stof(yString));
//        temp.push_back(stof(xString));
//        points.push_back(temp);
//    }
//    file.close(); // close the file that was opened
//}

//void gradient::Descent::display()
//{
//    for (int i = 0; i < points.size(); ++i)
//    {
//        for (int j = 0; j < points[i].size(); ++j)
//        {
//            if (j == 0) cout << "Y-axis: ";
//            else cout << "X-axis: ";
//            cout << points[i][j] << endl;
//        }
//        cout << endl;
//    }
//}

auto gradient::Descent::descent(value* m, value* b, value learningRate) -> residual
{
    value gradM = 0; // gradient of M to be accumulated
    value gradB = 0; // gradient of B to be accumulated

    // iterating over the dataset and grabbing values of x and y and calculating
    // the partial derivative of the cost function with respect to the m and then
    // with respect with b
    for (int i = 0; i < points.size(); ++i)
    {
        // assigning the x and y values from the multidimentional array
        value x = points(i,0);
        value y = points(i,1);

        gradM += -(x * (y - ((*m * x) + *b)));
        gradB += -(y - ((*m * x) + *b));
    }
    // type casting an int to a float for the size which is n in the formula
    value n = static_cast<value>(points.size());

    // averaging the gradient
    gradM = gradM * (2. / n);
    gradB = gradB * (2. / n);

    // learning the new m and b value
    // changes the value of m and b depending on the gradient of cost function with respect to M and B
    *m = *m - (gradM * learningRate);
    *b = *b - (gradB * learningRate);
    return 0.;
}

auto gradient::Descent::batch_train(value* m, value* b, value learningRate) -> residual
{
    value rM = 0; // value to change starts at 0
    value rB = 0; // value to change starts at 0

    // Explicitly deciding how many times that one data set will be iterated
    // this is the key part of gradient descent, each iteration changes the value
    // of m and b, this change improves the m and b over each iterations every time.
    // as gradDesc function finds the derivative of the cost function in terms of 
    // m and then b. For each points in the dataset the gradient is found and added and then by using
    // the formula new_m = m - (gradient_of_m * learning_rate), we find the new m and b as well similarly.
    residual res = 0;
    for (int i = 0; i < iteration_of_same_data_set; ++i)
        res += descent(&rM, &rB, learningRate);

    *m = rM;
    *b = rB;
    return res;
}

