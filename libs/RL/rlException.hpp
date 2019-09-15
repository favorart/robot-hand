/*   This file is part of rl-lib
 *
 *   Copyright (C) 2010,  Supelec
 *
 *   Author : Herve Frezza-Buet and Matthieu Geist
 *
 *   Contributor :
 *
 *   This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU General Public
 *   License (GPL) as published by the Free Software Foundation; either
 *   version 3 of the License, or any later version.
 *   
 *   This library is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *   General Public License for more details.
 *   
 *   You should have received a copy of the GNU General Public
 *   License along with this library; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 *   Contact : Herve.Frezza-Buet@supelec.fr Matthieu.Geist@supelec.fr
 *
 */

#pragma once

#include <string>
#include <sstream>

namespace rl {
  namespace exception {

    class Any : public std::exception {
    private:
      std::string message;
    public:
      Any(const std::string& msg) 
	: message(std::string("RL exception : ") + msg) {}

      virtual ~Any(void) throw () {}

      virtual const char * what(void) const throw ()
      {
        return message.c_str();
      }
    };

    /**
     * @short Terminal state.
     * 
     * Raised when a state is accessed whereas the controlled system
     * has reached a terminal state.
     */
    class Terminal : public Any {
    public:
      
      Terminal(std::string comment) 
	: Any(std::string("Terminal state access : ")+comment) {}
    };

    /**
     * @short Problem with gsl vector size.
     * 
     */
    class BadVectorSize : public Any {
        static std::string error(size_t actual_size, size_t expected_size)
        {
            std::ostringstream ostr;
            ostr << "Bad vector size : gsl_vector of size" << expected_size
                << " expected while size " << actual_size
                << " is received : ";
            return ostr.str();
        }

    public:
      BadVectorSize(size_t actual_size, size_t expected_size,
                    std::string comment) 
	: Any(error(actual_size,expected_size)+comment) {}
    };

    /**
     * @short Problem with gsl matrix dimension.
     *
     */
    class BadMatrixDim : public Any {
        static std::string error(size_t actual_size1, size_t expected_size1,
                          size_t actual_size2, size_t expected_size2)
        {
            std::ostringstream ostr;
            ostr << "Bad matrix dimension : gsl_matrix of"
                << " size1=" << expected_size1 << "Xsize2=" << expected_size2
                << " expected while size1=" << actual_size1 << "Xsize2=" << actual_size2
                << " is received : ";
            return ostr.str();
        }
    public:
        BadMatrixDim(size_t actual_size1, size_t expected_size1,
                     size_t actual_size2, size_t expected_size2,
                     std::string comment)
            : Any(error(actual_size1, expected_size1, actual_size2, expected_size2) + comment)
        {}
    };

    class NotPositiveDefiniteMatrix : public Any {
    public:
      NotPositiveDefiniteMatrix(std::string comment) 
	: Any(std::string("A positive definite matrix is required : ")+comment) {}
    };

    class NullVectorPtr : public Any {
    public:
      NullVectorPtr(std::string comment) 
	: Any(std::string("Got a gsl_vector*=NULL : ")+comment) {}
    };

  }
}
