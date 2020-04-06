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

#include <rlTypes.hpp>
#include <rlConcept.hpp>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
#include <cmath>
#include <functional>

namespace rl {
namespace transfer {
inline double identity(double weighted_sum)
{
    return weighted_sum;
}

/**
 * @short This acts as a sigmoid, since it is y=ax kept in [-1,1]
 */
inline double saturation(double weighted_sum, double a)
{
    double res = weighted_sum * a;
    if (res > 1.)
        return 1.;
    if (res < -1.)
        return -1.;
    return weighted_sum;
}
  
/**
 * @short This is tanh(ax) saturation.
 */
/**
 * @short This acts as a sigmoid, since it is y=ax kept in [-1,1]
 */
inline double tanh(double weighted_sum, double a)
{
  return ::tanh(weighted_sum*a);
} 
}

namespace gsl {
namespace mlp {
/**
 * @short This defines the input layer of the neural network.
 */
template<
     typename STATE,
     typename ACTION,
     typename fctFEATURE> 
class Input
{
    unsigned phi_dim;
	gsl_vector* xx;
	std::function<void (gsl_vector*,const STATE&, const ACTION&)> phi;
	
public:
	typedef STATE  state_type;
	typedef ACTION action_type;
	
	unsigned int rank(void)         const {return 0;}
	unsigned int minParamRank(void) const {return 0;}
	unsigned int nbParams(void)     const {return 0;}
	unsigned int layerSize(void)    const {return phi_dim;}

	unsigned int size;

	void displayParameters(std::ostream& os) const
    {
	  os << "Input  #" << std::setw(3) << rank()
	     << " :        no weight"
	     << " : size = " << std::setw(4) << layerSize() << std::endl;
	}

	Input(/*const*/ fctFEATURE& f, unsigned feature_dimension) // Warning C4180: qualifier applied to function type has no meaning; ignored
      : phi_dim(feature_dimension),
	    xx(gsl_vector_alloc(phi_dim)),
	    phi(f),
	    size(0)
    {}
	Input(const Input<STATE,ACTION,fctFEATURE>& cp) 
	  : phi_dim(cp.phi_dim),
        xx(gsl_vector_alloc(cp.xx->size)),
        phi(cp.phi),
        size(cp.size)
    {
	  gsl_vector_memcpy(xx,cp.xx);
	}
    ~Input()
    {
        gsl_vector_free(xx);
    }

	Input<STATE, ACTION, fctFEATURE> operator=(const Input<STATE,ACTION,fctFEATURE>& cp)
    {
	  if(this != &cp)
      {
	    phi_dim = cp.phi_dim;
	    gsl_vector_free(xx);
	    xx = gsl_vector_alloc(cp.xx->size);
	    gsl_vector_memcpy(xx, cp.xx);
        phi = cp.phi;
	    size = cp.size;
	  }
	  return this;
	}
    
	void operator()(const gsl_vector* /*theta*/, const state_type& s, const action_type& a, gsl_vector * const yy) const
    {
	  phi(xx, s, a);
      assert(yy->size == xx->size);
      gsl_vector_memcpy(yy, xx);
	}
    friend std::istream& operator>>(std::istream &s, Input<STATE, ACTION, fctFEATURE> &output)
    {
        char c;
        return s >> c/*{*/
            >> output.phi_dim
            >> output.xx /*>> output.phi*/
            >> output.size
            >> c/*}*/;
    }
    friend std::ostream& operator<<(std::ostream &s, const Input<STATE, ACTION, fctFEATURE> &output)
    {
        return s << '{'
            << output.phi_dim << ' '
            << output.xx << ' ' /*<< output.phi << ' '*/
            << output.size
            << '}' << std::endl;
    }
}; // Input

template <
     typename STATE,
     typename ACTION,
     typename fctFEATURE> 
Input<STATE,ACTION,fctFEATURE> input(const fctFEATURE& f, unsigned feature_dimension)
{
	return Input<STATE,ACTION,fctFEATURE>(f,feature_dimension);
}

/**
 * @short This defines the some hidden layer of the neural network.
 */
template <typename PREVIOUS_LAYER, typename MLP_TRANSFER>
class Hidden
{
	unsigned int layer_size;
	mutable gsl_vector *yy;

public:
	typedef typename PREVIOUS_LAYER::state_type  state_type;
	typedef typename PREVIOUS_LAYER::action_type action_type;

	PREVIOUS_LAYER &input;
	std::function<double(double)> f;
	unsigned int size;
	
	unsigned int rank(void)         const {return 1+input.rank();}
	unsigned int minParamRank(void) const {return input.minParamRank()+input.nbParams();}
	unsigned int nbParams(void)     const {return layer_size*(1+input.layerSize());}
	unsigned int layerSize(void)    const {return layer_size;}

	void displayParameters(std::ostream& os) const
    {
	  input.displayParameters(os);
	  os << "Hidden #" << std::setw(3) << rank()
	     << " : " << "[" << std::setw(6) << minParamRank() << ", " << std::setw(6) << minParamRank()+nbParams() << "["
	     << " : size = " << std::setw(4) << layerSize() << std::endl;
	}

	Hidden(PREVIOUS_LAYER& in, unsigned nb_neurons, const MLP_TRANSFER& transfer) 
	  : layer_size(nb_neurons),
        yy(gsl_vector_alloc(in.layerSize())),
        input(in),
        f(transfer),
        size(minParamRank() + nbParams())
    {}
	Hidden(const Hidden<PREVIOUS_LAYER,MLP_TRANSFER>& cp) 
	  : layer_size(cp.layer_size),
        yy(gsl_vector_alloc(cp.yy.size)),
        input(cp.input),
        f(cp.f),
        size(cp.size)
    {
        gsl_vector_memcpy(yy, cp.yy);
    }
    virtual ~Hidden()
    {
        gsl_vector_free(yy);
    }

	Hidden<PREVIOUS_LAYER,MLP_TRANSFER>& operator=(const Hidden<PREVIOUS_LAYER,MLP_TRANSFER>& cp)
    {
	  if (this != &cp)
      {
	    layer_size = cp.layer_size;
        gsl_vector_free(yy);
        yy = gsl_vector_alloc(cp.yy.size);
        gsl_vector_memcpy(yy, cp.yy);
	    input = cp.input;
	    f = cp.f;
	    size = cp.size;
	  }
	  return *this;
	}

    void operator()(const gsl_vector *theta, const state_type &s, const action_type &a, gsl_vector * const y) const
    {
        assert(yy->size == input.layerSize());
        input(theta, s, a, yy);

        assert(y->size == layerSize());
        unsigned k = minParamRank();
        for (size_t i = 0; i < y->size; ++i)
        {
            double sum = gsl_vector_get(theta, k);
            ++k;
            for (size_t j = 0; j < yy->size; ++j)
            {
                sum += gsl_vector_get(theta, k) * gsl_vector_get(yy, j);
                ++k;
            }
            gsl_vector_set(y, i, f(sum));
        }
    }
    friend std::istream& operator>>(std::istream &s, Hidden<PREVIOUS_LAYER, MLP_TRANSFER> &output)
    {
        char c;
        return s >> c/*{*/
            >> output.layer_size 
            >> output.yy /*>> output.input >> output.f*/
            >> output.size 
            >> c/*}*/;
    }
    friend std::ostream& operator<<(std::ostream &s, const Hidden<PREVIOUS_LAYER, MLP_TRANSFER> &output)
    {
        return s << '{'
            << output.layer_size << ' ' 
            << output.yy << ' ' /*<< output.input << ' ' << output.f << ' '*/
            << output.size 
            << '}' << std::endl;
    }
}; // Hidden

template <typename PREVIOUS_LAYER,typename MLP_TRANSFER>
Hidden<PREVIOUS_LAYER,MLP_TRANSFER> hidden(PREVIOUS_LAYER& in, unsigned layer_size, const MLP_TRANSFER& transfer)
{
	return Hidden<PREVIOUS_LAYER,MLP_TRANSFER>(in,layer_size,transfer);
}

/**
 * @short This defines the output layer of the neural network.
 */
template<typename PREVIOUS_LAYER,typename MLP_TRANSFER>
class Output 
{
private:
	mutable gsl_vector *y;
public:
	typedef typename PREVIOUS_LAYER::state_type  state_type;
	typedef typename PREVIOUS_LAYER::action_type action_type;

	unsigned int rank(void)         const {return 1+input.rank();}
	unsigned int minParamRank(void) const {return input.minParamRank()+input.nbParams();}
	unsigned int nbParams(void)     const {return 1*(1+input.layerSize());}
	unsigned int layerSize(void)    const {return 1;}
	
	PREVIOUS_LAYER &input;
	std::function<double(double)> f;
	unsigned int size;
	
	void displayParameters(std::ostream& os) const
    {
	  input.displayParameters(os);
	  os << "Output #" << std::setw(3) << rank()
	     << " : " << "[" << std::setw(6) << minParamRank() << ", " << std::setw(6) << minParamRank()+nbParams() << "["
	     << " : size = " << std::setw(4) << layerSize() << std::endl;
	}

	Output(PREVIOUS_LAYER& in, /*const*/ MLP_TRANSFER& transfer) 
	  : y(gsl_vector_alloc(in.layerSize())),
        input(in),
        f(transfer),
        size(minParamRank() + nbParams())
    {}
	Output(const Output<PREVIOUS_LAYER,MLP_TRANSFER>& cp)
	  : y(gsl_vector_alloc(cp.input.layerSize())),
        input(cp.input),
        f(cp.f),
        size(cp.size)
    {
        gsl_vector_memcpy(y, cp.y);
    }
    virtual ~Output()
    {
        gsl_vector_free(y);
    }

	Output<PREVIOUS_LAYER,MLP_TRANSFER>& operator=(const Output<PREVIOUS_LAYER,MLP_TRANSFER>& cp)
    {
	  if (this != &cp)
      {
          gsl_vector_free(y);
          y = gsl_vector_alloc(cp.input.layerSize());
          gsl_vector_memcpy(y, cp.y);
          input = cp.input;
          f = cp.f;
          size = cp.size;
	  }
	  return *this;
	}

    double operator()(const gsl_vector *theta, const state_type &s, const action_type &a) const
    {
        assert(y->size == input.layerSize());
        input(theta, s, a, y);

        unsigned k = minParamRank();
        double sum = gsl_vector_get(theta, k);
        ++k;
        for (size_t j = 0; j != y->size; ++j)
        {
            sum += gsl_vector_get(theta, k) * gsl_vector_get(y, j);
            ++k;
        }
        return f(sum);
    }
    friend std::istream& operator>>(std::istream &s, Output<PREVIOUS_LAYER, MLP_TRANSFER> &output)
    {
        char c;
        return s >> c/*{*/
            >> output.y /*>> output.input >> output.f*/ 
            >> output.size 
            >> c/*}*/;
    }
    friend std::ostream& operator<<(std::ostream &s, const Output<PREVIOUS_LAYER, MLP_TRANSFER> &output)
    {
        return s << '{' 
            << output.y << ' ' /*<< output.input << ' ' << output.f << ' '*/
            << output.size 
            << '}' << std::endl;
    }
}; // Output

template <typename PREVIOUS_LAYER,typename MLP_TRANSFER>
Output<PREVIOUS_LAYER,MLP_TRANSFER> output(PREVIOUS_LAYER& in, const MLP_TRANSFER& transfer)
{
	return Output<PREVIOUS_LAYER,MLP_TRANSFER>(in,transfer);
}

} // mlp
} // gsl
} // rl
