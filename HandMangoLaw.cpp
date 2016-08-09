#include "HandMotionLawsCustom.h"
#include "lagrange_interp_1d.hpp"

//------------------------------------------------------------------------------
namespace NewHand
{
  namespace
  {
    void  MangoInputFile (const tstring        &filename,
                          std::vector<double>  &angles,
                          std::vector<double>  &frames)
    {
      std::ifstream  fin (filename, std::ios_base::in);
      // --------------------------------
      if ( fin.is_open () )
      {
        size_t  mii;
        if ( fin >> mii )
        { angles.reserve (mii); }
        else
        { throw std::exception ("ifstream: input error."); }

        double angle = 0.;
        while ( fin >> angle )
        { angles.push_back (angle); }
        // --------------------------------
        size_t m = angles.size ();
        frames.resize (m);
        // --------------------------------
        for ( size_t i = 1U; i < m; ++i )
        { frames[i] = i; }
        // --------------------------------
      }
      else
      { throw std::exception ("ifstream: input error."); }
    }

    void  MangoOutputFile (const tstring       &filename,
                           MotionLaws::VectorDoublesIterator first, size_t  n)
    {
      std::ofstream  fout (filename + tstring (_T ("-out.txt")),
                           std::ofstream::out);
      //------------------------------------------------------
      auto iter = first;
      for ( size_t i = 0U; i < n; ++i )
      { fout << *iter << std::endl;
        ++iter;
      }
    }
  };

  namespace MotionLaws
  {
          MangoAcceleration::MangoAcceleration (const tstring &filename, bool out) : 
            filename (filename), out (out)
    { MangoInputFile (filename, angles, frames); }
    //------------------------------------------------------
          MangoDeceleration::MangoDeceleration (const tstring &filename, bool out) :
            filename (filename), out (out)
    { MangoInputFile (filename, angles, frames); }
    //------------------------------------------------------
    //------------------------------------------------------
    void  MangoAcceleration::generate (VectorDoublesIterator first, size_t frames_count,
                                       double left_border, double right_border) const
    {
      VectorDoublesIterator  iter = first;
      // --------------------------------
      size_t  n =  frames_count;
      size_t  m =  angles.size ();
      double  a =  left_border,
              b = right_border;
      // --------------------------------
      // std::vector<double>  new_frames (n);
      // double n_frame = 1.;
      // for ( auto &frame : new_frames )
      // { frame = n_frame * m / n;  n_frame += 1.; }
      // // --------------------------------
      // double*  new_angles = lagrange_value_1d (static_cast<int> (m), frames.data (), angles.data (),
      //                                          static_cast<int> (n), new_frames.data ());
      // --------------------------------
      double  norm_sum = 0.;
      for ( size_t i = 0U; i < n; ++i )
      {
        double d = angles[size_t (i * m / n)];

        norm_sum += d;
        *iter = d;
        ++iter;
      }
      // --------------------------------
      // delete[] new_angles;
      // --------------------------------
      iter = first;
      for ( size_t i = 0U; i < n; ++i )
      { /* apply the normalization */
        *iter = *iter * (b - n * a) / norm_sum + a;
        ++iter;
      }
      // --------------------------------
      if ( out ) MangoOutputFile (filename, first, n);
    }
    //------------------------------------------------------
    void  MangoDeceleration::generate (VectorDoublesIterator first, size_t frames_count,
                                       double left_border, double right_border,
                                       double max_velosity) const
    {
      VectorDoublesIterator  iter = first;
      size_t  n =  frames_count;
      size_t  m =  angles.size ();
      double  a =  left_border,
              b = right_border,
              v = max_velosity;
      //------------------------------------------------------
      // std::vector<double>  new_frames (n);
      // size_t  n_frame = 1U;
      // for ( auto &frame : new_frames )
      // { frame = size_t (n_frame * m / n);  n_frame += 1.; }
      // --------------------------------
      // double*  new_angles = lagrange_value_1d (static_cast<int> (10), frames.data (), angles.data (),
      //                                          static_cast<int> (3), new_frames.data ());
      //------------------------------------------------------
      double  norm_sum = 0.;
      for ( size_t i = 0U; i < n; ++i )
      {
        double d = angles[size_t (i * m / n)]; // new_angles[i];

        norm_sum += d;
        *iter = d;
        ++iter;
      }
      //------------------------------------------------------
      // delete[] new_angles;
      //------------------------------------------------------
      iter = first;
      for ( size_t i = 0U; i < n; ++i )
      { /* apply normalization */
        *iter = *iter * (b - n * a) / norm_sum + a;
        if ( *iter > v )  *iter = v;
        ++iter;
      }
      //------------------------------------------------------
      if ( out ) MangoOutputFile (filename, first, n);
    }
  };
  //------------------------------------------------------
};
