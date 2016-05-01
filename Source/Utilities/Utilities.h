#include "StdAfx.h"

#ifndef  _UTILITIES_H_
#define  _UTILITIES_H_
//------------------------------------------------------------------------------
/* �������� ���� ����� */
template <typename T> int  sign (T val)
{ return (T (0) < val) - (val < T (0)); }
//------------------------------------------------------------------------------
/*  ���������� � ������������:
 *    currents   - ������� ����������
 *    n_alphabet - ������ �������� { 0, 1, ... k }
 */
static bool  next_placement_repeats (std::vector<int> &currents, int n_alphabet)
{
  bool    result = true;
  size_t  position = currents.size ();
  while ( position > 0U && result )
  {
    ++currents[position - 1U];
    result = (currents[position - 1U] == n_alphabet);
    if ( result )
    { currents[position - 1U] = 0U; }
    --position;
  }
  return (!result);
}
//------------------------------------------------------------------------------
#endif //  _UTILITIES_H_
