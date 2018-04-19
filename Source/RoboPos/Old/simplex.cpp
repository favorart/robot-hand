#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>


//****************************************************************************
namespace SimplexMethod
{
  // using namespace std;

  /* ������� ��� ������������ �����. ����� ��� M-������.
   * ����� ����� ������������ ��� ���������� - ������������ �����
   * � ����������� ��� ��������� ����� M.
   */
  class number
  {
  private:
    // ������������ �����
    double real;

    // ���������� ��� M
    double coef;

    // ���� "�� �����"
    bool is_nan;

  public:
    /**
    * �����������.
    * r - ������������ ����������
    * c - ����������� ��� M
    * num - ���� ���� �������� ����� false, �� ����� ��������� ��� NaN
    */
    number (double r = 0, double c = 0, bool num = true)
    {
      real = r;
      coef = c;
      is_nan = !num;
    }

    double get_real ()
    { return real; }
    double get_coef ()
    { return coef; }

    /**
    * ����� ���� ������ ���������� �������������� ����������.
    */

    number operator+  (number num)
    {
      number res;
      res.real = real + num.real;
      res.coef = coef + num.coef;
      return res;
    }

    void   operator+= (number num)
    {
      real += num.real;
      coef += num.coef;
    }

    number operator-  (number num)
    {
      number res;
      res.real = real - num.real;
      res.coef = coef - num.coef;
      return res;
    }

    void   operator-= (number num)
    {
      real -= num.real;
      coef -= num.coef;
    }

    number operator*  (number num)
    {
      number res;
      res.real = real * num.real;
      if ( real != 0 && coef == 0 && num.coef != 0 )
      {
        res.coef = real * num.coef;
      }
      else if ( num.real != 0 && num.coef == 0 && coef != 0 )
      {
        res.coef = coef * num.real;
      }
      return res;
    }

    void   operator*= (number num)
    {
      real *= num.real;
      if ( real != 0 && coef == 0 && num.coef != 0 )
      {
        coef = real * num.coef;
      }
      else if ( num.real != 0 && num.coef == 0 && coef != 0 )
      {
        coef = coef * num.real;
      }
    }

    number operator/  (number num)
    {
      number res;
      res.real = real / num.real;
      return res;
    }

    void   operator/= (number num)
    {
      real /= num.real;
    }

    bool   operator!= (number num)
    {
      return (real != num.real && coef != num.coef);
    }

    bool   operator!= (double num)
    {
      return (real != num);
    }

    bool   operator<  (number num)
    {
      if ( coef != num.coef )
      {
        return (coef < num.coef);
      }
      return (real < num.real);
    }

    bool   operator>  (number num)
    {
      if ( coef != num.coef )
      {
        return (coef > num.coef);
      }
      return (real > num.real);
    }

    bool   operator== (number num)
    {
      return (real == num.real && coef == num.coef);
    }

    bool   operator>= (number num)
    {
      return this->operator>(num) || this->operator==(num);
    }

    bool   operator<= (number num)
    {
      return this->operator<(num) || this->operator==(num);
    }

    /**
    * ����� � ���� ������ � �������
    * [R][+,-][C]M
    * ��� R - ������������ �����,
    * C - ����������� ��� M.
    */
    std::string str ()
    {
      if ( is_nan )
      {
        return std::string ("NaN");
      }
      if ( real == 0 && coef == 0 )
      {
        return std::string ("0");
      }
      std::stringstream ss;
      ss << std::setprecision (2);
      if ( real != 0 )
      {
        ss << real;
      }
      if ( coef != 0 )
      {
        if ( real != 0 && coef > 0 )
        {
          ss << '+';
        }
        else if ( coef < 0 )
        {
          ss << '-';
        }
        if ( coef != 1 && coef != -1 )
        {
          ss << abs (coef);
        }
        ss << 'M';
      }
      return ss.str ();
    }

    /*  ���������� ������� ����� ������������ ����������  */
    number fract ()
    {
      return (real > 0) ? number (real - floor (real)) : number (abs (floor (real)) + (real));
    }

    /*  ��������� �������� �� ����� (�����) �����  */
    bool is_integer ()
    { return  this->fract () < 0.00001; }

    /*  ������ �����  */
    number absolute ()
    { return number (fabs (real)); }
  };

  /****************************************************************************
  * ��������� ���������
  */
  typedef std::vector<number> vector_nums;
  typedef std::vector<int> vector_ints;
  typedef std::vector<std::vector<number> > matrix_nums;

  /****************************************************************************
  * ��������� ������� ��� ������ � �������
  */
  void print_line ()
  {
    std::cout << std::endl;
    for ( int i = 0; i < 80; ++i )
    {
      std::cout << (char) 0xCD;
    }
    std::cout << std::endl;
  }

  /**
  * ������ ������
  */
  void print_str (char* msg)
  {
    std::cout << std::endl << msg << std::endl;
  }

  /**
  * �������� �����
  */
  void print_num (char* msg, number n)
  {
    std::cout << std::endl << msg << n.str () << std::endl;
  }

  /**
  * �������� ������������ �����
  */
  void print_num (char* msg, double n)
  {
    std::cout << std::endl << msg << n << std::endl;
  }

  /**
  * �������� ����� �����
  */
  void print_num (char* msg, int n)
  {
    std::cout << std::endl << msg << n << std::endl;
  }

  void print_num (char* msg, size_t n)
  {
    std::cout << std::endl << msg << n << std::endl;
  }

  /**
  * ������ ������
  */
  void print_vector (char* msg, vector_nums v)
  {
    int i, j;
    int width = 8;

    std::cout << std::endl << msg << std::endl << (char) 0xDA;

    for ( i = 0; i < v.size (); ++i )
    {
      for ( j = 0; j < width - 1; ++j )
      {
        std::cout << (char) 0xC4;
      }
      if ( i < v.size () - 1 )
      {
        std::cout << (char) 0xC2;
      }
    }

    std::cout << (char) 0xBF << std::endl << (char) 0xB3;

    for ( i = 0; i < v.size (); ++i )
    {
      std::cout << std::setw (width - 1) << v[i].str () << (char) 0xB3;
    }

    std::cout << std::endl << (char) 0xC0;

    for ( i = 0; i < v.size (); ++i )
    {
      for ( j = 0; j < width - 1; ++j )
      {
        std::cout << (char) 0xC4;
      }
      if ( i < v.size () - 1 )
      {
        std::cout << (char) 0xC1;
      }
    }

    std::cout << (char) 0xD9 << std::endl;
  }

  /**
  * �������� ������ ����� �����
  */
  void print_vector (char* msg, std::vector<int> v)
  {
    std::cout << std::endl << msg << std::endl << (char) 0xDA;
    int i, j;
    int width = 8;

    for ( i = 0; i < v.size (); ++i )
    {
      for ( j = 0; j < width - 1; ++j )
      {
        std::cout << (char) 0xC4;
      }
      if ( i < v.size () - 1 )
      {
        std::cout << (char) 0xC2;
      }
    }

    std::cout << (char) 0xBF << std::endl << (char) 0xB3;

    for ( i = 0; i < v.size (); ++i )
    {
      std::cout << std::setw (width - 1) << v[i] << (char) 0xB3;
    }

    std::cout << std::endl << (char) 0xC0;

    for ( i = 0; i < v.size (); ++i )
    {
      for ( j = 0; j < width - 1; ++j )
      {
        std::cout << (char) 0xC4;
      }
      if ( i < v.size () - 1 )
      {
        std::cout << (char) 0xC1;
      }
    }

    std::cout << (char) 0xD9 << std::endl;
  }

  /**
  * �������� �������
  */
  void print_matrix (char* msg, matrix_nums m)
  {
    int i, j, k;
    int width = 8;
    std::cout << std::endl << msg << std::endl << (char) 0xDA;

    for ( i = 0; i < m[0].size (); ++i )
    {
      for ( j = 0; j < width - 1; ++j )
      {
        std::cout << (char) 0xC4;
      }
      if ( i < m[0].size () - 1 )
      {
        std::cout << (char) 0xC2;
      }
    }
    std::cout << (char) 0xBF << std::endl;

    for ( i = 0; i < m.size (); ++i )
    {
      std::cout << (char) 0xB3;
      for ( j = 0; j < m[i].size (); ++j )
      {
        std::cout << std::setw (width - 1) << std::right << m[i][j].str () << (char) 0xB3;
      }
      std::cout << std::endl;
      if ( i < m.size () - 1 )
      {
        std::cout << (char) 0xC3;
        for ( j = 0; j < m[i].size (); ++j )
        {
          for ( k = 0; k < width - 1; ++k )
          {
            std::cout << (char) 0xC4;
          }
          if ( j < m[i].size () - 1 )
          {
            std::cout << (char) 0xC5;
          }
          else
          {
            std::cout << (char) 0xB4;
          }
        }
        std::cout << std::endl;
      }
    }

    std::cout << (char) 0xC0;
    for ( i = 0; i < m[0].size (); ++i )
    {
      for ( j = 0; j < width - 1; ++j )
      {
        std::cout << (char) 0xC4;
      }
      if ( i < m[0].size () - 1 )
      {
        std::cout << (char) 0xC1;
      }
    }
    std::cout << (char) 0xD9 << std::endl;

    std::cout << std::endl;
  }

  // ������������ ���������� �������� ������
#define DEBUG_MAX_ITER_NUM 10

/****************************************************************************
* ����������
*/
  class simplex_error
  {
  public:
    std::string msg;
    simplex_error (char* m)
    {
      msg = std::string (m);
    }
  };

  /****************************************************************************
  * ��������� ������� ��������-������
  */
  struct opt_table
  {
    vector_nums cib;
    vector_ints bp;
    vector_nums br;
    vector_nums c;
    matrix_nums a;
    vector_nums z;
    vector_nums delta;
  };

  /****************************************************************************
  * �������� ��������������� ��������� �������
  */
  bool vector_is_integer (vector_nums v)
  {
    for ( int i = 0; i < v.size (); ++i )
    {
      if ( !v[i].is_integer () )
      {
        return false;
      }
    }
    return true;
  }

  /****************************************************************************
  * �������� ����� ������ ���������
  */
  opt_table simplex_max (opt_table task, bool negative = false, bool int_break = false)
  {
    size_t  i, j;
    size_t  n = task.c.size ();
    size_t  m = task.bp.size ();

    vector_nums cib (m);
    vector_nums z (n);
    vector_nums delta (n);
    vector_nums min (m);
    matrix_nums tmp_a (m, vector_nums (n));
    vector_nums tmp_br (m);

    for ( i = 0; i < m; ++i )
    {
      if ( task.bp[i] == -1 )
      {
        continue;
      }
      cib[i] = task.c[task.bp[i]];
    }

    for ( int k = 0; k < DEBUG_MAX_ITER_NUM; ++k )
    {
      print_line ();

      // ���������� ����������� �������
      number deltaExtr;
      if ( negative )
      { deltaExtr = 1000000; }
      else
      { deltaExtr = delta[0]; }
      size_t  r = -1;
      for ( j = 0; j < n; ++j )
      {
        z[j] = number ();
        for ( i = 0; i < m; ++i )
        {
          if ( task.bp[i] == -1 )
          { continue; }
          z[j] += cib[i] * task.a[i][j];
        }
        delta[j] = task.c[j] - z[j];

        // ���������� �� ������ ������������� ������
        if ( negative )
        {
          if ( delta[j] < number (0) && delta[j].absolute () < deltaExtr )
          {
            deltaExtr = delta[j].absolute ();
            r = j;
          }

          // ���������� ������
        }
        else
        {
          if ( deltaExtr < delta[j] )
          {
            deltaExtr = delta[j];
            r = j;
          }
        }
      }

      print_vector ("Coefs of variables (c)", task.c);
      print_vector ("Coefs of basic variables (cib)", cib);
      print_vector ("Basic variables numbers (bp)", task.bp);
      print_vector ("Basic variables values (br)", task.br);
      print_matrix ("Coefs of system (a)", task.a);
      print_vector ("Values (z)", z);
      print_vector ("Relative valuations (delta)", delta);
      print_num    ("Permitted column number (r): ", r);

      if ( r == -1 )
      { break; }

      // ���������� ����������� ������
      number min_row = number (100000);
      int s = -1;
      for ( i = 0; i < m; ++i )
      {
        if ( task.bp[i] == -1 )
        {
          s = i;
          break;
        }

        if ( task.a[i][r] != 0 )
        {
          min[i] = task.br[i] / task.a[i][r];
        }
        else
        {
          min[i] = number (0, 0, false);
          continue;
        }
        if ( min[i] < 0 )
        {
          min[i] = number (0, 0, false);
          continue;
        }
        if ( min_row > min[i] )
        {
          min_row = min[i];
          s = i;
        }
      }

      // print_vector("Relations x/a (br[i]/a[i][r], min): ", min);

      if ( s == -1 )
      { throw simplex_error ("Permitted row not found"); }

      print_num ("Permitted row number (s): ", s);

      number element = task.a[s][r];

      print_num ("Permitted value (element): ", element);

      for ( i = 0; i < m; ++i )
      {
        for ( j = 0; j < n; ++j )
        { tmp_a[i][j] = task.a[i][j]; }
        tmp_br[i] = task.br[i];
      }

      // ������ ���������� � �����

      task.bp[s] = r;
      cib[s] = task.c[r];
      for ( j = 0; j < n; ++j )
      { task.a[s][j] /= element; }
      task.br[s] /= element;


      for ( i = 0; i < m; ++i )
      {
        if ( i == s )
        { continue; }

        number air = tmp_a[i][r];

        for ( j = 0; j < n; ++j )
        { task.a[i][j] -= (air * tmp_a[s][j]) / element; }
        task.br[i] -= (air * tmp_br[s]) / element;
      }

      if ( int_break && vector_is_integer (task.br) )
      { break; }
    }

    print_line ();
    print_vector ("Result: ", task.br);

    opt_table tab;
    tab.cib = cib;
    tab.bp = task.bp;
    tab.br = task.br;
    tab.c = task.c;
    tab.a = task.a;
    tab.z = z;
    tab.delta = delta;

    return tab;
  }

  /****************************************************************************
  * ��������� �� ������ 1
  */
  bool cmp_mod_one (number a, number b)
  { return (a - b).is_integer (); }

  /****************************************************************************
  * �������� �������� �������� � �������
  */
  bool in_vector (int a, vector_ints v)
  {
    for ( int i = 0; i < v.size (); ++i )
    {
      if ( a == v[i] )
      {
        return true;
      }
    }
    return false;
  }

  /****************************************************************************
  * ����� ������
  */
  opt_table gomory (opt_table tab)
  {
    tab = simplex_max (tab, false, false);

    int i, j;
    // ����� ���������� ����������
    size_t  n = tab.c.size ();
    // ���������� �������� ����������
    size_t  m = tab.bp.size ();

    for ( size_t k = 0; k < DEBUG_MAX_ITER_NUM; ++k )
    {
      print_line ();
      std::cout << "Gomory iteration #" << k << std::endl;
      print_line ();

      // ������� ������
      if ( vector_is_integer (tab.br) )
      { return tab; }

      // ������������ ������� �����
      number max_fract = 0;

      // ����� ������ � ������������ ������� ������
      int r = -1;

      // ����� ���������� � ������������ ������� ������
      for ( i = 0; i < m; ++i )
      {
        number fract = tab.br[i].fract ();
        if ( fract >= max_fract )
        {
          max_fract = fract;
          r = i;
        }
      }

      // ���������� ����� �������, �������� ���� ������� � ���� ������
      ++n;
      ++m;
      opt_table ext_tab;

      // ������ ������������� ���� ����������
      ext_tab.c = vector_nums (n);
      for ( j = 0; j < n - 1; ++j )
      {
        ext_tab.c[j] = tab.c[j];
      }
      ext_tab.c[n - 1] = 0;

      // ������� a
      ext_tab.a = matrix_nums (m, vector_nums (n));
      for ( i = 0; i < m - 1; ++i )
      {
        for ( j = 0; j < n - 1; ++j )
        {
          ext_tab.a[i][j] = tab.a[i][j];
        }
        ext_tab.a[i][n - 1] = 0;
      }
      for ( j = 0; j < n - 1; ++j )
      {
        if ( in_vector (j, tab.bp) )
        {
          ext_tab.a[m - 1][j] = 0;
        }
        else
        {
          ext_tab.a[m - 1][j] = tab.a[r][j].fract ();
        }
      }
      ext_tab.a[m - 1][n - 1] = -1;

      // ������-�������
      ext_tab.bp = vector_ints (m);
      ext_tab.br = vector_nums (m);
      for ( i = 0; i < m - 1; ++i )
      {
        //ext_tab.cib[i] = tab.cib[i];
        ext_tab.bp[i] = tab.bp[i];
        ext_tab.br[i] = tab.br[i];
      }
      // ext_tab.cib[m - 1] = ?
      ext_tab.bp[m - 1] = -1; // ?
      ext_tab.br[m - 1] = tab.br[r].fract ();

      tab = simplex_max (ext_tab, true, true);
    }

    return tab;
  }
};
/****************************************************************************
* ����, ���
*/
using namespace SimplexMethod;

int main1 (int argc, char* argv[])
{
  {
    int n = 5;
    int m = 2;

    vector_nums c (n);
    c[0] = number (1);
    c[1] = number (-1);
    c[2] = number (0);
    c[3] = number (0);
    c[4] = number (0, -1);

    matrix_nums a (m, vector_nums (n));
    a[0][0] = number (-1);
    a[0][1] = number (2);
    a[0][2] = number (-1);
    a[0][3] = number (0);
    a[0][4] = number (1);
    a[1][0] = number (3);
    a[1][1] = number (2);
    a[1][2] = number (0);
    a[1][3] = number (1);
    a[1][4] = number (0);

    vector_ints bp (m);
    bp[0] = 4;
    bp[1] = 3;

    vector_nums br (m);
    br[0] = number (4);
    br[1] = number (14);

    opt_table task;
    task.a = a;
    task.bp = bp;
    task.br = br;
    task.c = c;

    /*opt_table task;

    task.c = vector_nums(n);
    task.c[0] = number(2);
    task.c[1] = number(1);
    task.c[2] = number(0);

    task.a = matrix_nums(m, vector_nums(n));
    task.a[0][0] = number(15);
    task.a[0][1] = number(30);
    task.a[0][2] = number(1);

    task.bp = vector_ints(m);
    task.bp[0] = 2;

    task.br = vector_nums(m);
    task.br[0] = number(96);*/

    /*opt_table task;

    task.c = vector_nums(n);
    task.c[0] = number(1);
    task.c[1] = number(0);
    task.c[2] = number(0);
    task.c[3] = number(0);
    task.c[4] = number(0);
    task.c[5] = number(0, -1);

    task.a = matrix_nums(m, vector_nums(n));

    task.a[0][0] = number(1);
    task.a[0][1] = number(-2);
    task.a[0][2] = number(1);
    task.a[0][3] = number(0);
    task.a[0][4] = number(0);
    task.a[0][5] = number(0);

    task.a[1][0] = number(1);
    task.a[1][1] = number(-1);
    task.a[1][2] = number(0);
    task.a[1][3] = number(-1);
    task.a[1][4] = number(0);
    task.a[1][5] = number(1);

    task.a[2][0] = number(1);
    task.a[2][1] = number(1);
    task.a[2][2] = number(0);
    task.a[2][3] = number(0);
    task.a[2][4] = number(1);
    task.a[2][5] = number(0);

    task.bp = vector_ints(m);
    task.bp[0] = 2;
    task.bp[1] = 5;
    task.bp[2] = 4;

    task.br = vector_nums(m);
    task.br[0] = number(4);
    task.br[1] = number(-1);
    task.br[2] = number(2);*/

    gomory (task);
  }
  //****************************************************************************

  {

    int n = 4;
    int m = 2;

    vector_nums c (n);
    c[0] = number (1);
    c[1] = number (1);
    c[2] = number (1);
    c[3] = number (1);

    matrix_nums a (m, vector_nums (n));
    a[0][0] = number (0);
    a[0][1] = number (230);
    a[0][2] = number (150);
    a[0][3] = number (0);

    a[1][0] = number (0);
    a[1][1] = number (220);
    a[1][2] = number (160);
    a[1][3] = number (0);

    vector_ints bp (m);
    bp[0] = 0.0178;
    bp[1] = 0.0179;

    vector_nums br (m);
    br[0] = number (4);
    br[1] = number (14);

    opt_table task;
    task.a = a;
    task.bp = bp;
    task.br = br;
    task.c = c;
  }

  return 0;
}
//****************************************************************************
