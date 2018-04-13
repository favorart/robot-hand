
namespace SimplexMethod
{
  extern "C"
  {
#include "lp_lib.h"

    int  calculate (IN  int nCols /* variables in the model */,
                    IN  int nRows,
                    IN  double** rows,
                    IN  double*  rights,
                    IN  double*  objectives,
                    OUT int* answer,
                    IN  int verbose)
    {
      lprec *lp;
      int result = 0;

      char *str = NULL;
      int *colno = NULL;
      double *row = NULL;

      /*  We will build the model row by row
       *  So we start with creating a model
       *  with 0 rows and 2 columns
       */
      if ( !(lp = make_lp (0, nCols)) )
      { 
        /* couldn't construct a new model... */
        result = 1;
        goto RESULT;
      }

      if ( !(str = (char*) malloc ((log10 (nCols) + 10) * sizeof (*str))) )
      {
        result = 2;
        goto RESULT;
      }

      /*  let us name our variables. Not required, 
       *  but can be useful for debugging
       */
      for ( int i = 1; i <= nCols; ++i )
      {       
        str[0] = 't';
        _itoa (i, str + 1, 10);

        set_col_name (lp, i, str);
        // set_int (lp, i, TRUE);
      }

      /* create space large enough for one row */
      colno = (int   *) malloc (nCols * sizeof (*colno));
      row   = (double*) malloc (nCols * sizeof (*row));

      if ( (colno == NULL) || (row == NULL) )
      {
        result = 2;
        goto RESULT;
      }

      for ( int j = 0; j < nCols; ++j )
      { colno[j] = j + 1; /* (j + 1) column */ }

      /* makes building the model faster if it is done rows by row */
      set_add_rowmode (lp, TRUE);
      
      for ( int i = 0; i < nRows; ++i )
      {
        // /* construct j row */
        // for ( int j = 0; j < nCols; ++j )
        // { row[j] = ??? ; }

        /* (210 * t2 + 156 * t3 == 0.0178) */
        /* (230 * t2 + 160 * t3 == 0.0176) */

        /* add the row to lp_solve */
        if ( !add_constraintex (lp, nCols, rows[i], colno, EQ, rights[i]) )
        {
          result = 3;
          goto RESULT;
        }
      }

      /* rowmode should be turned off again when done building the model */
      set_add_rowmode (lp, FALSE); 

      // /* set the objective function  */
      // for ( int j = 0; j < nCols; ++j )
      // { row[j] = objectives[j]; }

      /* (t1 + t2 + t3 + t4) */

      /* set the objective in lp_solve */
      if ( !set_obj_fnex (lp, nCols, objectives, colno) )
      {
        result = 4;
        goto RESULT;
      }
      
      /* set the object direction to maximize */
      set_minim (lp);

      if ( verbose )
      {
        /* just out of curioucity, now show the model in lp format on screen */
        /* this only works if this is a console application. If not, use write_lp and a filename */
        write_LP (lp, stdout);
        /* write_lp(lp, "model.lp"); */
      }
      
      /* I only want to see important messages on screen while solving */
      set_verbose (lp, IMPORTANT);
      
      /* Now let lpsolve calculate a solution */
      result = solve (lp);
      if ( result == OPTIMAL )
      { result = 0; }
      else
      {
        result = 5;
        goto RESULT;
      }

      /*  a solution is calculated,
       *  now lets get some results
       */
      if ( verbose )
      {
        /* objective value */
        printf ("Objective value: %f\n", get_objective (lp));
      }

      /* variable values */
      get_variables (lp, row);
      for ( int j = 0; j < nCols; j++ )
      {
        if ( verbose )
          printf ("%s: %f\n", get_col_name (lp, j + 1), row[j]);
        
        answer[j] = row[j];
      }
      /* we are done now */

RESULT:;
      /* free allocated memory */
      if ( str != NULL )free (str);
      if ( row != NULL ) free (row);
      if ( colno != NULL ) free (colno);

      if ( lp != NULL )
      {
        /* clean up such that all used memory by lpsolve is freed */
        delete_lp (lp);
      }

      return result;
    }
  };

  // void  calculate (std::vector<double> &rows,
  //                  std::vector<int> &answer)
  // {
  //   calculate (rows.data (), answer.data ());
  // }

};

// int main ()
// { demo (); }

//   When this is run, the following is shown on screen :
//   
//   /* Objective function */
//   max : +143 x + 60 y;
//   
//   /* Constraints */
//   +120 x + 210 y <= 15000;
//   +110 x + 30 y <= 4000;
//   +x + y <= 75;
//   Objective value : 6315.625000
//   x : 21.875000
//   y : 53.125000
//   
// Note that this example is very limited. It is also possible to set bounds on variables,
// ranges on constraints, define variables as integer, get more result information
// changing solver options and parameters and much more. See lp_solve API reference
// for an overview of the API to do this.
// file:///C:/Users/MainUser/Desktop/lp_solve_5.5.2.0_doc/lp_solveAPIreference.htm
//