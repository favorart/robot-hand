#include "StdAfx.h"

#ifndef  _TARGET_H_
#define  _TARGET_H_

#include "MyWindow.h"
//------------------------------------------------------------------------------
class Target
{ 
typedef std::set<Point> set_t;

protected:
  //---------------------------------
  set_t  coords_; // main content
  double average_distance_;

public:
	//---------------------------------
  // virtual flann::Matrix<double>  Generate () =0;
  //---------------------------------
	Target (): coords_ (), average_distance_ (0.0) {}
	uint_t  CoordsCount  () const { return coords_.size (); }

	// virtual Point*  HitH (const Point &h) const;
	virtual void    draw (const HDC &hdc, const HPEN &hPen) const =0;
	
	virtual ~Target () {}
	//---------------------------------
};

class RecTarget : public Target
{ 
protected:
	//---rectangle range---------------
	double  lft, rgh, top, btm;		
	uint_t  c_rows, c_cols;
  
public:
	//---------------------------------
	// virtual flann::Matrix<double>  Generate ();

  //---------------------------------
	RecTarget () : Target (), c_rows (0U), c_cols (0U)
		{ lft = 0.0; rgh = 0.0;
      top = 0.0; btm = 0.0;
		}
	RecTarget (uint_t r, uint_t c, const Point &min, const Point &max)
			: Target (), c_rows (r), c_cols (c)
		{ lft = min.x; rgh = max.x;
      top = max.y; btm = min.y; 

			// Generate ();				
		}
	RecTarget (uint_t r, uint_t c, double lft, double rgh, double top, double btm)
			: Target (), c_rows (r), c_cols (c)
		{ this->lft = lft; this->rgh = rgh;
			this->top = top; this->btm = btm;

			// Generate ();		
		}

  // virtual void  Near ();
	virtual void  draw (const HDC &hdc, const HPEN &hPen) const; // !!! Generate...
	//---------------------------------
};
//------------------------------------------------------------------------------
#endif // _TARGET_H_
