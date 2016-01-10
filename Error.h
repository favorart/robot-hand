#include "StdAfx.h"

#ifndef  _ERROR_H_
#define  _ERROR_H_

//---error-----------------------------------------------------------------------
typedef enum { ERR_NO, ERR_NMEM, ERR_FINC, ERR_FEND /* ... */ } terr;
/* inline */ static void  SetError_ (terr err)
{ switch (err)
	 { default:       MessageBox (NULL, _T("Unknown."               ), _T("Error"), MB_OK | MB_ICONWARNING); break;																								 
	 	 case ERR_FINC: MessageBox (NULL, _T("The invalid input file."), _T("Error"), MB_OK | MB_ICONWARNING); break;																								 
	 	 case ERR_FEND: MessageBox (NULL, _T("Unexpected end of file."), _T("Error"), MB_OK | MB_ICONWARNING); break;																								 
	 	 case ERR_NMEM: MessageBox (NULL, _T("Cannot allocate memory."), _T("Error"), MB_OK | MB_ICONWARNING); break;
	 }
	 /* exit(1); */
}
//-------------------------------------------------------------------------------
#endif // _ERROR_H_
