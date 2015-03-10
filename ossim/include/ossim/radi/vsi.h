#ifndef VSI_H_INCLUDED
#define VSI_H_INCLUDED

#include <windows.h>
#include <tiff.h>
#    define STRCASECMP(a,b)         (stricmp(a,b))
#    define STRNCASECMP(a,b,n)      (strnicmp(a,b,n))

#  define EQUALN(a,b,n)           (STRNCASECMP(a,b,n)==0)
#  define EQUAL(a,b)              (STRCASECMP(a,b)==0)

  typedef short           GInt16;
typedef unsigned short  GUInt16;
typedef unsigned char   GByte;

  typedef int             GInt32;
typedef unsigned int    GUInt32;

typedef __int64          GIntBig;
typedef unsigned __int64 GUIntBig;


#ifndef NULL
#  define NULL  0
#endif

#ifndef FALSE
#  define FALSE 0
#endif

#ifndef TRUE
#  define TRUE  1
#endif

#ifndef MAX
#  define MIN(a,b)      ((a<b) ? a : b)
#  define MAX(a,b)      ((a>b) ? a : b)
#endif

#ifndef ABS
#  define ABS(x)        ((x<0) ? (-1*(x)) : x)
#endif

#ifndef M_PI
# define M_PI		3.14159265358979323846	/* pi */
#endif


int  GetDataTypeSize( TIFFDataType eDataType );
HANDLE WW_VSIFOpenL(const char* name, const char* mode);
size_t WW_VSIFReadL( void * pBuffer, size_t nSize, size_t nCount, HANDLE hFile );
 int WW_VSIFCloseL( HANDLE hFile );
  size_t WW_VSIFWriteL( const void *pBuffer, size_t nSize, size_t nCount, HANDLE hFile);
  int WW_VSIFSeekL( GUIntBig nOffset, int nWhence ,HANDLE hFile);
 GUIntBig WW_VSIFTellL(HANDLE hFile);

#endif // VSI_H_INCLUDED