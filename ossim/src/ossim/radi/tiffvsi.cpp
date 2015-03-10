
//#include "cpl_vsi.h"

//wwadd
#include <ossim/radi/tifvsi.h>
 #include <ossim/radi/vsi.h>

//////
#include <windows.h>
#include <errno.h>

// We avoid including xtiffio.h since it drags in the libgeotiff version
// of the VSI functions.

enum
{
    ENDIANNESS_NATIVE,
    ENDIANNESS_LITTLE,
    ENDIANNESS_BIG
};
static tsize_t
_tiffReadProc(thandle_t fd, tdata_t buf, tsize_t size)
{
    return WW_VSIFReadL( buf, 1, size, fd );
	
	//size_t WW_VSIFReadL( void * pBuffer, size_t nSize, size_t nCount, HANDLE hFile )

}

static tsize_t
_tiffWriteProc(thandle_t fd, tdata_t buf, tsize_t size)
{
    tsize_t nRet = WW_VSIFWriteL( buf, 1, size,   fd );
    //if (nRet < size)
    //{
    //    TIFFErrorExt( fd, "_tiffWriteProc", "%s", VSIStrerror( errno ) );
    //}
    return nRet;
}

static toff_t
_tiffSeekProc(thandle_t fd, toff_t off, int whence)
{
    if( WW_VSIFSeekL(  off, whence,fd ) == 0 )
        return (toff_t) WW_VSIFTellL(   fd );
    else
    {
//        TIFFErrorExt( fd, "_tiffSeekProc", "%s", VSIStrerror( errno ) );
        return (toff_t) -1;
    }
}

static int
_tiffCloseProc(thandle_t fd)
{
    return WW_VSIFCloseL(   fd );
}

static toff_t
_tiffSizeProc(thandle_t fd)
{
    GUIntBig  old_off;
    toff_t        file_size;

    old_off = WW_VSIFTellL(   fd );
    WW_VSIFSeekL(  0, SEEK_END ,fd );
    
    file_size = (toff_t) WW_VSIFTellL(  fd );
    WW_VSIFSeekL(  old_off, SEEK_SET , fd);

    return file_size;
}

static int
_tiffMapProc(thandle_t fd, tdata_t* pbase, toff_t* psize)
{
	(void) fd; (void) pbase; (void) psize;
	return (0);
}

static void
_tiffUnmapProc(thandle_t fd, tdata_t base, toff_t size)
{
	(void) fd; (void) base; (void) size;
}

/*
 * Open a TIFF file for read/writing.
 */
TIFF* WW_VSI_TIFFOpen(const char* name, const char* mode)
{
    static const char module[] = "TIFFOpen";
    int           i, a_out;
    char          access[32];
    HANDLE      fp;
    TIFF          *tif;

    a_out = 0;
    access[0] = '\0';
    for( i = 0; mode[i] != '\0'; i++ )
    {
        if( mode[i] == 'r'
            || mode[i] == 'w'
            || mode[i] == '+'
            || mode[i] == 'a' )
        {
            access[a_out++] = mode[i];
            access[a_out] = '\0';
        }
    }

    strcat( access, "b" );
                    
    fp = WW_VSIFOpenL( name, access );
    if (fp == NULL)         return ((TIFF *)0);

    tif = XTIFFClientOpen(name, mode,
                          (thandle_t) fp,
                          _tiffReadProc, _tiffWriteProc,
                          _tiffSeekProc, _tiffCloseProc, _tiffSizeProc,
                          _tiffMapProc, _tiffUnmapProc);

    if( tif == NULL )
        WW_VSIFCloseL( fp );
        
    return tif;
}

TIFF* WW_VSI_Create( const char* name,int nRasterXSizeRead, int nRasterYSizeRead, int bands,TIFFDataType eDT)
{
    double  dfUncompressedImageSize;
	  int                 bCreateBigTIFF = FALSE;
	  TIFF		*hTIFF;
    int                 nBlockXSize = 0, nBlockYSize = 0;
    int                 bTiled = FALSE;
    int                 nCompression = COMPRESSION_NONE;
	uint16              nSampleFormat;
	int nSamplesAccountedFor;
	int eEndianness = ENDIANNESS_NATIVE;
    dfUncompressedImageSize = 
        nRasterXSizeRead * ((double)nRasterYSizeRead) * bands * (GetDataTypeSize(eDT)/8);


	        if(dfUncompressedImageSize > 4200000000.0 )
            bCreateBigTIFF = TRUE;
			#ifdef CPL_LSB
            eEndianness = ENDIANNESS_BIG;
#else
            eEndianness = ENDIANNESS_LITTLE;
#endif
	 char szOpeningFlag[5];
    strcpy(szOpeningFlag, "w+");
    if (bCreateBigTIFF)
        strcat(szOpeningFlag, "8");
    if (eEndianness == ENDIANNESS_BIG)
        strcat(szOpeningFlag, "b");
    else if (eEndianness == ENDIANNESS_LITTLE)
        strcat(szOpeningFlag, "l");
    hTIFF = WW_VSI_TIFFOpen( name, szOpeningFlag );
	
/* -------------------------------------------------------------------- */
/*      Setup some standard flags.                                      */
/* -------------------------------------------------------------------- */
    TIFFSetField( hTIFF, TIFFTAG_IMAGEWIDTH, nRasterXSizeRead );
    TIFFSetField( hTIFF, TIFFTAG_IMAGELENGTH, nRasterYSizeRead );
    TIFFSetField( hTIFF, TIFFTAG_BITSPERSAMPLE, GetDataTypeSize(eDT) );
	//    const char *pszPixelType = CSLFetchNameValue( papszParmList, "PIXELTYPE" );
		   const char *pszPixelType =NULL;
    if( pszPixelType == NULL )
        pszPixelType = "";

	    if( (eDT == TIFF_SBYTE && EQUAL(pszPixelType,"SIGNEDBYTE"))
        || eDT == TIFF_SLONG || eDT == TIFF_SSHORT )
        nSampleFormat = SAMPLEFORMAT_INT;
    else if( eDT == TIFF_LONG || eDT == TIFF_SLONG )
        nSampleFormat = SAMPLEFORMAT_COMPLEXINT;
    else if( eDT == TIFF_RATIONAL || eDT == TIFF_SRATIONAL )
        nSampleFormat = SAMPLEFORMAT_IEEEFP;
    else if( eDT == TIFF_RATIONAL || eDT == TIFF_LONG8 )
        nSampleFormat = SAMPLEFORMAT_COMPLEXIEEEFP;
    else
        nSampleFormat = SAMPLEFORMAT_UINT;

	//nPlanar = PLANARCONFIG_CONTIG;
	    TIFFSetField( hTIFF, TIFFTAG_SAMPLEFORMAT, nSampleFormat );
    TIFFSetField( hTIFF, TIFFTAG_SAMPLESPERPIXEL, bands );
    TIFFSetField( hTIFF, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG );
	  if( bands == 3 && eDT == TIFF_BYTE )
        {
            TIFFSetField( hTIFF, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_RGB );
            nSamplesAccountedFor = 3;
        }
        else if( bands == 4 && eDT == TIFF_BYTE )
        {
  /*          uint16 v[1];

            v[0] = GTiffGetAlphaValue(CSLFetchNameValue(papszParmList,"ALPHA"),
                                      DEFAULT_ALPHA_TYPE);

            TIFFSetField(hTIFF, TIFFTAG_EXTRASAMPLES, 1, v);
            TIFFSetField( hTIFF, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_RGB );
            nSamplesAccountedFor = 4;*/
        }
        else
        {
            TIFFSetField( hTIFF, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK );
            nSamplesAccountedFor = 1;
        }

	/*	    if( bands > nSamplesAccountedFor )
    {
        uint16 *v;
        int i;
        int nExtraSamples = nBands - nSamplesAccountedFor;

        v = (uint16 *) CPLMalloc( sizeof(uint16) * nExtraSamples );

        v[0] = GTiffGetAlphaValue(CSLFetchNameValue(papszParmList, "ALPHA"),
                                  EXTRASAMPLE_UNSPECIFIED);

        for( i = 1; i < nExtraSamples; i++ )
            v[i] = EXTRASAMPLE_UNSPECIFIED;

        TIFFSetField(hTIFF, TIFFTAG_EXTRASAMPLES, nExtraSamples, v );
        
        CPLFree(v);
    }*/

			 TIFFSetField( hTIFF, TIFFTAG_COMPRESSION, nCompression );


			 
/* -------------------------------------------------------------------- */
/*      Setup tiling/stripping flags.                                   */
/* -------------------------------------------------------------------- */
    if( bTiled )
    {
        if( nBlockXSize == 0 )
            nBlockXSize = 256;
        
        if( nBlockYSize == 0 )
            nBlockYSize = 256;

        if (!TIFFSetField( hTIFF, TIFFTAG_TILEWIDTH, nBlockXSize ) ||
            !TIFFSetField( hTIFF, TIFFTAG_TILELENGTH, nBlockYSize ))
        {
            XTIFFClose(hTIFF);
            return NULL;
        }
    }
    else
    {
        uint32 nRowsPerStrip;

        if( nBlockYSize == 0 )
            nRowsPerStrip = MIN(nRasterYSizeRead, (int)TIFFDefaultStripSize(hTIFF,0));
        else
            nRowsPerStrip = nBlockYSize;
        
        TIFFSetField( hTIFF, TIFFTAG_ROWSPERSTRIP, nRowsPerStrip );
    }


	 return( hTIFF );
}