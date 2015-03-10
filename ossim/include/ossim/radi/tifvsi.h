
#ifndef TIFVSI_H_INCLUDED
#define TIFVSI_H_INCLUDED

#include "tiffio.h"
#include "xtiffio.h"
#include "vsi.h"
TIFF* WW_VSI_TIFFOpen(const char* name, const char* mode);
TIFF* WW_VSI_Create(const char* name, int nRasterXSizeRead, int nRasterYSizeRead, int bands, TIFFDataType eDT);
#endif // TIFVSI_H_INCLUDED
