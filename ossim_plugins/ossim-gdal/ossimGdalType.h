#ifndef ossimGdalType_HEADER
#define ossimGdalType_HEADER
#include <ossim/base/ossimConstants.h>
#include <gdal.h>
class ossimGdalType
{
public:
   ossimScalarType toOssim(GDALDataType gdalType)const;
   GDALDataType    toGdal(ossimScalarType)const;
};
#endif
