#include <ossim/radi/ossimXiAn80Datum.h>
#include <ossim/base/ossimConstants.h>
#include <ossim/base/ossimEllipsoidFactory.h>
#include <ossim/base/ossimEllipsoid.h>
#include <ossim/base/ossimNotifyContext.h>
ossimXiAn80Datum::ossimXiAn80Datum()
   :ossimThreeParamDatum("XiAn80",
                         "XiAn System 1980",
                         ossimEllipsoidFactory::instance()->xian80(),
						 -1, -1, -1,-M_PI, M_PI,
                         M_PI*36/180.0, M_PI*89/180.0, 
                         28, -130, -95)
{
   if(!ellipsoid())
   {
   }
}
