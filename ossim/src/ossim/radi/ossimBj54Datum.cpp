#include <ossim/radi/ossimBj54Datum.h>
#include <ossim/base/ossimConstants.h>
#include <ossim/base/ossimEllipsoidFactory.h>
#include <ossim/base/ossimEllipsoid.h>
#include <ossim/base/ossimNotifyContext.h>
ossimBj54Datum::ossimBj54Datum()
   :ossimThreeParamDatum("BJ54",
                         "BeiJing System 1954",
                         ossimEllipsoidFactory::instance()->bj54(),
						 -1, -1, -1,-M_PI, M_PI,
                         M_PI*36/180.0, M_PI*89/180.0, 
                         28, -130, -95)
{
   if(!ellipsoid())
   {
   }
}
