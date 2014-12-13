#include <ossim/init/ossimInit.h>
#include <ossim/base/ossimArgumentParser.h>
#include <ossim/base/ossimApplicationUsage.h>
#include <ossim/base/ossimGeoidManager.h>
#include <ossim/elevation/ossimElevManager.h>

#include <cstdlib>
#include <iomanip>

int main(int argc, char* argv[])
{
   ossimArgumentParser argumentParser(&argc, argv);
   ossimInit::instance()->addOptions(argumentParser);
   ossimInit::instance()->initialize(argumentParser);
   argumentParser.getApplicationUsage()->setApplicationName(
      argumentParser.getApplicationName());
   argumentParser.getApplicationUsage()->setDescription(
      argumentParser.getApplicationName()+
      " Returns a MSL and ellipoid height given a lat lon position");
   argumentParser.getApplicationUsage()->addCommandLineOption(
      "-h or --help", "Shows help");
   argumentParser.getApplicationUsage()->setCommandLineUsage(
      argumentParser.getApplicationName()+
      " <lat degrees> <lon degrees>");

  // if(argumentParser.read("-h") || argumentParser.read("--help")||
  //    (argc != 3))
 //  {
 //     argumentParser.getApplicationUsage()->write(std::cout);
 //     exit(0);
 //  }
      if(argumentParser.read("-h") || argumentParser.read("--help"))
   {
      argumentParser.getApplicationUsage()->write(std::cout);
      exit(0);
   }
   if (argumentParser.errors())
   {
      argumentParser.writeErrorMessages(std::cout);
      exit(1);
   }

   ossimGpt gpt;
  // gpt.latd(ossimString(argumentParser.argv()[1]).toDouble());
  // gpt.lond(ossimString(argumentParser.argv()[2]).toDouble());
      gpt.latd(ossimString("40.2").toDouble());
   gpt.lond(ossimString("91.1").toDouble());
   ossim_float64 hgtAboveMsl       = ossimElevManager::instance()->
      getHeightAboveMSL(gpt);
   ossim_float64 hgtAboveEllipsoid = ossimElevManager::instance()->
      getHeightAboveEllipsoid(gpt);
   ossim_float64 geoidOffset       = ossimGeoidManager::instance()->offsetFromEllipsoid(gpt);
   ossim_float64 mslOffset       = 0.0;
    std::cout << std::setprecision(15) << hgtAboveMsl;

         gpt.latd(ossimString("43.1").toDouble());
   gpt.lond(ossimString("99.8").toDouble());
    hgtAboveMsl       = ossimElevManager::instance()->
      getHeightAboveMSL(gpt);
    hgtAboveEllipsoid = ossimElevManager::instance()->
      getHeightAboveEllipsoid(gpt);

	 std::cout << std::setprecision(15) << hgtAboveMsl;
   

	          gpt.latd(ossimString("40.2").toDouble());
   gpt.lond(ossimString("91.1").toDouble());
    hgtAboveMsl       = ossimElevManager::instance()->
      getHeightAboveMSL(gpt);
    hgtAboveEllipsoid = ossimElevManager::instance()->
      getHeightAboveEllipsoid(gpt);

	 std::cout << std::setprecision(15) << hgtAboveMsl;


   if(ossim::isnan(hgtAboveEllipsoid)||ossim::isnan(hgtAboveMsl))
   {
      mslOffset = ossim::nan();
   }
   else
   {
      mslOffset = hgtAboveEllipsoid - hgtAboveMsl;
   }

#if 1 /* Tmp until this functionality is added back. */
   std::vector<ossimFilename> cellList;
   ossimElevManager::instance()->getOpenCellList(cellList);

   if (!cellList.empty())
   {
      std::cout << "Opened cell:  " << cellList[0] << std::endl;
   }
   else
   {
      std::cout << "Did not find cell for point!" << std::endl;
   }
#endif
   
   std::cout << "MSL to ellipsoid delta:   ";
   if (!ossim::isnan(mslOffset))
   {
      std::cout << std::setprecision(15) << mslOffset;
   }
   else
   {
      std::cout << "nan";
   }
   std::cout << "\nHeight above MSL:        ";
   if (!ossim::isnan(hgtAboveMsl))
   {
      std::cout << std::setprecision(15) << hgtAboveMsl;
   }
   else
   {
      std::cout << "nan";
   }
   std::cout << "\nHeight above ellipsoid:  ";
   if (!ossim::isnan(hgtAboveEllipsoid))
   {
      std::cout << std::setprecision(15) << hgtAboveEllipsoid << std::endl;
   }
   else
   {
      std::cout << "nan" << std::endl;
   }
   std::cout << "Geoid value:  ";
   if (!ossim::isnan(geoidOffset))
   {
      std::cout << std::setprecision(15) << geoidOffset << std::endl;
   }
   else
   {
      std::cout << "nan" << std::endl;
   }
   
   return 0;
}
