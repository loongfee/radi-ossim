//----------------------------------------------------------------------------
//
// "Copyright Centre National d'Etudes Spatiales"
//
// License:  LGPL
//
// See LICENSE.txt file in the top level directory for more details.
//
//----------------------------------------------------------------------------
// $Id$


#include "ossimPluginProjectionFactory.h"
#include <ossim/base/ossimKeywordNames.h>
#include <ossim/base/ossimRefPtr.h>
#include <ossim/projection/ossimProjection.h>
#include "ossimRadarSatModel.h"
#include "ossimEnvisatAsarModel.h"
#include "ossimTerraSarModel.h"
//#include <ossim/projection/ossimCosmoSkymedModel.h>
#include "ossimRadarSat2Model.h"
#include "ossimErsSarModel.h"
#include "ossimAlosPalsarModel.h"
#include "ossimPleiadesModel.h"
#include <ossim/base/ossimNotifyContext.h>
#include "ossimTileMapModel.h"

// new added model
#include <radi/ossimHj1Model.h>
#include <radi/radiZY3Model.h>
#include <radi/radiRpcModel.h>
#include <radi/radiCbers04Model.h>
#include <ossimRadarSat2RPCModel.h>

//***
// Define Trace flags for use within this file:
//***
#include <ossim/base/ossimTrace.h>
static ossimTrace traceExec  = ossimTrace("ossimPluginProjectionFactory:exec");
static ossimTrace traceDebug = ossimTrace("ossimPluginProjectionFactory:debug");


#include <ossimFormosatModel.h>
#include <ossimFormosatDimapSupportData.h>

namespace ossimplugins
{


ossimPluginProjectionFactory* ossimPluginProjectionFactory::instance()
{
   static ossimPluginProjectionFactory* factoryInstance =
      new ossimPluginProjectionFactory();

   return factoryInstance;
}
   
ossimProjection* ossimPluginProjectionFactory::createProjection(
   const ossimFilename& filename, ossim_uint32 /*entryIdx*/)const
{
   static const char MODULE[] = "ossimPluginProjectionFactory::createProjection(ossimFilename& filename)";
   ossimRefPtr<ossimProjection> projection = 0;
   //traceDebug.setTraceFlag(true);

   if(traceDebug())
   {
    	ossimNotify(ossimNotifyLevel_DEBUG)
        	   << MODULE << " DEBUG: testing ossimRadarSat2Model" << std::endl;
   }

   if ( !projection )
   {
      ossimRefPtr<ossimRadarSat2Model> model = new ossimRadarSat2Model();
      if ( model->open(filename) )
      {
         // Check if a coarse grid was generated, and use it instead:
         projection = model->getReplacementOcgModel().get();
         if (projection.valid())
            model = 0; // Have OCG, don't need this one anymore
         else
            projection = model.get();
      }
      else
      {
         model = 0;
      }
   }
   
   if(traceDebug())
   {
      ossimNotify(ossimNotifyLevel_DEBUG)
             << MODULE << " DEBUG: testing ossimPleiadesModel" << std::endl;
   }

   // Pleiades
   if ( !projection )
   {
      ossimRefPtr<ossimPleiadesModel> model = new ossimPleiadesModel();
      if ( model->open(filename) )
      {
         projection = model.get();
      }
      else
      {
         model = 0;
      }
   }

   if(traceDebug())
   	{
    	ossimNotify(ossimNotifyLevel_DEBUG)
        	   << MODULE << " DEBUG: testing ossimTerraSarModel" << std::endl;
    }

   if ( !projection )
   {
      ossimRefPtr<ossimTerraSarModel> model = new ossimTerraSarModel();

     if ( model->open(filename) )
      {
         // Check if a coarse grid was generated, and use it instead:
         projection = model->getReplacementOcgModel().get();
         if (projection.valid())
            model = 0; // Have OCG, don't need this one anymore
         else
            projection = model.get();
      }
      else
      {
         model = 0;
      }
   }

   if(traceDebug())
   {
    	ossimNotify(ossimNotifyLevel_DEBUG)
        	   << MODULE << " DEBUG: testing ossimErsSarModel" << std::endl;
   }

   if ( !projection )
   {
      ossimRefPtr<ossimErsSarModel> model = new ossimErsSarModel();
      if ( model->open(filename) )
      {
         projection = model.get();
      }
      else
      {
         model = 0;
      }
   }

   if(traceDebug())
   {
    	ossimNotify(ossimNotifyLevel_DEBUG)
        	   << MODULE << " DEBUG: testing ossimEnvisatSarModel" << std::endl;
   }

   if (!projection)
   {
     ossimRefPtr<ossimEnvisatAsarModel> model = new ossimEnvisatAsarModel();
     if (model->open(filename))
     {
       projection = model.get();
     }
     else
     {
       model = 0;
     }
   }

   if(traceDebug())
   {
    	ossimNotify(ossimNotifyLevel_DEBUG)
        	   << MODULE << " DEBUG: testing ossimRadarSatModel" << std::endl;
   }

   if (!projection)
   {
     ossimRefPtr<ossimRadarSatModel> model = new ossimRadarSatModel();
     if (model->open(filename))
     {
       projection = model.get();
     }
     else
     {
       model = 0;
     }
   }

   if(traceDebug())
   {
    	ossimNotify(ossimNotifyLevel_DEBUG)
        	   << MODULE << " DEBUG: testing ossimAlosPalsarModel" << std::endl;
   }

   if (!projection)
   {
     ossimRefPtr<ossimAlosPalsarModel> model = new ossimAlosPalsarModel();
     if (model->open(filename))
     {
       projection = model.get();
     }
     else
     {
       model = 0;
     }
   }

   if(traceDebug())
   	{
    	ossimNotify(ossimNotifyLevel_DEBUG)
        	   << MODULE << " DEBUG: testing ossimFormosatModel" << std::endl;
    }

   ossimFilename formosatTest = filename;
   formosatTest = formosatTest.setExtension("geom");
   if(!formosatTest.exists())
   {
      formosatTest = filename.path();
      formosatTest = formosatTest.dirCat(ossimFilename("METADATA.DIM"));
      if (formosatTest.exists() == false)
      {
         formosatTest = filename.path();
         formosatTest = formosatTest.dirCat(ossimFilename("metadata.dim"));
      }
   }
   if(formosatTest.exists())
   {
      ossimRefPtr<ossimFormosatDimapSupportData> meta =
         new ossimFormosatDimapSupportData;
      if(meta->loadXmlFile(formosatTest))
      {
   		 ossimRefPtr<ossimFormosatModel> model = new ossimFormosatModel(meta.get());
         if(!model->getErrorStatus())
         {
            projection = model.get();
         }
         model = 0;
      }
   }

   if(traceDebug())
   {
    	ossimNotify(ossimNotifyLevel_DEBUG)
        	   << MODULE << " DEBUG: testing ossimTileMapModel" << std::endl;
   }

   if (!projection)
   {
     ossimRefPtr<ossimTileMapModel> model = new ossimTileMapModel();
     if (model->open(filename))
     {
       projection = model.get();
     }
     else
     {
       model = 0;
     }
   }

   if (!projection)
   {
	   ossimFilename testFile = filename;
	   if (testFile.exists())
	   {
		   testFile = filename.path();
		   testFile = testFile.dirCat(ossimFilename("DESC.XML"));
		   if (testFile.exists() == false)
		   {
			   testFile = filename.path();
			   testFile = testFile.dirCat(ossimFilename("desc.xml"));
		   }
		   ossimRefPtr<ossimQVProcSupportData> meta =
			   new ossimQVProcSupportData;
		   if (meta->loadXmlFile(testFile))
		   {
			   if (meta->getSpacecraftID().upcase().contains("HJ"))
			   {
				   // HJ1

				   projection = new ossimHj1Model(meta.get());
				   if (!projection->getErrorStatus())
				   {
					   return projection.release();
				   }
				   projection = 0;

			   }
			   else if (meta->getSpacecraftID().upcase().contains("ZY 3"))
			   {
				   // ZY3
				   projection = new radiZY3Model(meta.get());
				   if (!projection->getErrorStatus())
				   {
					   return projection.release();
				   }
				   projection = 0;

			   }
			   else if (meta->getSpacecraftID().upcase().contains("CBERS04"))
			   {
				   // Cbers04
				   projection = new radiCbers04Model(meta.get());
				   if (!projection->getErrorStatus())
				   {
					   return projection.release();
				   }
				   projection = 0;

			   }
			   else
			   {
				   projection = 0;
			   }
		   }
	   }
   }
   
   if (!projection)
   {
	   ossimRefPtr<ossimRadarSat2RPCModel> radarSat2RPCModel = new ossimRadarSat2RPCModel();
	   if (radarSat2RPCModel->open(filename))
	   {
		   if (traceDebug())
		   {
			   ossimNotify(ossimNotifyLevel_DEBUG)
				   << MODULE << " DEBUG: returning ossimQuickbirdRpcModel"
				   << std::endl;
		   }
		   projection = radarSat2RPCModel.get();
		   radarSat2RPCModel = 0;
		   return projection.release();
	   }
	   else
	   {
		   radarSat2RPCModel = 0;
	   }
   }


   if (!projection)
   {
	   ossimFilename pleiadesTest = filename;
	   if (pleiadesTest.exists())
	   {
		   ossimRefPtr<ossimPleiadesModel> pPleiadesModel = new ossimPleiadesModel();
		   if (pPleiadesModel->open(pleiadesTest))
		   {
			   projection = pPleiadesModel.get();
			   pPleiadesModel = 0;
			   if (!projection->getErrorStatus())
			   {
				   return projection.release();
			   }
			   projection = 0;
		   }
	   }
   }


   if (!projection)
   {
	   // at last test the generic rpc model
	   ossimRefPtr<ossimplugins::radiRpcModel> rpcModel = new radiRpcModel;
	   if (rpcModel->parseRpcFile(filename))
	   {
		   if (traceDebug())
		   {
			   ossimNotify(ossimNotifyLevel_DEBUG)
				   << MODULE << " DEBUG: returning radiRpcModel"
				   << std::endl;
		   }
		   projection = rpcModel.get();
		   rpcModel = 0;
		   return projection.release();
	   }
	   else
	   {
		   rpcModel = 0;
	   }
   }

   return projection.release();
}

ossimProjection* ossimPluginProjectionFactory::createProjection(
   const ossimString& name)const
{
   static const char MODULE[] = "ossimPluginProjectionFactory::createProjection(ossimString& name)";

   if(traceDebug())
   {
    	ossimNotify(ossimNotifyLevel_DEBUG)
        	   << MODULE << " DEBUG: Entering ...." << std::endl;
   }

   //   else if (name == STATIC_TYPE_NAME(ossimCosmoSkymedModel))
   //    {
   //      return new ossimCosmoSkymedModel;
   //   }
   if (name == STATIC_TYPE_NAME(ossimRadarSat2Model))
   {
      return new ossimRadarSat2Model();
   }
   else if (name == STATIC_TYPE_NAME(ossimTerraSarModel))
   {
      return new ossimTerraSarModel();
   }
   else if (name == STATIC_TYPE_NAME(ossimErsSarModel))
   {
     return new ossimErsSarModel;
   }
   else if (name == STATIC_TYPE_NAME(ossimEnvisatAsarModel))
   {
     return new ossimEnvisatAsarModel;
   }
   else if (name == STATIC_TYPE_NAME(ossimRadarSatModel))
   {
     return new ossimRadarSatModel;
   }
   else if (name == STATIC_TYPE_NAME(ossimAlosPalsarModel))
   {
     return new ossimAlosPalsarModel;
   }
   else if (name == STATIC_TYPE_NAME(ossimFormosatModel))
   {
     return new ossimFormosatModel;
   }
   else if (name == STATIC_TYPE_NAME(ossimTileMapModel))
   {
     return new ossimTileMapModel;
   }
   else if (name == STATIC_TYPE_NAME(ossimPleiadesModel))
   {
     return new ossimPleiadesModel;
   }
   else if (name == STATIC_TYPE_NAME(ossimHj1Model))
   {
	   return new ossimHj1Model;
   }
   else if (name == STATIC_TYPE_NAME(radiZY3Model))
   {
	   return new radiZY3Model;
   }
   else if (name == STATIC_TYPE_NAME(radiCbers04Model))
   {
	   return new radiCbers04Model;
   }
   else if (name == STATIC_TYPE_NAME(radiRpcModel))
   {
	   return new radiRpcModel;
   }
   else if (name == STATIC_TYPE_NAME(ossimRadarSat2RPCModel))
   {
	   return new ossimRadarSat2RPCModel;
   }

   if(traceDebug())
   {
    	ossimNotify(ossimNotifyLevel_DEBUG)
        	   << MODULE << " DEBUG: Leaving ...." << std::endl;
   }

   return 0;
}

ossimProjection* ossimPluginProjectionFactory::createProjection(
   const ossimKeywordlist& kwl, const char* prefix)const
{
   ossimRefPtr<ossimProjection> result = 0;
   static const char MODULE[] = "ossimPluginProjectionFactory::createProjection(ossimKeywordlist& kwl)";

   if(traceDebug())
   {
    	ossimNotify(ossimNotifyLevel_DEBUG)
        	   << MODULE << " DEBUG: Start ...." << std::endl;
   }

   const char* lookup = kwl.find(prefix, ossimKeywordNames::TYPE_KW);
   if (lookup)
   {
      ossimString type = lookup;

      if (type == "ossimRadarSat2Model")
      {
         result = new ossimRadarSat2Model();
         if ( !result->loadState(kwl, prefix) )
         {
            result = 0;
         }
      }
      else if (type == "ossimTerraSarModel")
      {
         result = new ossimTerraSarModel();
         if ( !result->loadState(kwl, prefix) )
         {
            result = 0;
         }
      }
      else if (type == "ossimErsSarModel")
      {
         result = new ossimErsSarModel();
         if ( !result->loadState(kwl, prefix) )
         {
            result = 0;
         }
      }
      else if (type == "ossimEnvisatAsarModel")
      {
         result = new ossimEnvisatAsarModel();
         if ( !result->loadState(kwl, prefix) )
         {
            result = 0;
         }
      }
      else if (type == "ossimRadarSatModel")
      {
         result = new ossimRadarSatModel();
         if ( !result->loadState(kwl, prefix) )
         {
            result = 0;
         }
      }
      else if (type == "ossimAlosPalsarModel")
      {
         result = new ossimAlosPalsarModel();
         if ( !result->loadState(kwl, prefix) )
         {
            result = 0;
         }
      }
      else if (type == "ossimFormosatModel")
      {
         result = new ossimFormosatModel();
         if ( !result->loadState(kwl, prefix) )
         {
            result = 0;
         }
      }
      else if (type == "ossimTileMapModel")
      {
         result = new ossimTileMapModel();
         if ( !result->loadState(kwl, prefix) )
         {
            result = 0;
         }
      }
      else if (type == "ossimPleiadesModel")
      {
         result = new ossimPleiadesModel();
         if ( !result->loadState(kwl, prefix) )
         {
            result = 0;
         }
	  }
	  else if (type == "ossimHj1Model")
	  {
		  result = new ossimHj1Model();
		  if (!result->loadState(kwl, prefix))
		  {
			  result = 0;
		  }
	  }
	  else if (type == "radiZY3Model")
	  {
		  result = new radiZY3Model();
		  if (!result->loadState(kwl, prefix))
		  {
			  result = 0;
		  }
	  }
	  else if (type == "radiCbers04Model")
	  {
		  result = new radiCbers04Model();
		  if (!result->loadState(kwl, prefix))
		  {
			  result = 0;
		  }
	  }
	  else if (type == "radiRpcModel")
	  {
		  result = new radiRpcModel();
		  if (!result->loadState(kwl, prefix))
		  {
			  result = 0;
		  }
	  }
	  else if (type == "ossimRadarSat2RPCModel")
	  {
		  result = new ossimRadarSat2RPCModel();
		  if (!result->loadState(kwl, prefix))
		  {
			  result = 0;
		  }
	  }

   }

   if(traceDebug())
   {
    	ossimNotify(ossimNotifyLevel_DEBUG)
        	   << MODULE << " DEBUG: End ...." << std::endl;
   }
   
   return result.release();
}

ossimObject* ossimPluginProjectionFactory::createObject(
   const ossimString& typeName)const
{
   return createProjection(typeName);
}

ossimObject* ossimPluginProjectionFactory::createObject(
   const ossimKeywordlist& kwl, const char* prefix)const
{
   return createProjection(kwl, prefix);
}


void ossimPluginProjectionFactory::getTypeNameList(std::vector<ossimString>& typeList)const
{
   typeList.push_back(STATIC_TYPE_NAME(ossimRadarSatModel));
   typeList.push_back(STATIC_TYPE_NAME(ossimRadarSat2Model));
   typeList.push_back(STATIC_TYPE_NAME(ossimTerraSarModel));
   //   result.push_back(STATIC_TYPE_NAME(ossimCosmoSkymedModel));
   typeList.push_back(STATIC_TYPE_NAME(ossimEnvisatAsarModel));
   typeList.push_back(STATIC_TYPE_NAME(ossimErsSarModel));
   typeList.push_back(STATIC_TYPE_NAME(ossimAlosPalsarModel));
   typeList.push_back(STATIC_TYPE_NAME(ossimFormosatModel));
   typeList.push_back(STATIC_TYPE_NAME(ossimTileMapModel));
   typeList.push_back(STATIC_TYPE_NAME(ossimPleiadesModel));
   typeList.push_back(STATIC_TYPE_NAME(ossimHj1Model));
   typeList.push_back(STATIC_TYPE_NAME(radiZY3Model));
   typeList.push_back(STATIC_TYPE_NAME(radiCbers04Model));
   typeList.push_back(STATIC_TYPE_NAME(radiRpcModel));
   typeList.push_back(STATIC_TYPE_NAME(ossimRadarSat2RPCModel));
}

bool ossimPluginProjectionFactory::isTileMap(const ossimFilename& filename)const
{
  ossimFilename temp(filename);
  temp.downcase();
  
  ossimString os = temp.beforePos(4);
  
  if(temp.ext()=="otb")
  {
    return true;
  }
  else if(os == "http")
  {
    return true;
  }
  return false;
}


}
