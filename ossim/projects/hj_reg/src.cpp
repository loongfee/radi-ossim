#include <iostream>
#include <iterator>
#include <fstream>
//#include <ossim/matrix/newmatrc.h>
#include "ossim/base/ossimFilename.h"
#include "ossim/base/ossimString.h"
#include "ossim/imaging/ossimImageHandlerRegistry.h"
#include "ossim/imaging/ossimImageHandler.h"
#include "ossim/imaging/ossimImageFileWriter.h"
#include "ossim/imaging/ossimImageWriterFactoryRegistry.h"
#include <ossim/base/ossimXmlNode.h>
#include <ossim/base/ossimDirectory.h>
#include <ossim/base/ossimDirectoryTree.h>
#include <ossim/base/ossimXmlDocument.h>

#include "ossim/base/ossimGpt.h"
#include "ossim/base/ossimDpt.h"
#include "ossim/projection/ossimProjection.h"
#include "ossim/projection/ossimUtmProjection.h"
#include "ossim/projection/ossimTransMercatorProjection.h"
#include "ossim/projection/ossimProjectionFactoryRegistry.h"
#include "ossim/imaging/ossimImageRenderer.h"
#include "ossim/init/ossimInit.h"

#include "ossim/projection/ossimIkonosRpcModel.h"
#include "ossim/projection/ossimquickbirdrpcmodel.h"
#include "ossim/projection/ossimLandSatModel.h"
#include "ossim/support_data/ossimFfL5.h"




#include <ossim/base/ossimTieGptSet.h> 
#include "ossim/base/ossimobjectfactoryregistry.h"
#include <ossim/projection/ossimRpcProjection.h> 

#include <ossim/parallel/ossimIgen.h>
#include <ossim/parallel/ossimOrthoIgen.h>
#include "ossim/base/ossimAdjustmentInfo.h"

#include "ossim/base/ossimIrect.h"
#include "ossim/base/ossimStdOutProgress.h"
#include "ossim/imaging/ossimImageData.h"
#include "ossim/projection/ossimUtmProjection.h"

/////////////////////////////////////////////////////////////////////////////////
#include "ossim/base/ossimKeywordNames.h"
#include <ossim/imaging/ossimBandSelector.h>
#include <ossim/imaging/ossimCacheTileSource.h>
#include <ossim/matrix/newmat.h>
#include <ossim/matrix/newmatrc.h>
#include <ossim/imaging/ossimFilterResampler.h>
#include <ossim/base/ossimGeoidManager.h>
#include <ossim/elevation/ossimElevManager.h>
#include <ossim/base/ossimPreferences.h>
#include <iostream>
#include <direct.h>

#include <io.h>

#include "gdal_priv.h"
#include "gdal.h"
#include "ogr_srs_api.h"
#include "cpl_string.h"
#include "cpl_conv.h"
#include "cpl_multiproc.h"
#include "ogrsf_frmts.h"
#include <math.h>

//#include "ossim/projection/ossimTransMercatorProjection.h"
using namespace std;
using namespace NEWMAT;


	typedef struct point
	{
		int indext;//序号
		double x;//待纠正坐标X
		double y;//待纠正坐标Y
		double x2;//标准坐标X
		double y2;//标准坐标Y
		double z;//DEM高程

	}pointinfo;
	struct r_cluster
	{
		int indext;
		double r;
	};
bool Robe_DistributeOptimize(ossimProjection* proj,ossimTieGptSet* srcGptSet, ossimTieGptSet* &ctrlGptSet, ossimTieGptSet* &chkGtpSet,int nControl, int nCheck);
bool Rww_DistributeOptimize(ossimProjection* proj,ossimTieGptSet* srcGptSet, ossimTieGptSet* &ctrlGptSet, ossimTieGptSet* &chkGtpSet,int nControl, int nCheck);


bool DistributeOptimize(ossimTieGptSet* srcGptSet, ossimTieGptSet* &ctrlGptSet, ossimTieGptSet* &chkGtpSet,int nControl, int nCheck);
std::vector< ossimFilename >  	 fileList;
void Search_Directory(char* szFilename)
{
long handle;
//char path_search[_MAX_PATH];
struct _finddata_t filestruct;
char path_search[_MAX_PATH];
ossimFilename filenamelist,tmp;

handle = _findfirst("*",&filestruct);
if((handle == -1)) return;
tmp=filestruct.name;
//if( ::GetFileAttributes(filestruct.name) & FILE_ATTRIBUTE_DIRECTORY )
if( tmp.isDir() )
{
if( filestruct.name[0] != '.' )
{
_chdir(filestruct.name);

Search_Directory(szFilename);

_chdir("..");

}

}

else
{filenamelist=filestruct.name;
	if(!filenamelist.match(szFilename).empty() ) {
	ossimString pp;
_getcwd(path_search,_MAX_PATH);
pp=path_search;
	filenamelist=pp+"\\"+filenamelist;
	fileList.push_back(filenamelist);}
}

while(!(_findnext(handle,&filestruct)))

{
tmp=filestruct.name;

	//if( ::GetFileAttributes(filestruct.name) & FILE_ATTRIBUTE_DIRECTORY )
		if( tmp.isDir() )
	{
		if(*filestruct.name != '.')
		{_chdir(filestruct.name);
		Search_Directory(szFilename);
		_chdir("..");
		}

}

else

{
	filenamelist=filestruct.name;

if(!filenamelist.match(szFilename).empty() ) {
	ossimString pp;
_getcwd(path_search,_MAX_PATH);
	 pp=path_search;
	filenamelist=pp+"\\"+filenamelist;
	fileList.push_back(filenamelist);}

}

}

_findclose(handle);
}  
void writegcpTofile(ossimTieGptSet* m_gptSet,ossimProjection* proj,ossimFilename outFile)
{
		ofstream out;
		out.open(outFile.c_str());
		out.setf(ios::fixed, ios::floatfield);
		out.precision(6);

			for(int i=0;i < static_cast<int>(m_gptSet->size());i++)
			{
				ossimDpt dpt = proj->forward(*m_gptSet->getTiePoints()[i]);
				//cout<<m_gptSet->refTiePoints()[i]->GcpNumberID
				//	<<"\t"<<m_gptSet->refTiePoints()[i]->getImagePoint().x
				//	<<"\t"<<m_gptSet->refTiePoints()[i]->getImagePoint().y
				//	<<"\t"<<m_gptSet->refTiePoints()[i]->getGroundPoint().lat
				//	<<"\t"<<m_gptSet->refTiePoints()[i]->getGroundPoint().lon
				//	<<"\t"<<m_gptSet->refTiePoints()[i]->getGroundPoint().hgt<<"\n";

				out<<m_gptSet->refTiePoints()[i]->GcpNumberID
					<<"\t"<<m_gptSet->refTiePoints()[i]->getImagePoint().x
					<<"\t"<<m_gptSet->refTiePoints()[i]->getImagePoint().y
					<<"\t"<<dpt.x
					<<"\t"<<dpt.y
					<<"\t"<<m_gptSet->refTiePoints()[i]->getGroundPoint().hgt<<"\n";
			}
			out.close();


}


      vector<ossimDpt>  goundList;
     vector<ossimDpt>  imageList;
	ossimFilename gcpfilename,resultgcpfilename,checkfilename;
	//ossimTieGptSet * gptset;
	ossimTieGptSet* theTieSet;
	ossimTieGptSet* theTieSet5;
bool readgcpfromtxt(ossimFilename gcpfilename,ossimTieGptSet* gptset,ossimTransMercatorProjection* transMerc);
  int   my_cmp2(ossimRefPtr<ossimTieGpt>   i1, ossimRefPtr<ossimTieGpt>  i2);  

     				 const char*  WRS_PATH_NUMBER_KW     = "theWrsPathNumber";
				  const char*  ROW_NUMBER_KW          = "theWrsRowNumber";
				  const char*  ACQUISITION_DATE_KW          = "theAcquisitionDate";
				   const char*  CENTER_LONGTIDUDE_KW	="theCenterLongtidude";

int main(int argc, char* argv[])
{
	 
	setlocale(LC_ALL,"Chinese-simplified");
	ossimInit::instance()->initialize();
	OGRRegisterAll();
	GDALAllRegister();
	CPLSetConfigOption("GDAL_FILENAME_IS_UTF8","NO");

  bool result;

   std::vector<ossimFilename> gcpfileList;
   std::vector<ossimFilename> allmatchfile;
   ossimFilename master,slave,file_tmp,outimgfile;

    ossimString str_tmp1,str_tmp2,str_tmp3,str_tmp4;
 ossimString strpath,strrow,strdate,strcenterll,strcenterllTif,orderstr;
 ossimFilename stdfilename;
 ossimFilename gcpfilename,gcpOrthfilename;
  ossimFilename outfile;
 ossimString str,str1,str2;
 double img_ll,tif_ll;
   ossimKeywordlist tt_geom,geomp,des;

 std::vector<ossimString>   strsplit;
 bool flag=false;

	const char *lookup;
	 
   //ossimFilename srcpath("S:\\PROCESS\\时间序列数据\\黄河入海口时间序列\\黄河入海口");//argv[1]
   //ossimFilename outpath("S:\\PROCESS\\时间序列数据\\黄河入海口时间序列\\orthimg\\7bands");//argv[1]
   //ossimFilename gcppath("S:\\PROCESS\\时间序列数据\\黄河入海口时间序列\\gcpfilenew");


   	 
   ossimFilename srcpath("S:\\PROCESS\\时间序列数据\\74");//argv[1]
   ossimFilename outpath("S:\\PROCESS\\时间序列数据\\长江三角洲时间序列\\orthimg74");//argv[1]
   ossimFilename gcppath("S:\\PROCESS\\时间序列数据\\长江三角洲时间序列\\gcpfilenew74");

   //ossimPreferences::instance()->addPreference("OSSIM_NULL_PIX_UINT8","0");

	//const char* nullPixKw = ossimPreferences::instance()->findPreference("OSSIM_NULL_PIX_UINT8");
	//
	//		 if(nullPixKw )
	//	 {
	//			 std::vector<ossimString> splitArray;
	//			 ossimString tempString(nullPixKw );
	//			 tempString.split(splitArray, " ");
	//			 bool hasX = true;
	//			 if(splitArray.size() == 1)
	//			 {
	//				 return splitArray[0].toDouble();
	//			 }
	//			 else
	//			 {
	//				 return OSSIM_DEFAULT_NULL_PIX_UINT8;
	//			 }
	//		 }





   ossimDirectory dirsrc(gcppath);
  dirsrc.findAllFilesThatMatch(gcpfileList,ossimString(".xml"));//取得标准影像

  allmatchfile.clear();
 char* sx="header.dat";
  _chdir(srcpath.c_str());
  fileList.clear();
Search_Directory(sx);

for(int h=0;h<fileList.size();h++)
{
	allmatchfile.push_back(fileList[h]);
}

	
 	double m_centerlong;
	ossimDpt resolution(30.0,30.0);

					bool noerror=false;

			ossimTieGptSet* gptsetOptimize;
			ossimTieGptSet* gptsetTestOptimize;
			ossimTieGptSet* gcpOptimize;
			ossimTieGptSet* gcpTestOptimize;
			ossimTieGpt* aTiePt;
			int i=0;
			ossimDpt tobeclearpoint,imagepoint,geoxy;
			ossimGpt goundpoint,tGpt;
			ossimDpt residue1,residue2;
		vector<ossimRefPtr<ossimTieGpt> >&       theTGcp = gptsetOptimize->refTiePoints();
	
		vector<ossimRefPtr<ossimTieGpt> >&    theTest = gptsetTestOptimize->refTiePoints();
		vector<ossimRefPtr<ossimTieGpt> >&     theTallgcp = theTieSet->refTiePoints();
		vector<ossimRefPtr<ossimTieGpt> >::iterator tit;
		vector<ossimRefPtr<ossimTieGpt> >::iterator titi;
		ossim_float64 hgtAboveMsl;

					gptsetOptimize = new ossimTieGptSet;
					gptsetTestOptimize = new ossimTieGptSet;
					//allmatchfile.size()
	for(int h=0;h<allmatchfile.size();h++) {  

				ossimLandSatModel* model = new ossimLandSatModel(allmatchfile[h]);
				des.clear();
				model->saveState(des);
				

		 		lookup=NULL;strcenterll="";
				lookup=des.find("",CENTER_LONGTIDUDE_KW);
				if(lookup) {
					strcenterll=lookup;strcenterll.trim();
					img_ll=strcenterll.toDouble();
					}
				if(strcenterll.empty() )   continue;
				lookup=NULL;strdate="";
				lookup=des.find("",ACQUISITION_DATE_KW);
				
				if(lookup) {
					strdate=lookup;strdate.trim();
					strsplit.clear();
					strdate.split(strsplit,"-");
					if(strsplit.size()>1) strdate=strsplit[0]+strsplit[1]+strsplit[2];
					strsplit.clear();
					}

				strpath="";strrow="";
		 		lookup=NULL;
				lookup=des.find("",WRS_PATH_NUMBER_KW);
				if(lookup) {
					strpath=lookup;strpath.trim();
					}
				lookup=NULL;
				lookup=des.find("",ROW_NUMBER_KW);
				if(lookup) {
				
					strrow=lookup;strrow.trim();
						if(strrow.length()==1) strrow="00"+strrow;
						if(strrow.length()==2) strrow="0"+strrow;
				}
				if(strrow.empty() || strpath.empty())  continue;

				ossimGpt long_center(0,img_ll);
					int theMapZone =(img_ll+3)/6+30;

				outfile=outpath+"\\"+strpath+"-"+strrow+"-"+strdate+".tif"; 
				if(outfile.exists()) continue;

					ossimUtmProjection* utm   = new ossimUtmProjection(theMapZone);
					utm->setHemisphere('N');
					utm->setMetersPerPixel(resolution);
					model->m_proj=  utm;

					ossimXmlDocument gmlDoc;

				 gcpOrthfilename=gcppath+"\\"+strpath+"-"+strrow+"-"+strdate+"-Optimize-gcp.xml"; 
	/*			  if (gcpOrthfilename.exists())  {


					  continue;

				  }
				  else
				  {
							cout<<allmatchfile[h]<<endl;
							continue;
				  
				  
				  }*/
				 if (gcpOrthfilename.exists())  {  /////////////有正射 文件 
				 
				 		 theTieSet = new ossimTieGptSet;
						gmlDoc.openFile(gcpfilename);
						std::vector< ossimRefPtr< ossimXmlNode > > tieSetList;
						gmlDoc.findNodes(ossimString("/") + ossimTieGptSet::TIEPTSET_TAG, tieSetList);

					 if (tieSetList.size() != 1)
						{
						 ossimNotify(ossimNotifyLevel_WARN) << 
						 "WARNING: ossimModelOptimizer::loadGMLTieSet need exactly one element of type "<<
							   ossimTieGptSet::TIEPTSET_TAG<<", found "<<tieSetList.size()<<"\n";
							 continue;
						 }
					 theTieSet->importFromGmlNode(tieSetList[0]);
					  tit= theTieSet->refTiePoints().begin();i=0;
					 while( tit != theTieSet->refTiePoints().end())
						{
			 
						 imagepoint=(*tit)->getImagePoint();
						tGpt=(*tit)->getGroundPoint();
						 tGpt.hgt=ossimElevManager::instance()->getHeightAboveMSL(tGpt);
						 (*tit)->setGroundPoint(tGpt);
						 (*tit)->GcpNumberID=ossimString::toString(i++);
						 tit++;
			
						 }
					  model->updateModel();
				 	model->optimizeFit(*theTieSet);
					 model->updateModel();
				 
					ossimImageHandler *handler   = ossimImageHandlerRegistry::instance()->open(allmatchfile[h]);
					if(!handler)
						{
							cout << "Unable to open input image: "<< endl;
							continue;
						}
			ossimImageSource* source = handler;
				source->getImageGeometry()->setProjection(model);

			ossimImageFileWriter* writercc = ossimImageWriterFactoryRegistry::instance()->createWriter(ossimString("tiff_strip"));
			ossimImageRenderer* renderercc = new ossimImageRenderer;
			renderercc->getResampler()->setFilterType(ossimFilterResampler::ossimFilterResampler_CUBIC);
	
			tt_geom.clear();
			writercc->saveState(tt_geom);
			tt_geom.add("pixel_type","area",true);
			writercc->loadState(tt_geom);

			renderercc->connectMyInputTo(source);
			renderercc->setView(utm);
			writercc->connectMyInputTo(0,renderercc);
			writercc->setFilename(outfile);
			ossimStdOutProgress progresscc(0,true);
			writercc->addListener(&progresscc);
			writercc->execute();
			theTieSet->clearTiePoints();
				 		continue;	 
				 
				 }  //endif 

					theTieSet = new ossimTieGptSet;
					  theTieSet5 = new ossimTieGptSet;
					  std::vector< ossimRefPtr< ossimXmlNode > > tieSetList;
				 	gcpfilename=gcppath+"\\"+strpath+"-"+strrow+"-"+strdate+"-4-gcp.xml"; 
				if(!gcpfilename.exists()) continue;
									 
			
					gmlDoc.openFile(gcpfilename);
				   
					 gmlDoc.findNodes(ossimString("/") + ossimTieGptSet::TIEPTSET_TAG, tieSetList);

					 if (tieSetList.size() != 1)
					{
						 ossimNotify(ossimNotifyLevel_WARN) << 
						 "WARNING: ossimModelOptimizer::loadGMLTieSet need exactly one element of type "<<
							   ossimTieGptSet::TIEPTSET_TAG<<", found "<<tieSetList.size()<<"\n";
							 continue;
						 }

				 theTieSet->importFromGmlNode(tieSetList[0]);tieSetList.clear();

				 
			gcpfilename=gcppath+"\\"+strpath+"-"+strrow+"-"+strdate+"-3-gcp.xml"; 
				if(!gcpfilename.exists()) continue;
					gmlDoc.openFile(gcpfilename);
				 gmlDoc.findNodes(ossimString("/") + ossimTieGptSet::TIEPTSET_TAG, tieSetList);

					 if (tieSetList.size() != 1)
					{
						 ossimNotify(ossimNotifyLevel_WARN) << 
						 "WARNING: ossimModelOptimizer::loadGMLTieSet need exactly one element of type "<<
							   ossimTieGptSet::TIEPTSET_TAG<<", found "<<tieSetList.size()<<"\n";
							 continue;
						 }
					 
				 //get the TieSet object
				 theTieSet5->importFromGmlNode(tieSetList[0]);
		for (tit = theTieSet5->refTiePoints().begin() ; tit != theTieSet5->refTiePoints().end() ; ++tit)
			{
			theTieSet->addTiePoint(*tit);
			}
	theTieSet5->clearTiePoints();

				 
				  int numPoint=30;
				  if(theTieSet->size()>1000) numPoint=30;
					 else 
						 numPoint=theTieSet->size()/3;

////////////////方法1////////////////////////////////
	//			 gptsetOptimize->clearTiePoints();
	//			 gptsetTestOptimize->clearTiePoints();
	//			DistributeOptimize(theTieSet, gptsetOptimize, gptsetTestOptimize,numPoint,numPoint);

	//			 
	//				  
	//			  ossimDpt endpt,dpttemp;
	//			 ossimDpt mpl,men,spl,sen;
	//			ossimGpt sgpt;
	//			 vector< ossimDpt >im;
	//			 vector< ossimDpt >gd;
	//			ossimLeastSquaresBilin* lsba;
	//		ossimLeastSquaresBilin* lsbb;
	//		lsba= new ossimLeastSquaresBilin;
	//		lsbb= new ossimLeastSquaresBilin;
	//		int iter,iter_max;
	//		iter_max=150;iter=0;
	//		flag=false;
 //while ( (!flag) && (iter < iter_max) ) //non linear optimization loop
 //  {

	//	 lsba->clear();lsbb->clear();
	//	
	//	for (tit = gptsetOptimize->refTiePoints().begin() ; tit != gptsetOptimize->refTiePoints().end() ; ++tit)
	//	{
	//		endpt=utm->forward((*tit)->getGroundPoint());
	//		dpttemp=(*tit)->getImagePoint(); 
	//		im.push_back(dpttemp);
	//		gd.push_back(endpt);
	// 
	//		lsba->addSample(endpt.x,endpt.y,dpttemp.x);
	//		lsbb->addSample(endpt.x,endpt.y,dpttemp.y);
	//  
	//	 } 
 //   lsba->solveLS();
	//lsbb->solveLS();
	//ossimString max_Optimize_point,min_Check_point;
	//double errorx,errory,errorp;
	//double errorLimtid=0;
	//double errorCheckLimtid=9999999999999999.0;
	//for (tit = gptsetOptimize->refTiePoints().begin() ; tit != gptsetOptimize->refTiePoints().end() ; ++tit)
	//	{
	//		endpt=utm->forward((*tit)->getGroundPoint());
	//		dpttemp=(*tit)->getImagePoint(); 
	//		errorx=lsba->lsFitValue(endpt.x,endpt.y)-dpttemp.x;
	//		errory=lsbb->lsFitValue(endpt.x,endpt.y)-dpttemp.y;
	//		errorp=sqrt(errorx*errorx+errory*errory);
	//		if(errorp> errorLimtid)  {errorLimtid=errorp;max_Optimize_point=(*tit)->GcpNumberID;}
	//	}

	//	for (tit = gptsetTestOptimize->refTiePoints().begin() ; tit != gptsetTestOptimize->refTiePoints().end() ; ++tit)
	//	{
	//		endpt=utm->forward((*tit)->getGroundPoint());
	//		dpttemp=(*tit)->getImagePoint(); 
	//		errorx=lsba->lsFitValue(endpt.x,endpt.y)-dpttemp.x;
	//		errory=lsbb->lsFitValue(endpt.x,endpt.y)-dpttemp.y;
	//		errorp=sqrt(errorx*errorx+errory*errory);
	//		if(errorp< errorCheckLimtid)  {errorCheckLimtid=errorp;min_Check_point=(*tit)->GcpNumberID;}
	//	}
	//	cout << errorCheckLimtid<<"<------------->"<< errorLimtid<<endl;
	//	if (errorCheckLimtid<errorLimtid) {
	//				for (tit = gptsetOptimize->refTiePoints().begin() ; tit != gptsetOptimize->refTiePoints().end() ; ++tit)
	//					{
	//						if ((*tit)->GcpNumberID!=max_Optimize_point) continue;
	//						for (titi = gptsetTestOptimize->refTiePoints().begin() ; titi != gptsetTestOptimize->refTiePoints().end() ; ++titi)
	//						{
	//							if( (*titi)->GcpNumberID==min_Check_point) { 
	//								ossimGpt swap1;
	//								ossimDpt swap2;
	//								double score;
	//								swap1=(*tit)->refGroundPoint();(*tit)->setGroundPoint((*titi)->refGroundPoint());(*titi)->setGroundPoint(swap1);
	//								swap2=(*tit)->refImagePoint();(*tit)->setImagePoint((*titi)->refImagePoint());(*titi)->setImagePoint(swap2);
	//								 (*tit)->GcpNumberID=min_Check_point; (*titi)->GcpNumberID=min_Check_point;
	//								 score=(*tit)->getScore(); (*tit)->setScore((*titi)->getScore());(*titi)->setScore(score);
	//								 break;
	//							}

	//						}
	//						break;
	//				}
	//				++iter;
	//				continue;
	//	}//if
	//	
	//	flag=true;
 //}

		///////方法2/////////////////////////	
		 i=0;
	 for (tit = theTieSet->refTiePoints().begin() ; tit != theTieSet->refTiePoints().end() ; ++tit)
			  {
				   if( (*tit)->getScore()<0.9) continue;
					 imagepoint=(*tit)->getImagePoint();
					 tGpt=(*tit)->getGroundPoint();
					 tGpt.hgt=0;//ossimElevManager::instance()->getHeightAboveMSL(tGpt);
					 (*tit)->setGroundPoint(tGpt);
					 (*tit)->GcpNumberID=ossimString::toString(i++);
					 theTieSet5->addTiePoint(*tit);
						
		   }
	 theTieSet->clearTiePoints();
		   //////////////////////////////////////////////////////////////////////////////////////
		   
		//	if(theTieSet->size()<numPoint) {cout<< "good point number is too small ---->" <<gcpfilename.c_str() <<endl;continue;}


		// gptsetOptimize->clearTiePoints();
		// gptsetTestOptimize->clearTiePoints();
		// DistributeOptimize(theTieSet, gptsetOptimize, gptsetTestOptimize,numPoint, 0);

		//for (tit = gptsetOptimize->refTiePoints().begin() ; tit != gptsetOptimize->refTiePoints().end() ; ++tit)
		//		{
		//			theTieSet->addTiePoint(*tit);
		//}


	 gptsetOptimize->clearTiePoints();
	 gptsetTestOptimize->clearTiePoints();
	//DistributeOptimize(theTieSet, gptsetOptimize, gptsetTestOptimize,numPoint, 0);
	//	ossimFilename gcpout;
	//gcpout="G:\\黄河入海口\\gcpnewfile\\"+strpath+"-"+strrow+"-"+strdate+"1.txt"; 
	//cout<<allmatchfile[h].c_str()<<endl;
	//writegcpTofile(gptsetOptimize,utm,gcpout);
	//	model->optimizeFit(*gptsetOptimize);
	// model->updateModel();



	double minlength;
	bool findmoreerror;
	// double *targetVariance;
	// NEWMAT::ColumnVector lastresidue(3*numPoint);
	//std::vector<int> errorPointList;
	//errorPointList.clear();

	ossimTieGptSet* theTieSetRetry;
	bool flagaddpoint;
	double  ki2_nobs;
		double cidaerror=0.0, maxerror=0.0;
		int ciGcpIn, gcpIn,gcpswap;

		theTieSetRetry = new ossimTieGptSet;
		theTieSetRetry->clearTiePoints();
		flagaddpoint=true;

	do {
		if(theTieSet5->size()<numPoint) break;

		///////////////////////begin///////////////////////////////
		//if(theTieSet5->size()<2*numPoint && flagaddpoint) {
		//	flagaddpoint=false;
		//	 for (tit = theTieSetRetry->refTiePoints().begin() ; tit != theTieSetRetry->refTiePoints().end() ; ++tit)
		//	  {
		//		 imagepoint=(*tit)->getImagePoint();
		//	     model->lineSampleToWorld(imagepoint,tGpt);
		//		 residue1=utm->forward(tGpt);
		//		residue2=utm->forward((*tit)->getGroundPoint());
		//		if((residue2-residue1).length() < sqrt(ki2_nobs) ) 
		//			{
		//				theTieSet5->addTiePoint(*tit);
		//			}
		//  	 }

		//}
		/////////////////////////////////end///////////////////////////////////////////////////////////////
		noerror=false;
		// numPoint=numPoint-4;
		//	Rww_DistributeOptimize(utm,theTieSet, gptsetOptimize, gptsetTestOptimize,numPoint, numPoint);
		numPoint=30;
		gptsetOptimize->clearTiePoints();
		gptsetTestOptimize->clearTiePoints();

		DistributeOptimize(theTieSet5, gptsetOptimize, gptsetTestOptimize,numPoint, 0);

		//model->robustoptimizeFit(*gptsetOptimize,targetVariance,1,lastresidue);
		 model->updateModel();
		ki2_nobs=model->optimizeFit(*gptsetOptimize);
		 model->updateModel();


		 maxerror=0.0;
		 for (tit = gptsetOptimize->refTiePoints().begin() ; tit != gptsetOptimize->refTiePoints().end() ; ++tit)
			  {
				 imagepoint=(*tit)->getImagePoint();
				 tGpt.hgt=(*tit)->hgt;
					model->lineSampleToWorld(imagepoint,tGpt);
				 residue1=utm->forward(tGpt);
				residue2=utm->forward((*tit)->getGroundPoint());
					 //////////////////////////排序，删除最大误差点///
				if((residue2-residue1).length() > maxerror ) 
					{
						cidaerror=maxerror;
						ciGcpIn=gcpIn;
						maxerror=(residue2-residue1).length();
						//tobeclearpoint=imagepoint;
						gcpIn=(*tit)->GcpNumberID.toInt();


						}
		 }
		 cout<<"maxerror="<<maxerror<<endl;
		cout<<" number of points ="<<theTieSet5->size()<<endl;
		  cout<<"ki2_nobs="<<sqrt(ki2_nobs)<<endl;
	//	 errorPointList.push_back(gcpIn);
	 // if(maxerror/30.0 > 2.0) {//如果大于2 个象素，则删除此电，重新结构优化和参数优化
	   if(maxerror>100) {
		for (tit = theTieSet5->refTiePoints().begin() ; tit != theTieSet5->refTiePoints().end() ; ++tit)
			 {
				 if(gcpIn ==  (*tit)->GcpNumberID.toInt()) // ||(ciGcpIn ==  (*tit)->GcpNumberID.toInt())) {
				 {
					 theTieSet5->refTiePoints().erase(tit);
					 theTieSetRetry->addTiePoint(*tit);
					 break;

					 }

		}

				for (tit = theTieSet5->refTiePoints().begin() ; tit != theTieSet5->refTiePoints().end() ; ++tit)
			 {
				 if(ciGcpIn ==  (*tit)->GcpNumberID.toInt()) {
				 {
					 theTieSet5->refTiePoints().erase(tit);
					 theTieSetRetry->addTiePoint(*tit);
					 break;

					 }

				 }
				}




		noerror=true;
	   }//if(maxerror>100) {
	 
}while(noerror);

		if (!noerror) {
				 // -- 8 -- export as XML/GML
			 ossimRefPtr<ossimXmlNode> set_node = gptsetOptimize->exportAsGmlNode();
			 ossimXmlDocument gmlDoc;
			 gmlDoc.initRoot(set_node); //need namespaces etc...
			 gcpfilename=gcppath+"\\"+strpath+"-"+strrow+"-"+strdate+"-Optimize-gcp.xml"; 

			 if (gcpfilename.exists()) continue;
			 gmlDoc.write(gcpfilename);
			}
		else
			{
					 gptsetOptimize->clearTiePoints();
					 gptsetTestOptimize->clearTiePoints();
					 theTieSetRetry->clearTiePoints();
					 theTieSet5->clearTiePoints();
					  theTieSet->clearTiePoints();

				cout<< "good point number is too small ---->" <<gcpfilename.c_str() <<endl;
				continue;

			}



//////////////////////////////////////////////////////////////////////////////
		 gptsetOptimize->clearTiePoints();
	 gptsetTestOptimize->clearTiePoints();
	 theTieSetRetry->clearTiePoints();
	 theTieSet5->clearTiePoints();
	  theTieSet->clearTiePoints();
	
		 ossimImageHandler *handler   = ossimImageHandlerRegistry::instance()->open(allmatchfile[h]);
			if(!handler)
				{
				cout << "Unable to open input image: "<< endl;
					continue;
				}
			ossimImageSource* source = handler;
			 source->getImageGeometry()->setProjection(model);

			ossimImageFileWriter* writercc = ossimImageWriterFactoryRegistry::instance()->createWriter(ossimString("tiff_strip"));
			ossimImageRenderer* renderercc = new ossimImageRenderer;
			 renderercc->getResampler()->setFilterType(ossimFilterResampler::ossimFilterResampler_CUBIC);
	
			tt_geom.clear();
		writercc->saveState(tt_geom);
		tt_geom.add("pixel_type","area",true);
		writercc->loadState(tt_geom);

		renderercc->connectMyInputTo(source);
		renderercc->setView(utm);
		writercc->connectMyInputTo(0,renderercc);
		if (outfile.exists()) continue;
		writercc->setFilename(outfile);
		ossimStdOutProgress progresscc(0,true);
		writercc->addListener(&progresscc);
		writercc->execute();
	
			}
			
			
			
			
			return 1;
}


bool Robe_DistributeOptimize(ossimProjection* proj,ossimTieGptSet* srcGptSet, ossimTieGptSet* &ctrlGptSet, ossimTieGptSet* &chkGtpSet,int nControl, int nCheck)
{

	ossimTieGptSet* tmppoint;
	tmppoint = new ossimTieGptSet;

			ossimDpt endpt,dpttemp;
			ossimDpt mpl,men,spl,sen;
			ossimGpt sgpt;
			vector< ossimDpt >im;
			vector< ossimDpt >gd;
			bool flag;
			vector<ossimRefPtr<ossimTieGpt> >::iterator titi,tit;

				ossimString max_Optimize_point,min_Check_point;
	double errorx,errory,errorp;
	double errorLimtid=0;
	double errorCheckLimtid=9999999999999999.0;


			ossimLeastSquaresBilin* lsba;
			ossimLeastSquaresBilin* lsbb;
			lsba= new ossimLeastSquaresBilin;
			lsbb= new ossimLeastSquaresBilin;
			int iter,iter_max;
			iter_max=150;iter=0;
			
			 lsba->clear();lsbb->clear();
			 im.clear();gd.clear();
	for (tit = srcGptSet->refTiePoints().begin() ; tit != srcGptSet->refTiePoints().end() ; ++tit)
		{
			endpt=proj->forward((*tit)->getGroundPoint());
			dpttemp=(*tit)->getImagePoint(); 
			im.push_back(dpttemp);
			gd.push_back(endpt);
	 
			lsba->addSample(endpt.x,endpt.y,dpttemp.x);
			lsbb->addSample(endpt.x,endpt.y,dpttemp.y);
	  
		 } 
    lsba->solveLS();
	lsbb->solveLS();

		for (tit = srcGptSet->refTiePoints().begin() ; tit != srcGptSet->refTiePoints().end() ; ++tit)
		{
			endpt=proj->forward((*tit)->getGroundPoint());
			dpttemp=(*tit)->getImagePoint(); 
			errorx=lsba->lsFitValue(endpt.x,endpt.y)-dpttemp.x;
			errory=lsbb->lsFitValue(endpt.x,endpt.y)-dpttemp.y;
			errorp=sqrt(errorx*errorx+errory*errory);
			if(errorp> errorLimtid)  {errorLimtid=errorp;max_Optimize_point=(*tit)->GcpNumberID;}
		}

		errorLimtid=errorLimtid/2;
		std::cout<<"errorLimtid="<<errorLimtid<<endl;

	for (tit = srcGptSet->refTiePoints().begin() ; tit != srcGptSet->refTiePoints().end() ; ++tit)
		{
			endpt=proj->forward((*tit)->getGroundPoint());
			dpttemp=(*tit)->getImagePoint(); 
			errorx=lsba->lsFitValue(endpt.x,endpt.y)-dpttemp.x;
			errory=lsbb->lsFitValue(endpt.x,endpt.y)-dpttemp.y;
			errorp=sqrt(errorx*errorx+errory*errory);
			if(errorp<errorLimtid)  {tmppoint->addTiePoint(*tit);}
		}

	if(tmppoint->size()<(nControl+nCheck))    {  tmppoint->clearTiePoints();return false;}
 while ( iter < iter_max ) //non linear optimization loop
   {

		 lsba->clear();lsbb->clear();
		 im.clear();gd.clear();
		DistributeOptimize(tmppoint, ctrlGptSet, chkGtpSet,nControl, nCheck);

		for (tit = ctrlGptSet->refTiePoints().begin() ; tit != ctrlGptSet->refTiePoints().end() ; ++tit)
		{
			endpt=proj->forward((*tit)->getGroundPoint());
			dpttemp=(*tit)->getImagePoint(); 
			im.push_back(dpttemp);
			gd.push_back(endpt);
	 
			lsba->addSample(endpt.x,endpt.y,dpttemp.x);
			lsbb->addSample(endpt.x,endpt.y,dpttemp.y);
	  
		 } 
		 lsba->solveLS();
		lsbb->solveLS();

		flag=false;
		for (tit = ctrlGptSet->refTiePoints().begin() ; tit != ctrlGptSet->refTiePoints().end() ; ++tit)
			{
			endpt=proj->forward((*tit)->getGroundPoint());
			dpttemp=(*tit)->getImagePoint(); 
			errorx=lsba->lsFitValue(endpt.x,endpt.y)-dpttemp.x;
			errory=lsbb->lsFitValue(endpt.x,endpt.y)-dpttemp.y;
			errorp=sqrt(errorx*errorx+errory*errory);
			if(errorp> errorLimtid)  {flag=true;break;}
		}

				std::cout<<"errorLimtid="<<errorLimtid<<endl;
			std::cout<<"errorp="<<errorp<<"iter="<<iter<<endl;
		if (!flag) break;
		iter++;

 }

 tmppoint->clearTiePoints();
 return true;
}

bool Rww_DistributeOptimize(ossimProjection* proj,ossimTieGptSet* srcGptSet, ossimTieGptSet* &ctrlGptSet, ossimTieGptSet* &chkGtpSet,int nControl, int nCheck)
{

	ossimTieGptSet* tmppoint;
	tmppoint = new ossimTieGptSet;

			ossimDpt endpt,dpttemp;
			ossimDpt mpl,men,spl,sen;
			ossimGpt sgpt;
			vector< ossimDpt >im;
			vector< ossimDpt >gd;
			bool flag;
			vector<ossimRefPtr<ossimTieGpt> >::iterator titi,tit;

				ossimString max_Optimize_point,min_Check_point;
	double errorx,errory,errorp;
	double errorLimtid=0;
	double errorCheckLimtid=9999999999999999.0;


			ossimLeastSquaresBilin* lsba;
			ossimLeastSquaresBilin* lsbb;
			lsba= new ossimLeastSquaresBilin;
			lsbb= new ossimLeastSquaresBilin;
			int iter,iter_max;
			iter_max=150;iter=0;
	
		 tmppoint->clearTiePoints();
		for (tit = srcGptSet->refTiePoints().begin() ; tit != srcGptSet->refTiePoints().end() ; ++tit)
		{
			tmppoint->addTiePoint(*tit);
			 }


	 iter=0;
	 while(iter < iter_max )
	 {
		 lsba->clear();lsbb->clear();
		 im.clear();gd.clear();

		 iter++;
	for (tit = tmppoint->refTiePoints().begin() ; tit != tmppoint->refTiePoints().end() ; ++tit)
		{
			endpt=proj->forward((*tit)->getGroundPoint());
			dpttemp=(*tit)->getImagePoint(); 
			im.push_back(dpttemp);
			gd.push_back(endpt);
	 
			lsba->addSample(endpt.x,endpt.y,dpttemp.x);
			lsbb->addSample(endpt.x,endpt.y,dpttemp.y);
	  
		 } 
    lsba->solveLS();
	lsbb->solveLS();
	errorLimtid=0.0;
		for (tit = tmppoint->refTiePoints().begin() ; tit != tmppoint->refTiePoints().end() ; ++tit)
		{
			endpt=proj->forward((*tit)->getGroundPoint());
			dpttemp=(*tit)->getImagePoint(); 
			errorx=lsba->lsFitValue(endpt.x,endpt.y)-dpttemp.x;
			errory=lsbb->lsFitValue(endpt.x,endpt.y)-dpttemp.y;
			errorp=sqrt(errorx*errorx+errory*errory);
			if(errorp> errorLimtid)  {errorLimtid=errorp;max_Optimize_point=(*tit)->GcpNumberID;}
		}
		if(tmppoint->size()<(nControl+nCheck))    break;
		if (errorLimtid<10) break;
		std::cout<<"errorLimtid="<<errorLimtid<<"      iter="<<iter<<endl;
		//std::cout<<"errorp="<<errorp<<"iter="<<iter<<endl;
		for (tit = tmppoint->refTiePoints().begin() ; tit != tmppoint->refTiePoints().end() ; ++tit)
		{
			if ((*tit)->GcpNumberID==max_Optimize_point) {tmppoint->refTiePoints().erase(tit);break;}
			//endpt=proj->forward((*tit)->getGroundPoint());
			//dpttemp=(*tit)->getImagePoint(); 
			//errorx=lsba->lsFitValue(endpt.x,endpt.y)-dpttemp.x;
			//errory=lsbb->lsFitValue(endpt.x,endpt.y)-dpttemp.y;
			//errorp=sqrt(errorx*errorx+errory*errory);
			//if(errorp<errorLimtid)  {tmppoint->addTiePoint(*tit);}
		}
	 }
	 DistributeOptimize(tmppoint, ctrlGptSet, chkGtpSet,nControl, nCheck);

	 tmppoint->clearTiePoints();

	//if(tmppoint->size()<(nControl+nCheck))    {  tmppoint->clearTiePoints();return false;}
 //while ( iter < iter_max ) //non linear optimization loop
 //  {

	//	 lsba->clear();lsbb->clear();
	//	 im.clear();gd.clear();
	//	DistributeOptimize(tmppoint, ctrlGptSet, chkGtpSet,nControl, nCheck);

	//	for (tit = ctrlGptSet->refTiePoints().begin() ; tit != ctrlGptSet->refTiePoints().end() ; ++tit)
	//	{
	//		endpt=proj->forward((*tit)->getGroundPoint());
	//		dpttemp=(*tit)->getImagePoint(); 
	//		im.push_back(dpttemp);
	//		gd.push_back(endpt);
	// 
	//		lsba->addSample(endpt.x,endpt.y,dpttemp.x);
	//		lsbb->addSample(endpt.x,endpt.y,dpttemp.y);
	//  
	//	 } 
	//	 lsba->solveLS();
	//	lsbb->solveLS();

	//	flag=false;
	//	for (tit = ctrlGptSet->refTiePoints().begin() ; tit != ctrlGptSet->refTiePoints().end() ; ++tit)
	//		{
	//		endpt=proj->forward((*tit)->getGroundPoint());
	//		dpttemp=(*tit)->getImagePoint(); 
	//		errorx=lsba->lsFitValue(endpt.x,endpt.y)-dpttemp.x;
	//		errory=lsbb->lsFitValue(endpt.x,endpt.y)-dpttemp.y;
	//		errorp=sqrt(errorx*errorx+errory*errory);
	//		if(errorp> errorLimtid)  {flag=true;break;}
	//	}

	//			std::cout<<"errorLimtid="<<errorLimtid<<endl;
	//		std::cout<<"errorp="<<errorp<<"iter="<<iter<<endl;
	//	if (!flag) break;
	//	iter++;

 //}

 //tmppoint->clearTiePoints();
 return true;
}
bool DistributeOptimize(ossimTieGptSet* srcGptSet, ossimTieGptSet* &ctrlGptSet, ossimTieGptSet* &chkGtpSet,int nControl, int nCheck)
{
	int line_count;
	line_count = srcGptSet->size();

	if (line_count == 0)
		return false;
	if ((nControl + nCheck) > line_count)
		return false;

	//ctrlGptSet = new ossimTieGptSet;
	//chkGtpSet = new ossimTieGptSet;



	/////////////////////////////////////////////

	vector<ossimRefPtr<ossimTieGpt> >& theTPV = srcGptSet->refTiePoints();
	//vector<ossimRefPtr<ossimTieGpt> >::const_iterator tit,titi;
	vector<ossimRefPtr<ossimTieGpt> >::iterator titi,tit;
	vector<ossimString> m_gcpid;
	for (tit = theTPV.begin() ; tit != theTPV.end() ; ++tit)
	{
		ctrlGptSet->addTiePoint(*tit);

	}
	int i=0;

	pointinfo* p = new pointinfo[line_count];
	pointinfo* p_cluster = new pointinfo[line_count];
	double* d = new double[line_count];

	//	theTPV= ctrlGptSet->refTiePoints();
	//m_ProjectS.m_chkGtpSet->clearTiePoints();
	//m_ProjectS.m_CtrlGptSet->clearTiePoints();

	for (tit = theTPV.begin() ; tit != theTPV.end() ; ++tit)
	{
		//	  p[i].indext=(*tit)->GcpNumberID.toInt();
		m_gcpid.push_back((*tit)->GcpNumberID);
		p[i].indext=i;
		p[i].x=(*tit)->getImagePoint().x;
		p[i].y=(*tit)->getImagePoint().y;
		p[i].x2=(*tit)->getGroundPoint().lat;
		p[i].y2=(*tit)->getGroundPoint().lon;
		p[i].z=(*tit)->hgt;
		i++;
	}

	ctrlGptSet->clearTiePoints();

	///////////////////////////////////////////
	//求出最长距离的两点做为初始聚类中心
	double r=0.0;

	srand((unsigned)time(NULL));
	int random = rand() % line_count+1;
	cout<<random<<endl;
	p_cluster[0]=p[random-1];
	//r=std::distance(p[0].x,p_cluster[0].x,p[0].y,p_cluster[0].y);
	r=sqrt((p[0].x-p_cluster[0].x)*(p[0].x-p_cluster[0].x)+(p[0].y-p_cluster[0].y)*(p[0].y-p_cluster[0].y));

	p_cluster[1]=p[0];
	for (int i=0;i<line_count;i++)
	{
		d[i]=sqrt((p[i].x-p_cluster[0].x)*(p[i].x-p_cluster[0].x)+(p[i].y-p_cluster[0].y)*(p[i].y-p_cluster[0].y));

		//d[i]=distance(p[i].x,p_cluster[0].x,p[i].y,p_cluster[0].y);
		if (d[i]>r)
		{
			r=d[i];
			p_cluster[1]=p[i];
		}

	}
	int count=2;//类中的个数
	double r_temp;
	int flag=1;
	int j=0;
	int p_res;//保留的控制点数

	//	int remainnum;//保留点个数
	//int remainChecknum;//保留点个数

	p_res = nControl;

	while (count<p_res)
	{  j=0;
	r_cluster* r_clus=new r_cluster[line_count-count];

	for(int i=0;i<line_count;i++)
	{   flag=1;
	for(int l=0;l<count;l++){
		if(p_cluster[l].indext==p[i].indext)
		{flag=0;break;}}
	if(flag==1)
	{
		r=sqrt((p[i].x-p_cluster[0].x)*(p[i].x-p_cluster[0].x)+(p[i].y-p_cluster[0].y)*(p[i].y-p_cluster[0].y));
		//r=distance(p[i].x,p_cluster[0].x,p[i].y,p_cluster[0].y);
		for(int k=1;k<count;k++)
		{

			// r_temp=distance(p[i].x,p_cluster[k].x,p[i].y,p_cluster[k].y);
			r_temp=sqrt((p[i].x-p_cluster[k].x)*(p[i].x-p_cluster[k].x)+(p[i].y-p_cluster[k].y)*(p[i].y-p_cluster[k].y));

			if(r_temp<r){
				r_clus[j].r=r_temp;
				r_clus[j].indext=i;
				r=r_temp;}
			else{
				r_clus[j].r=r;
				r_clus[j].indext=i;}
		}
		j++;
	}
	}

	for(int n=0;n<line_count-count-1;n++)
	{if(r_clus[n].r>r_clus[n+1].r)
	r_clus[n+1]=r_clus[n];}
	p_cluster[count]=p[r_clus[line_count-count-1].indext];
	count++;

	delete r_clus;
	}

	//输出优化结果/////////////////////////////////////////////////////
	//const char *filename=outputname.c_str();

	ossimTieGpt *aTiePt;
	for (int i=0;i<p_res;i++)
	{ 

		//o_file<<p_cluster[i].indext<<setprecision(10)<<" "<<p_cluster[i].x<<" "<<p_cluster[i].y<<" "<<p_cluster[i].x2<<" "<<p_cluster[i].y2<<" "<<p_cluster[i].z<<endl;

		ossimDpt p1(p_cluster[i].x,p_cluster[i].y);
		ossimGpt p2;


		p2.lat=p_cluster[i].x2;
		p2.lon=p_cluster[i].y2;
		p2.hgt=p_cluster[i].z;
		aTiePt=new ossimTieGpt(p2,p1,p2.hgt,m_gcpid[p_cluster[i].indext]);
		//aTiePt->GcpNumberID=ossimString::toString(p_cluster[i].indext);
		//aTiePt->GcpNumberID=m_gcpid[p_cluster[i].indext];

		ctrlGptSet->addTiePoint(aTiePt);

	}



	if(nCheck==0) {
			m_gcpid.clear();
		////////////////////////////////////////////////////////////////////
		delete d;
		delete p_cluster;
		
		delete p;
		return true;
	}


	//输出剩余点 ///////////////////////////////////////////////
	for(j=0;j<p_res;j++)
		for(int i=0;i<line_count;i++)
		{
			if (p_cluster[j].indext==p[i].indext)
			{
				for(int k=i;k<line_count-1;k++)
					p[k]=p[k+1];
			}
		}
		//	const char filename2[]="rest.txt";

		//	ofstream o_file2;
		//	o_file2.open(filename2);
		for (int i=0;i<line_count-p_res;i++)
		{
			ossimDpt p1(p[i].x,p[i].y);
			ossimGpt p2;


			p2.lat=p[i].x2;
			p2.lon=p[i].y2;
			p2.hgt=p[i].z;

			aTiePt=new ossimTieGpt(p2,p1,p2.hgt,m_gcpid[p[i].indext]);
			//aTiePt->GcpNumberID=ossimString::toString(p[i].indext);
			//m_ProjectS.m_chkGtpSet->addTiePoint(aTiePt);

			//aTiePt->GcpNumberID=m_gcpid[p[i].indext];
			chkGtpSet->addTiePoint(aTiePt);
			//o_file2<<p[i].indext<<setprecision(10)<<" "<<p[i].x<<" "<<p[i].y<<" "<<p[i].x2<<" "<<p[i].y2<<" "<<p[i].z<<endl;
		}
		//	o_file.close();
		///////////////////////////////////////////////////////////////

		delete d;
		delete p_cluster;
		delete p;
		m_gcpid.clear();
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


		line_count = chkGtpSet->size();
	//	const vector<ossimRefPtr<ossimTieGpt> >& theRTPV = chkGtpSet->getTiePoints();
		p=new pointinfo[line_count];
		i=0;

		for (tit = chkGtpSet->refTiePoints().begin() ; tit !=  chkGtpSet->refTiePoints().end() ; ++tit)
		{
			m_gcpid.push_back((*tit)->GcpNumberID);
			p[i].indext=i;
			p[i].x=(*tit)->getImagePoint().x;
			p[i].y=(*tit)->getImagePoint().y;
			p[i].x2=(*tit)->getGroundPoint().lat;
			p[i].y2=(*tit)->getGroundPoint().lon;
			p[i].z=(*tit)->hgt;
			i++;
		}

		chkGtpSet->clearTiePoints();
		///////////////////////////////////////////
		//求出最长距离的两点做为初始聚类中心
		r=0.0;
		d = new double[line_count];

		p_cluster=new pointinfo[line_count];
		srand((unsigned)time(NULL));
		random = rand() % line_count+1;

		p_cluster[0]=p[random-1];
		// r=distance(p[0].x,p_cluster[0].x,p[0].y,p_cluster[0].y);
		r=sqrt((p[0].x-p_cluster[0].x)*(p[0].x-p_cluster[0].x)+(p[0].y-p_cluster[0].y)*(p[0].y-p_cluster[0].y));
		p_cluster[1]=p[0];
		for (int i=0;i<line_count;i++)
		{
			d[i]=sqrt((p[i].x-p_cluster[0].x)*(p[i].x-p_cluster[0].x)+(p[i].y-p_cluster[0].y)*(p[i].y-p_cluster[0].y));
			//d[i]=distance(p[i].x,p_cluster[0].x,p[i].y,p_cluster[0].y);
			if (d[i]>r)
			{
				r=d[i];
				p_cluster[1]=p[i];
			}

		}
		count=2;//类中的个数
		r_temp;
		flag=1;
		j=0;
		p_res;//保留的控制点数

		//	int remainnum;//保留点个数
		//int remainChecknum;//保留点个数

		p_res = nCheck;

		while (count<p_res)
		{  j=0;
		r_cluster* r_clus=new r_cluster[line_count-count];

		for(int i=0;i<line_count;i++)
		{   flag=1;
		for(int l=0;l<count;l++){
			if(p_cluster[l].indext==p[i].indext)
			{flag=0;break;}}
		if(flag==1)
		{
			r=sqrt((p[i].x-p_cluster[0].x)*(p[i].x-p_cluster[0].x)+(p[i].y-p_cluster[0].y)*(p[i].y-p_cluster[0].y));
			// r=distance(p[i].x,p_cluster[0].x,p[i].y,p_cluster[0].y);
			for(int k=1;k<count;k++)
			{
				r_temp=sqrt((p[i].x-p_cluster[k].x)*(p[i].x-p_cluster[k].x)+(p[i].y-p_cluster[k].y)*(p[i].y-p_cluster[k].y));

				//  r_temp=distance(p[i].x,p_cluster[k].x,p[i].y,p_cluster[k].y);
				if(r_temp<r){
					r_clus[j].r=r_temp;
					r_clus[j].indext=i;
					r=r_temp;}
				else{
					r_clus[j].r=r;
					r_clus[j].indext=i;}
			}
			j++;
		}
		}

		for(int n=0;n<line_count-count-1;n++)
		{if(r_clus[n].r>r_clus[n+1].r)
		r_clus[n+1]=r_clus[n];}
		p_cluster[count]=p[r_clus[line_count-count-1].indext];               
		count++;
		}

		//输出优化结果/////////////////////////////////////////////////////
		//const char *filename=outputname.c_str();


		for (int i=0;i<p_res;i++)
		{
			ossimTieGpt tpp;
			//o_file<<p_cluster[i].indext<<setprecision(10)<<" "<<p_cluster[i].x<<" "<<p_cluster[i].y<<" "<<p_cluster[i].x2<<" "<<p_cluster[i].y2<<" "<<p_cluster[i].z<<endl;

			ossimDpt p1(p_cluster[i].x,p_cluster[i].y);
			ossimGpt p2;


			p2.lat=p_cluster[i].x2;
			p2.lon=p_cluster[i].y2;
			p2.hgt=p_cluster[i].z;

			aTiePt=new ossimTieGpt(p2,p1,p2.hgt,m_gcpid[p_cluster[i].indext]);
			//aTiePt->GcpNumberID=ossimString::toString(p_cluster[i].indext);
			//		m_ProjectS.m_chkGtpSet->addTiePoint(aTiePt);
			//aTiePt->GcpNumberID=m_gcpid[p_cluster[i].indext];
			chkGtpSet->addTiePoint(aTiePt);
		}
		m_gcpid.clear();
		////////////////////////////////////////////////////////////////////
		delete d;
		delete p_cluster;
		delete p;

	
		/////////////////////

		return true;
}
