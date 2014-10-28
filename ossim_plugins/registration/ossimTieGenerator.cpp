
#include "ossimTieGenerator.h"
#include <ossim/imaging/ossimImageGeometry.h>
#include <ossim/imaging/ossimImageSource.h>
#include <ossim/imaging/ossimImageHandler.h>
#include <ossim/base/ossimTrace.h>
#include <ossim/base/ossimNotifyContext.h>
#include <fstream>
#include <iostream> //TBR DEBUG
using namespace std;

static ossimTrace traceDebug("ossimTieGenerator:debug");

RTTI_DEF2(ossimTieGenerator, "ossimTieGenerator",
          ossimOutputSource, ossimProcessInterface);

ossimTieGenerator::ossimTieGenerator(ossimImageSource* inputSource)
      :
      ossimOutputSource(NULL, // owner
                        1,
                        0,
                        true,
                        true),
      ossimProcessInterface(),
      theAreaOfInterest(),
      theFilename(ossimFilename::NIL),
      theFileStream(),
      theStoreFlag(false)
{
   connectMyInputTo(0, inputSource);
   theAreaOfInterest.makeNan();
}

ossimTieGenerator::~ossimTieGenerator()
{
}

bool ossimTieGenerator::execute()
{
   bool status = true;
   static const char MODULE[] = "ossimTieGenerator::execute";

   if (traceDebug()) CLOG << " Entered..." << endl;
   
   ossimImageSource* src = reinterpret_cast<ossimImageSource*>(getInput(0));
   if (theAreaOfInterest.isNan())
   {
      // ChipMatch will provide correct ROI
      theAreaOfInterest = src->getBoundingRect(0);
   }

   //open stream
   open();

   setProcessStatus(ossimProcessInterface::PROCESS_STATUS_EXECUTING);
   
   //write out projection info
   if (theFilename != ossimFilename::NIL)
   {
      ossimRefPtr<ossimImageGeometry> geom = src->getImageGeometry();
      ossimKeywordlist mpk;
      if(geom.valid())
      {
         geom->saveState(mpk);
      }
      theFileStream<<mpk.toString(); //writeToStream(theFileStream);
   }

   //do the actual work there
   if (status) status = getAllFeatures();  
   
   setProcessStatus(ossimProcessInterface::PROCESS_STATUS_NOT_EXECUTING);

   if (traceDebug()) CLOG << " Exited..." << endl;

   //close stream
   close();

   return status;
}

struct row_col{
	double row_idx;
	double col_idx;
	row_col(double r, double c)
	{
		row_idx = r;
		col_idx = c;
	}
};

static int row_col_step = 7;
// multiple keywords
bool RowColCompare(row_col rc1, row_col rc2)
{
	return (fabs(rc1.row_idx)+fabs(rc1.col_idx)) < (fabs(rc2.row_idx)+fabs(rc2.col_idx));
	//int fd1 = fabs(rc1.row_idx)+fabs(rc1.col_idx);
	//int fd2 = fabs(rc2.row_idx)+fabs(rc2.col_idx);
	//int d1 = (int)(fd1 + 0.5);
	//int d2 = (int)(fd2 + 0.5);
	//int m1 = d1%row_col_step;
	//int m2 = d2 %row_col_step;
	//if (m1 == m2)
	//{
	//	return fd1 < fd2;
	//}
	//return m1 < m2;
}

bool ossimTieGenerator::getGridFeaturesParallel(const ossimIrect& rect)
{
	ossimChipMatch* src = reinterpret_cast<ossimChipMatch*>(getInput(0));
	if (!src)
	{
		ossimNotify(ossimNotifyLevel_WARN)
			<< "WARN ossimTieGenerator::scanForEdges():"
			<< "\nInput source is not a ossimImageChip.  Returning..." << std::endl;
		return false;
	}

	int theTileSize = 64;
	const ossim_int32 TILE_HEIGHT	= theTileSize;
	const ossim_int32 TILE_WIDTH	= theTileSize;
	const ossim_int32 START_LINE = rect.ul().y;
	const ossim_int32 STOP_LINE  = rect.lr().y;
	const ossim_int32 START_SAMP = rect.ul().x;
	const ossim_int32 STOP_SAMP  = rect.lr().x;
	int nWidth = STOP_SAMP-START_SAMP;
	int nHeight = STOP_LINE-START_LINE;

	// For percent complete status.
	ossim_int32 tilerows=(STOP_LINE-START_LINE+TILE_HEIGHT) / TILE_HEIGHT; //ceil : (stop-start+1+size-1)/size
	ossim_int32 tilecols=(STOP_SAMP-START_SAMP+TILE_WIDTH) / TILE_WIDTH;
	double total_tiles = ((double)tilerows)*tilecols;
	double tiles_processed = 0.0;

	// loop through all tiles
	// need to use a sequencer for parallelism in the future TBD
	ossim_int32 line=START_LINE;
	ossim_int32 i,j;

	vector<row_col> row_col_List;
	double center_row = tilerows * 0.5;
	double center_col = tilecols * 0.5;
	for (i=0;(i<tilerows);++i)
	{
		for (j=0;(j<tilecols);++j )
		{
			row_col_List.push_back(row_col((center_row-i-1),(center_col-j-1)));
		}
	}
	row_col_step = 5;
	std::sort(row_col_List.begin(), row_col_List.end(), RowColCompare);	

	bool bDebug = true;
	bool found = false;
	int N_PARALLEL = 1;//ncore*2;
	int search_size = (int)row_col_List.size();
	search_size = min(search_size, 10);
	for (int i=0;i < search_size && !found;)
		//for (int i=0;i < (int)row_col_List.size() && !found;)
	{
		int nParallel = min(N_PARALLEL, (int)row_col_List.size()-1-i);	// 保证末尾不越界

		std::vector<ossimIrect> srectList;
		std::vector<bool>  slaveValidList(nParallel, false);
		for (int j = 0; j < nParallel;j++)
		{
			int icol = floor(row_col_List[i+j].col_idx+center_col+0.5);
			int irow = floor(row_col_List[i+j].row_idx+center_row+0.5);
			//int ii = (5 * i) % (int)row_col_List.size();
			//int icol = floor(row_col_List[ii+j].col_idx+center_col+0.5);
			//int irow = floor(row_col_List[ii+j].row_idx+center_row+0.5);
			ossim_int32 samp=START_SAMP+icol*TILE_WIDTH;
			ossim_int32 line=START_LINE+irow*TILE_HEIGHT;


			ossim_int32 BufHeight = TILE_HEIGHT;
			ossim_int32 BufWidth = TILE_WIDTH;
			//行末尾小块处理
			if (irow == tilerows-1)
			{
				BufHeight = nHeight - (tilerows-1) * TILE_HEIGHT;//得出当前块的宽度Bufsizex，高度Bufsizey
				BufHeight = min(BufHeight, TILE_HEIGHT);
			}
			//列末尾小块处理
			if (icol == tilecols-1)
			{
				BufWidth = nWidth - (tilecols-1) * TILE_WIDTH;//得出当前块的宽度Bufsizex，高度Bufsizey
				BufWidth = min(BufWidth, TILE_WIDTH);
			}

			// slave
			ossimIrect srect(ossimIpt(samp, line),ossimIpt(samp+BufWidth-1,line+BufHeight-1));
			const vector<ossimTDpt>& tp = src->getFeatures(srect);

			if (tp.size() > 0)
			{
				vector<ossimTDpt> result;
				result.push_back(tp[0]);
				if (theFilename != ossimFilename::NIL)
				{
					//write on stream
					writeTiePoints(result);
				}

				if (getStoreFlag())
				{
					//store them : insert at the end (constant time) //TBD : conditional store
					theTiePoints.insert(theTiePoints.end(),result.begin(),result.end());
				}
				return true;
			}
		}
		i += N_PARALLEL;
	}
	return found;
}

bool ossimTieGenerator::getAllFeatures()
{
   static const char MODULE[] = "ossimTieGenerator::getAllFeatures";

   if (traceDebug()) CLOG << " Entered..." << endl;
   
   // Some constants needed throughout...
   int nWidth = theAreaOfInterest.width();
   int nHeight =theAreaOfInterest.height();
   int nPointRequired = 100;
   //const ossim_int32 TILE_HEIGHT    = src->getTileHeight();
   //const ossim_int32 TILE_WIDTH     = src->getTileWidth();
   const ossim_int32 START_LINE = theAreaOfInterest.ul().y;
   const ossim_int32 STOP_LINE  = theAreaOfInterest.lr().y;
   const ossim_int32 START_SAMP = theAreaOfInterest.ul().x;
   const ossim_int32 STOP_SAMP  = theAreaOfInterest.lr().x;

   // For percent complete status.
   //ossim_int32 tilerows=(STOP_LINE-START_LINE+TILE_HEIGHT) / TILE_HEIGHT; //ceil : (stop-start+1+size-1)/size
   //ossim_int32 tilecols=(STOP_SAMP-START_SAMP+TILE_WIDTH) / TILE_WIDTH;
   ossim_int32 tilerows = ceil(sqrt(nPointRequired * nHeight / (double)nWidth ));
   ossim_int32 tilecols = ceil(sqrt(nPointRequired * nWidth / (double)nHeight ));
   const ossim_int32 TILE_HEIGHT    = ceil(nHeight / (double)tilerows);
   const ossim_int32 TILE_WIDTH     = ceil(nWidth / (double)tilecols);
   double total_tiles = ((double)tilerows)*tilecols;
   double tiles_processed = 0.0;

   // Set the status message to be "scanning source for edges..."
   ossimNotify(ossimNotifyLevel_INFO) << "Getting tie points..." << std::endl;
   
   // Start off with a percent complete at 0...
   setPercentComplete(0.0);

   // loop through all tiles
   // need to use a sequencer for parallelism in the future TBD
   theTiePoints.clear();
   vector<row_col> row_col_List;
   for (int i=0;i<tilerows;++i)
   {
	   for (int j=0;j<tilecols;++j )
	   {
		   row_col_List.push_back(row_col(i,j));
	   }
   }

   int myid = 0;
   int numprocs = 1;
   for (int i = myid;i < (int)row_col_List.size(); i += numprocs)
   {
	   int irow = (int)row_col_List[i].row_idx;

	   ossim_int32 line=START_LINE+irow*TILE_HEIGHT;
	   ossim_int32 BufHeight = TILE_HEIGHT;
	   //列末尾小块处理
	   if (irow == tilerows-1)
	   {
		   BufHeight = nHeight - (tilerows-1) * TILE_HEIGHT;
		   BufHeight = min(BufHeight, TILE_HEIGHT);
	   }

	   int icol = (int)row_col_List[i].col_idx;
	   ossim_int32 samp=START_SAMP+icol*TILE_WIDTH;
	   ossim_int32 BufWidth = TILE_WIDTH;
	   //列末尾小块处理
	   if (icol == tilecols-1)
	   {
		   BufWidth = nWidth - (tilecols-1) * TILE_WIDTH;
		   BufWidth = min(BufWidth, TILE_WIDTH);
	   }
	   // Get the tie points
	   getGridFeaturesParallel(ossimIrect(ossimIpt(samp, line),ossimIpt(samp+BufWidth-1,line+BufHeight-1)));

	   // Set the percent complete.
	   tiles_processed += 1.0;
	   //MPI_Reduce(&tiles_processed, 
	   //	&total_tiles_processed, 
	   //	1, 
	   //	MPI_DOUBLE, 
	   //	MPI_SUM, 
	   //	0, 
	   //	MPI_COMM_WORLD);
	   if (myid == 0)
	   {
		   setPercentComplete((i+1)/total_tiles*100.0);
	   }
	   //if (i+numprocs > (int)row_col_List.size())
	   //{
	   //	MPI_Barrier(MPI_COMM_WORLD);
	   //}

	   fflush( stdout );
   }


//   ossim_int32 line=START_LINE;
//   ossim_int32 i,j;
//   for (i=0;(i<tilerows)&&!needsAborting();++i)
//   {
//      ossim_int32 samp=START_SAMP;
//      for (j=0;(j<tilecols)&&!needsAborting();++j )
//	  {
//		  // Get the tie points
//		  getGridFeaturesParallel(ossimIrect(ossimIpt(samp, line),ossimIpt(samp+BufWidth-1,line+BufHeight-1)));
//         // Get the tie points
//         //TBC : can we go further than source edges with edge tiles?? TBD
//         const vector<ossimTDpt>& tp = src->getFeatures(ossimIrect(ossimIpt(samp, line),ossimIpt(samp+TILE_WIDTH-1,line+TILE_HEIGHT-1)));
//
//         if (theFilename != ossimFilename::NIL)
//         {
//            //write on stream
//            writeTiePoints(tp);
//         }
//         
//         if (getStoreFlag())
//         {
//            //store them : insert at the end (constant time) //TBD : conditional store
//            theTiePoints.insert(theTiePoints.end(),tp.begin(),tp.end());
//         }
//
//         samp+=TILE_WIDTH;
//         // Set the percent complete.
//         tiles_processed += 1.0;
//         setPercentComplete(tiles_processed/total_tiles*100.0);
//
////DEBUG TBR TBC
//// std::cout<<"p="<<tiles_processed/total_tiles*100.0<<std::endl;
//
//      }
//      line+=TILE_HEIGHT;
//   }

   if (myid == 0)
   {
	   setPercentComplete(100.0);
   }
   if (traceDebug()) CLOG << " Exited." << endl;
   return true;
}

void ossimTieGenerator::writeTiePoints(const vector<ossimTDpt>& tp)
{
   for (vector<ossimTDpt>::const_iterator it=tp.begin();it!=tp.end();++it)
   {
      it->printTab(theFileStream);
      theFileStream<<endl;
   }
}

void ossimTieGenerator::setOutputName(const ossimString& filename)
{
   ossimOutputSource::setOutputName(filename);

   if (isOpen()) close();
   
   if (filename != "")
   {
      theFilename = filename;
   }
}

void ossimTieGenerator::setAreaOfInterest(const ossimIrect& rect)
{
   theAreaOfInterest = rect;
}

bool ossimTieGenerator::isOpen()const
{
   return const_cast<ofstream*>(&theFileStream)->is_open();
}

bool ossimTieGenerator::open()
{
   if(isOpen())
   {
      close();
   }

   if (theFilename == ossimFilename::NIL)
   {
      return false;
   }
   
   theFileStream.open(theFilename.c_str());

   return theFileStream.good();
}

void ossimTieGenerator::close()
{
   if (isOpen()) theFileStream.close();
}
