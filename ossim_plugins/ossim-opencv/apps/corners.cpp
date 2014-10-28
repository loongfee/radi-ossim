/*!
 *
 * OVERVIEW: Application to test rspfOpenCVGoodFeaturesToTrack 
 *
 */

#include <iostream>
#include <iterator>

#include <rspf/base/rspfFilename.h>
#include <rspf/base/rspfString.h>
#include <rspf/base/rspfStdOutProgress.h>

#include <rspf/imaging/rspfImageHandlerRegistry.h>
#include <rspf/imaging/rspfImageHandler.h>
#include <rspf/imaging/rspfImageFileWriter.h>
#include <rspf/imaging/rspfImageWriterFactoryRegistry.h>
#include <rspf/imaging/rspfImageSourceFactoryRegistry.h>
#include <rspf/imaging/rspfTiffWriter.h>

#include "../plugin/rspfOpenCVGoodFeaturesToTrack.h"

#include <rspf/init/rspfInit.h>


using namespace std;
void usage();
void printOutputTypes();

int main(int argc, char* argv[])
{

if(argc<3) {
	usage();
	return 0;
}

rspfKeywordlist kwl; 

rspfInit::instance()->initialize(argc, argv);

rspfImageHandler *handler = rspfImageHandlerRegistry::instance()->open(rspfFilename(argv[2]));

if(!handler)
       {
         cout << "Unable to open input image: "<< argv[2] << endl;
         return 1;
       }

rspfOpenCVGoodFeaturesToTrack * filter = new rspfOpenCVGoodFeaturesToTrack();  
if(kwl.addFile(argv[4])){
    filter->loadState(kwl);
}
filter->connectMyInputTo(0,handler);

rspfImageFileWriter* writer = rspfImageWriterFactoryRegistry::instance()->createWriter(rspfString(argv[1]));
writer->setFilename(rspfFilename(argv[3]));
writer->connectMyInputTo(filter);        

rspfStdOutProgress progress(0, true);
writer->addListener(&progress);
writer->execute();
writer->removeListener(&progress);

printf("Finished!\n");

std::vector<rspfDpt> features = filter->theKeyPoints;
printf("Number of key points: %d\n",features.size());
for (int i=0; i<features.size(); i++) {
	rspfDpt c = features.at(i);
	printf("(%f,%f)\n",c.x,c.y);
}

return 0;

}


void usage()   
{
  cout << "test_corners <output_type> <input_im> <output_im> [<spec_file>]" << endl 
       << "where output types are: " << endl;
  printOutputTypes();
}

void printOutputTypes()
{
   std::vector<rspfString> outputType;

   rspfImageWriterFactoryRegistry::instance()->getImageTypeList(outputType);
   std::copy(outputType.begin(),
             outputType.end(),
             std::ostream_iterator<rspfString>(cout, "\n"));
}
