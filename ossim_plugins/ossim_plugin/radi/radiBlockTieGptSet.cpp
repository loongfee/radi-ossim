#include <iostream>
#include <iomanip>

#include <ossim/base/ossimTieGptSet.h>
#include <ossim/base/ossimString.h>
#include <ossim/base/ossimDatum.h>
#include <ossim/base/ossimNotifyContext.h>

#include <radi/radiBlockTieGpt.h>
#include <radi/radiBlockTieGptSet.h>

using namespace std;

namespace ossimplugins{

const radiBlockTieGptSet&
radiBlockTieGptSet::operator=(const radiBlockTieGptSet& aSet)
{
   if (this != &aSet)
   {
      theTies       = aSet.getTiePoints();
      theMasterPath = aSet.getMasterPath();
      theSlavePath  = aSet.getSlavePath();
      theImageCov   = aSet.getImageCov();
      theGroundCov  = aSet.getGroundCov();
   }
   return *this;
}

void 
radiBlockTieGptSet::addTiePoint(ossimRefPtr<radiBlockTieGpt> aTiePt)
{
   theTies.push_back(aTiePt);
}

void 
radiBlockTieGptSet::clearTiePoints()
{
   theTies.clear();
}

std::ostream& radiBlockTieGptSet::printTab(std::ostream& os) const
{
   os<<"MasterPath: "<<getMasterPath()<<endl;
   os<<"SlavePath: "<<getSlavePath()<<endl;
   os<< std::setiosflags(std::ios::fixed) << std::setprecision(15);
   os<<"ImageCov: (2) " <<symMatrixToText(getImageCov()) <<endl;
   os<<"GroundCov: (3) "<<symMatrixToText(getGroundCov())<<endl;
   os<<"TiePoints: ("<<endl;
   for(vector<ossimRefPtr<radiBlockTieGpt> >::const_iterator it = theTies.begin(); it != theTies.end(); ++it)
   {
      (*it)->printTab(os);
      os<<endl;
   }
   os<<")"<<endl;
   return os;
}

//constants for XML tags
const char* SLAVEPATH_TAG  = "SlavePath";
const char* MASTERPATH_TAG = "MasterPath";
const char* IMAGECOV_TAG   = "ImageCovariance";
const char* GROUNDCOV_TAG  = "GroundCovariance";
const char* TIEPOINTS_TAG  = "SimpleTiePoint";
const char* IMAGES_TAG = "Images";
const char* IMAGE_TAG = "Image";
const char* ID_TAG = "Id";
const char* PATH_TAG = "Path";
const char* TIE_TAG = "Tie";

//exported constants
const char* radiBlockTieGptSet::BLOCKTIEPTSET_TAG = "BlockTiePointSet";


void radiBlockTieGptSet::addImageToList(const std::pair<int, ossimFilename>& aImage)
{
	int newId = aImage.first;
	// check if the id already exists
	bool bExists = false;
	for (int i = 0;i < (int)theImageList.size();++i)
	{
		if(theImageList[i].first == newId)
		{
			bExists = true;
			break;
		}
	}
	if (!bExists)
	{
		theImageList.push_back(aImage);
		// resort by id
		sortImageListById();
	}
}

void radiBlockTieGptSet::addImagesToList(const std::vector< std::pair<int, ossimFilename> >& aImageList)
{
	bool NewAdded = false;
	for (int iNew = 0;iNew < (int)aImageList.size();++iNew)
	{
		int newId = aImageList[iNew].first;
		// check if the id already exists
		bool bExists = false;
		for (int i = 0;i < (int)theImageList.size();++i)
		{
			if(theImageList[i].first == newId)
			{
				bExists = true;
				break;
			}
		}
		if (!bExists)
		{
			theImageList.push_back(aImageList[iNew]);
			NewAdded = true;
		}
	}

	if (NewAdded)
	{
		// resort by id
		sortImageListById();
	}
}

static bool imageIdCompare(std::pair<int, ossimFilename> image1, std::pair<int, ossimFilename> image2)
{
	return image1.first < image2.first;
}

void radiBlockTieGptSet::sortImageListById()
{
	std::sort(theImageList.begin(), theImageList.end(), imageIdCompare);	
}

//export as XML/GML
ossimRefPtr<ossimXmlNode>
radiBlockTieGptSet::exportAsGmlNode(ossimString aGmlVersion)const
{   
   ossimRefPtr<ossimXmlNode> node(new ossimXmlNode);

   node->setTag(BLOCKTIEPTSET_TAG);
   node->addAttribute("xmlns:gml","""http://www.opengis.net/gml"""); //namespace definition

   //add header information : general accuracy + path
   //node->addChildNode(MASTERPATH_TAG,getMasterPath());
   //node->addChildNode(SLAVEPATH_TAG,getSlavePath());
   //node->addChildNode(IMAGECOV_TAG ,symMatrixToText(getImageCov()));
   //node->addChildNode(GROUNDCOV_TAG,symMatrixToText(getGroundCov()));

   ossimRefPtr<ossimXmlNode> imagesNode = node->addNode(IMAGES_TAG);
   for (int i = 0;i < (int)theImageList.size();++i)
   {
	   ossimRefPtr<ossimXmlNode> imageNode = imagesNode->addChildNode(IMAGE_TAG);
	   imageNode->addAttribute(PATH_TAG, theImageList[i].second);
	   imageNode->addAttribute(ID_TAG, ossimString::toString(theImageList[i].first));
   }

   //add all tiepoints
   for(vector<ossimRefPtr<radiBlockTieGpt> >::const_iterator it = theTies.begin(); it != theTies.end(); ++it)
   {
      ossimRefPtr<ossimXmlNode> tienode = (*it)->exportAsGmlNode(aGmlVersion);
      node->addChildNode(tienode.get());
      //TBD : add attribute / counter?
   }

   return node;
}

void radiBlockTieGptSet::appendAsGmlNode(ossimRefPtr<ossimXmlNode>& aGmlNode, ossimString aGmlVersion)const
{   
	if (!aGmlNode.valid())
	{
		aGmlNode = new ossimXmlNode;
	}
	// check BLOCKTIEPTSET_TAG
	if(aGmlNode->getTag() != BLOCKTIEPTSET_TAG)
	{
		aGmlNode = new ossimXmlNode;
		aGmlNode->setTag(BLOCKTIEPTSET_TAG);
		aGmlNode->addAttribute("xmlns:gml","""http://www.opengis.net/gml"""); //namespace definition
	}

	// check IMAGES_TAG
	ossimRefPtr< ossimXmlNode > imagesNode = aGmlNode->findFirstNode(IMAGES_TAG);
	if (!imagesNode.valid())
	{
		imagesNode = aGmlNode->addNode(IMAGES_TAG);
	}
	vector< ossimRefPtr< ossimXmlNode > > imageNodes;
	imagesNode->findChildNodes(IMAGE_TAG, imageNodes);
	for (int i = 0;i < (int)theImageList.size();++i)
	{
		int newId = theImageList[i].first;
		bool bExists = false;
		// check if the image id exists
		for (int j = 0;j < (int)imageNodes.size();++j)
		{
			if(newId == imageNodes[j]->getAttributeValue(ID_TAG).toInt())
			{
				if(imageNodes[j]->getAttributeValue(PATH_TAG) != theImageList[i].second)
				{
					cerr<<"Warning: the image path of id("<<newId<<") is not compatible."<<endl;
				}
				bExists = true;
				break;
			}
		}
		if (!bExists)
		{
			ossimRefPtr<ossimXmlNode> imageNode = imagesNode->addChildNode(IMAGE_TAG);
			imageNode->addAttribute(PATH_TAG, theImageList[i].second);
			imageNode->addAttribute(ID_TAG, ossimString::toString(theImageList[i].first));
		}
	}
	
	//add all tiepoints
	for(vector<ossimRefPtr<radiBlockTieGpt> >::const_iterator it = theTies.begin(); it != theTies.end(); ++it)
	{
		ossimRefPtr<ossimXmlNode> tienode = (*it)->exportAsGmlNode(aGmlVersion);
		aGmlNode->addChildNode(tienode.get());
	}
}

//import from XML/GML
bool
radiBlockTieGptSet::importFromGmlNode(ossimRefPtr<ossimXmlNode> aGmlNode, ossimString aGmlVersion)
{
	clearTiePoints();
	clearImageList();

   // check IMAGES_TAG
   ossimRefPtr< ossimXmlNode > imagesNode = aGmlNode->findFirstNode(IMAGES_TAG);
   if (!imagesNode.valid())
   {
	   imagesNode = aGmlNode->addNode(IMAGES_TAG);
   }
   vector< ossimRefPtr< ossimXmlNode > > imageNodes;
   imagesNode->findChildNodes(IMAGE_TAG, imageNodes);
   for (int i = 0;i < (int)imageNodes.size();++i)
   {
	   int imageId = imageNodes[i]->getAttributeValue(ID_TAG).toInt();
	   ossimFilename imagePath = imageNodes[i]->getAttributeValue(PATH_TAG);
	   theImageList.push_back(std::pair<int, ossimFilename>(imageId, imagePath));
   }
   sortImageListById();

   //load all tie points (skip errors but report them)
   vector< ossimRefPtr< ossimXmlNode > > tienodes;
   aGmlNode->findChildNodes(TIEPOINTS_TAG, tienodes);
   if (tienodes.size() <= 0)
   {
	   ossimNotify(ossimNotifyLevel_WARN) << "WARNING: ossimTieGptSet::importFromGmlNode no tag "<<TIEPOINTS_TAG<<" found\n";
	   return false;
   }

   int badtiecount=0;
   for(vector< ossimRefPtr< ossimXmlNode > >::iterator it=tienodes.begin(); it!=tienodes.end(); ++it)
   {
	   ossimRefPtr<radiBlockTieGpt> temp(new radiBlockTieGpt);
	   if (temp->importFromGmlNode(*it,aGmlVersion)) //pointer hacks nor beautiful nor direct
	   {
		   addTiePoint(temp);
	   } else {
		   badtiecount++;
	   }
   }
   if (badtiecount>0)
   {
	   ossimNotify(ossimNotifyLevel_WARN) << "WARNING: ossimTieGptSet::importFromGmlNode failed to import "<<badtiecount<<" tie point(s)\n";
   }
	return true;
}

ossimString
radiBlockTieGptSet::symMatrixToText(
   const NEWMAT::SymmetricMatrix& sym, 
   const ossimString& el_sep,
   const ossimString& row_sep)const
{
   // write lower half matrix (easier to see than upper half when left justifying)
   // separate elements and rows 
   ossimString res = row_sep;
   for (int i=1;i<=sym.Nrows();++i) //indices start at 1
   {
      for (int j=1;j<=i;++j)
      {
         if (j!=1) res += el_sep;
         res += ossimString::toString(sym(i,j));
      }
      res += row_sep;
   }
   return res;
}


NEWMAT::SymmetricMatrix
radiBlockTieGptSet::textToSymMatrix(
   const ossimString& text,
   unsigned int   dim,
   const ossimString& seps)const
{
   //sep can hold multiple possible separators characters
   //we don't know the matrix size yet, so we put everything into a buffer
   vector<double> buffer;

   vector<ossimString> vsv = text.explode(seps);
   for(vector<ossimString>::const_iterator vit=vsv.begin(); vit!=vsv.end(); ++vit)
   {
      if (vit->size() > 0)
      {
         buffer.push_back(vit->toDouble());
      }
   }

   //check number of elements
   if (buffer.size() != (dim*(dim+1))/2)
   {
      ossimNotify(ossimNotifyLevel_WARN) << "WARNING: ossimTieGptSet::textToSymMatrix wrong element number in sym. matrix : " << buffer.size() <<"\n";
      return NEWMAT::SymmetricMatrix();
   }

   //populate lower half sym matrix
   vector<double>::const_iterator it = buffer.begin();
   NEWMAT::SymmetricMatrix sym(dim);
   for(unsigned int i=1;i<=dim;++i)
   {
      for(unsigned int j=1;j<=i;++j)
      {
         sym(i,j) = *(it++);
      }
   }
   return sym;
}

void
radiBlockTieGptSet::getSlaveMasterPoints(
  std::vector<ossimDpt>& imv, 
  std::vector<ossimGpt>& gdv)const
{
   //re-dim
   imv.resize(theTies.size());
   gdv.resize(theTies.size());

   //fill
   std::vector<ossimDpt>::iterator imvit = imv.begin();
   std::vector<ossimGpt>::iterator gdvit = gdv.begin();

   for(vector<ossimRefPtr<radiBlockTieGpt> >::const_iterator it = theTies.begin(); it != theTies.end(); ++it,++imvit,++gdvit)
   {
      *imvit = (*it)->getImagePoint();
      *gdvit = (*it)->getGroundPoint();
   }
}

void
radiBlockTieGptSet::getGroundBoundaries(ossimGpt& gBoundInf, ossimGpt& gBoundSup)const
{
   //init
   gBoundInf.lat = gBoundInf.lon = gBoundInf.hgt = OSSIM_DEFAULT_MAX_PIX_DOUBLE;
   gBoundSup.lat = gBoundSup.lon = gBoundSup.hgt = OSSIM_DEFAULT_MIN_PIX_DOUBLE;

   //return image and/or ground bounds
   for(vector<ossimRefPtr<radiBlockTieGpt> >::const_iterator it = theTies.begin(); it != theTies.end(); ++it)
   {
      const ossimTieGpt& gp = *(*it);
      if (gp.lon > gBoundSup.lon) gBoundSup.lon = gp.lon;
      if (gp.lon < gBoundInf.lon) gBoundInf.lon = gp.lon;
      if (gp.lat > gBoundSup.lat) gBoundSup.lat = gp.lat;
      if (gp.lat < gBoundInf.lat) gBoundInf.lat = gp.lat;
      if (ossim::isnan(gp.hgt) == false)
      {
         if (gp.hgt > gBoundSup.hgt) gBoundSup.hgt = gp.hgt;
         if (gp.hgt < gBoundInf.hgt) gBoundInf.hgt = gp.hgt;
      } 
   }
}

}