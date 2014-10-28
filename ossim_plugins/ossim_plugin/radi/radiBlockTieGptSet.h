#ifndef radiBlockTieGptSet_HEADER
#define radiBlockTieGptSet_HEADER

#include <iostream>
#include <vector>
#include <ossim/base/ossimXmlNode.h>
#include <ossim/base/ossimTieGpt.h>
#include <ossim/base/ossimFilename.h>
#include <ossim/matrix/newmat.h>
#include <ossim_plugin/radi/radiBlockTieGpt.h>
#include <ossimPluginConstants.h>

class ossimDpt;

namespace ossimplugins{
/**
 * storage class for a set of geographic tie points, between master and slave images
 * + GML (OGC) serialization
 *
 * NOTES
 * master points are stored on the ground
 * ground SRS is EPSG:4326 only (WGS84) + height in meters above ellispoid
 *
 * slave points are stored as image positions
 *
 * general ground/image accuracy values are stored
 *
 * TODO :
 */
class OSSIM_PLUGINS_DLL radiBlockTieGptSet
{
public:

   inline radiBlockTieGptSet() {}

   inline radiBlockTieGptSet(const radiBlockTieGptSet& aSet)
      :
      theTies(aSet.getTiePoints()),
      theMasterPath(aSet.getMasterPath()),
      theSlavePath(aSet.getSlavePath()),
      theImageCov(aSet.getImageCov()),
      theGroundCov(aSet.getGroundCov())
   {}

   inline ~radiBlockTieGptSet() {}

   const radiBlockTieGptSet& operator=(const radiBlockTieGptSet&);

   // accessors   
   inline void  setTiePoints(const vector<ossimRefPtr<radiBlockTieGpt> >& aTieSet) { theTies = aTieSet; }
   inline const vector<ossimRefPtr<radiBlockTieGpt> >& getTiePoints()const         { return theTies; }
   inline       vector<ossimRefPtr<radiBlockTieGpt> >& refTiePoints()              { return theTies; }

   inline void  setMasterPath(const ossimString& aPath) { theMasterPath = aPath; }
   inline const ossimString& getMasterPath()const       { return theMasterPath; }

   inline void  setSlavePath(const ossimString& aPath) { theSlavePath = aPath; }
   inline const ossimString& getSlavePath()const       { return theSlavePath; }

   inline void  setImageCov(const NEWMAT::SymmetricMatrix& aCovMat) { theImageCov = aCovMat; }
   inline const NEWMAT::SymmetricMatrix& getImageCov()const       { return theImageCov; }
   inline       NEWMAT::SymmetricMatrix& refImageCov()            { return theImageCov; }

   inline void  setGroundCov(const NEWMAT::SymmetricMatrix& aCovMat) { theGroundCov = aCovMat; }
   inline const NEWMAT::SymmetricMatrix& getGroundCov()const       { return theGroundCov; }
   inline       NEWMAT::SymmetricMatrix& refGroundCov()            { return theGroundCov; }
  
   void getSlaveMasterPoints(std::vector<ossimDpt>& imv, std::vector<ossimGpt>& gdv)const;

   inline unsigned int size()const { return (unsigned int)theTies.size(); }

   /**
    * operations
    */
   void addTiePoint(ossimRefPtr<radiBlockTieGpt> aTiePt);
   void clearTiePoints();

   void getGroundBoundaries(ossimGpt& gBoundInf, ossimGpt& gBoundSup)const;

   /**
    * text output : header + tab separated tie points
    */
   std::ostream& printTab(std::ostream& os) const;

   /**
    * GML features (XML) serialization
    */
   ossimRefPtr<ossimXmlNode> exportAsGmlNode(ossimString aGmlVersion="2.1.2")const;
   void appendAsGmlNode(ossimRefPtr<ossimXmlNode>& aGmlNode, ossimString aGmlVersion="2.1.2")const;
   bool importFromGmlNode(ossimRefPtr<ossimXmlNode> aGmlNode, ossimString aGmlVersion="2.1.2");
   
  /**
   * Public data members
   */
   static const char* BLOCKTIEPTSET_TAG;
   std::vector< std::pair<int, ossimFilename> > getImageList(){return theImageList;};
   void clearImageList(){theImageList.clear();};
   void addImageToList(const std::pair<int, ossimFilename>& aImage);
   void addImagesToList(const std::vector< std::pair<int, ossimFilename> >& aImageList);
   void sortImageListById();
protected:
   /**
    * Protected data members
    */
   std::vector<ossimRefPtr<radiBlockTieGpt> > theTies;      //store by reference so derived classes can be used
   ossimString                       theMasterPath; //!full or relative path to master dataset
   ossimString                       theSlavePath; //!full or relative path to slave dataset
   NEWMAT::SymmetricMatrix           theImageCov;  //! image error covariance matrix
   NEWMAT::SymmetricMatrix           theGroundCov; //! ground error covariance matrix
   std::vector< std::pair<int, ossimFilename> > theImageList;

   /**
    * Protected methods
    */
   ossimString symMatrixToText(const NEWMAT::SymmetricMatrix& sym, 
                               const ossimString& el_sep=" ",
                               const ossimString& row_sep=";")const;

   NEWMAT::SymmetricMatrix textToSymMatrix(const ossimString& text,
                               unsigned int dim,
                               const ossimString& seps=" ;\t\r\n")const; //list of possible elements or row separators

};
}

#endif /* #ifndef radiBlockTieGptSet_HEADER */
