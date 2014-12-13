//----------------------------------------------------------------------------
//
// File: ossimPdalReader.cpp
//
// License:  LGPL
// 
// See LICENSE.txt file in the top level directory for more details.
//
// Author:  David Burken
//
// Description:
// 
// OSSIM Point Data Abstractin Library(PDAL) reader class declaration.
//
//----------------------------------------------------------------------------
// $Id$

#include "ossimPdalReader.h"

#include <ossim/base/ossimBooleanProperty.h>
#include <ossim/base/ossimCommon.h>
#include <ossim/base/ossimIpt.h>
#include <ossim/base/ossimGpt.h>
#include <ossim/base/ossimEndian.h>
#include <ossim/base/ossimKeywordlist.h>
#include <ossim/base/ossimProperty.h>
#include <ossim/base/ossimString.h>
#include <ossim/base/ossimStringProperty.h>
#include <ossim/base/ossimTrace.h>
#include <ossim/imaging/ossimImageGeometryRegistry.h>
#include <ossim/projection/ossimMapProjection.h>
#include <ossim/projection/ossimProjection.h>
#include <ossim/projection/ossimProjectionFactoryRegistry.h>
#include <ossim/support_data/ossimFgdcTxtDoc.h>
#include <ossim/support_data/ossimLasHdr.h>
#include <ossim/support_data/ossimTiffInfo.h>

#include <pdal/Options.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/SpatialReference.hpp>
#include <pdal/Stage.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/StageIterator.hpp>

// #include <pdal/drivers/las/Reader.hpp>
// #include <pdal/drivers/las/VariableLengthRecord.hpp>

// #include <pdal/Reader.hpp>
// #include <pdal/filters/Index.hpp>

#include <exception>
#include <fstream>
#include <limits>
#include <sstream>

RTTI_DEF1(ossimPdalReader, "ossimPdalReader", ossimImageHandler)

static ossimTrace traceDebug("ossimPdalReader:debug");
// static ossimTrace traceDump("ossimPdalReader:dump");

static const char SCALE_KW[] = "scale";
static const char SCAN_KW[]  = "scan"; // boolean


ossimPdalReader::ossimPdalReader()
   : ossimImageHandler(),
     m_str(),
     m_rdr(0),
     m_proj(0),
     m_ul(),
     m_lr(),
     m_maxZ(0.0),
     m_minZ(0.0),
     m_scale(),
     m_tile(0),
     m_mutex(),
     m_scan(false), // ???
     m_units(OSSIM_METERS),
     m_unitConverter(0),
     m_entry(0),
     m_entryList(0)
{
   //---
   // Nan out as can be set in several places, i.e. setProperty,
   // loadState and initProjection.
   //---
    m_scale.makeNan();
}

ossimPdalReader::~ossimPdalReader()
{
   close();
}

bool ossimPdalReader::open()
{
   static const char M[] = "ossimPdal::open";
   if (traceDebug())
   {
      ossimNotify(ossimNotifyLevel_DEBUG)
         << M << " entered...\nfile: " << theImageFile << "\n";
   }
   
   bool result = false;

   close();

   // Open a stream.
   m_str.open(theImageFile.c_str(), std::ios_base::in | std::ios_base::binary);

   if ( m_str.good() )
   {
      //---
      // Notes:
      // 1) PDAL library has many driver that we don't want to pick up. E.g. nitf.
      // For right now we're checking for specific type of las.  Will want to add
      // types / generalize as we need it.
      // 2) The createReader method is generic.  Currently called inside of openLas method.
      //---
      
      // Check for las:
      result = openLas();
   }

   if ( !result ) close();

   if (traceDebug())
   {
      ossimNotify(ossimNotifyLevel_DEBUG) << M << " exit status = " << (result?"true\n":"false\n");
   }
   
   return result;
}

bool ossimPdalReader::openLas()
{
   static const char M[] = "ossimPdal::openLas";
   if (traceDebug())
   {
      ossimNotify(ossimNotifyLevel_DEBUG) << M << " entered...\n";
   }

   bool result = false;

   ossimLasHdr hdr;
   if ( hdr.checkSignature( m_str ) )
   {
      m_str.seekg(0, std::ios_base::beg);
      hdr.readStream( m_str );
      
      if ( hdr.getNumberOfPoints() )
      {
         // Must create reader before projection.
         m_rdr = createReader( theImageFile.string() );
         if ( m_rdr )
         {
            // Find projection:
            if ( hdr.getWktBit() == false )
            {
               result = createProjectionFromLasGeotiff( hdr );
            }
            else
            {
               result = createProjectionFromLasWkt( hdr );
            }
            if ( result )
            {
               initEntryList( hdr );
               initTile();
            }
         }
         
      } // Matches: if ( hdr.getNumberOfPoints() )
      
   } // Matches: if ( hdr.checkSignature( m_str ) )
   
   if ( !result )
   {
      close();
   }

   if (traceDebug())
   {
      ossimNotify(ossimNotifyLevel_DEBUG) << M << " exit status = " << (result?"true\n":"false\n");
   }
   
   return result;
   
} // bool ossimPdalReader::openLas()

bool ossimPdalReader::createProjectionFromLasGeotiff( const ossimLasHdr& hdr )
{
   static const char M[] = "ossimPdalReader::createProjectionFromLasGeotiff";

   if (traceDebug()) ossimNotify(ossimNotifyLevel_DEBUG) << M << " entered...\n";
   
   bool result = false;

   // Seek to the end of the header:
   
   std::streamoff pos = static_cast<std::streamoff>(hdr.getHeaderSize());
   m_str.clear();
   m_str.seekg(pos, std::ios_base::beg);

   // Variable length records(vlr):
   ossim_uint32 vlrCount = hdr.getNumberOfVlrs();
   if ( vlrCount )
   {
      ossim_uint16 reserved;
      char uid[17];
      uid[16]='\n';
      ossim_uint16 recordId;
      ossim_uint16 length;
      char des[33];
      des[32] = '\n';
      
      //---
      // Things we need to save for printGeoKeys:
      //---
      ossim_uint16*  geoKeyBlock     = 0;
      ossim_uint64   geoKeyLength    = 0;
      ossim_float64* geoDoubleBlock  = 0;
      ossim_uint64   geoDoubleLength = 0;
      ossim_int8*    geoAsciiBlock   = 0;
      ossim_uint64   geoAsciiLength  = 0;
      
      ossimEndian* endian = 0;
      // LAS LITTLE ENDIAN:
      if ( ossim::byteOrder() == OSSIM_BIG_ENDIAN )
      {
         endian = new ossimEndian;
      }
      
      for ( ossim_uint32 i = 0; i < vlrCount; ++i )
      {
         m_str.read((char*)&reserved, 2);
         m_str.read(uid, 16);
         m_str.read((char*)&recordId, 2);
         m_str.read((char*)&length, 2);
         m_str.read(des, 32);
         
         // LAS LITTLE ENDIAN:
         if ( endian )
         {
            endian->swap(recordId);
            endian->swap(length);
         }
         
         if ( traceDebug() )
         {
            ossimNotify(ossimNotifyLevel_DEBUG)
               << "uid:      " << uid
               << "\nrecordId: " << recordId
               << "\nlength:   " << length
               << "\ndes:      " << des
               << std::endl;
         }
         
         if (recordId == 34735) // GeoTiff projection keys.
         {
            geoKeyLength = length/2;
            geoKeyBlock = new ossim_uint16[geoKeyLength];
            m_str.read((char*)geoKeyBlock, length);
            
         }
         else if (recordId == 34736) // GeoTiff double parameters.
         {
            geoDoubleLength = length/8;
            geoDoubleBlock = new ossim_float64[geoDoubleLength];
            m_str.read((char*)geoDoubleBlock, length);
         }
         else if (recordId == 34737) // GeoTiff ascii block.
         {
            geoAsciiLength = length;
            geoAsciiBlock = new ossim_int8[length];
            m_str.read((char*)geoAsciiBlock, length);
         }
         else
         {
            m_str.seekg(length, ios_base::cur);
         }
         
      } // End loop through variable length records.

      //---
      // Must have the geoKeyBlock and a geoDoubleBlock for a projection.
      // Note the geoAsciiBlock is not needed, i.e. only informational.
      //---
      if ( geoKeyBlock && geoDoubleBlock )
      {
         if ( endian )
         {
            endian->swap(geoKeyBlock, geoKeyLength);
            endian->swap(geoDoubleBlock, geoDoubleLength);
         }

         //---
         // Give the geokeys to ossimTiffInfo to get back a keyword list that can be fed to
         // ossimProjectionFactoryRegistry::createProjection
         //---
         ossimTiffInfo info;
         ossimKeywordlist geomKwl;
         info.getImageGeometry(geoKeyLength, geoKeyBlock,
                               geoDoubleLength,geoDoubleBlock,
                               geoAsciiLength,geoAsciiBlock,
                               geomKwl);
         
         // Create the projection.
         m_proj = ossimProjectionFactoryRegistry::instance()->createProjection(geomKwl);
         if (m_proj.valid())
         {
            // Units must be set before initValues and initProjection.
            initUnits(geomKwl);
            
            // Must be called before initProjection.
            initValues( hdr );
            
            result = initProjection();  // Sets the ties and scale...
            
            if (traceDebug())
            {
               m_proj->print(ossimNotify(ossimNotifyLevel_DEBUG));
            }
         }
      }

      if ( geoKeyBlock )
      {
         delete [] geoKeyBlock;
         geoKeyBlock = 0;
      }
      if (geoDoubleBlock)
      {
         delete [] geoDoubleBlock;
         geoDoubleBlock = 0;
      }
      if (geoAsciiBlock)
      {
         delete [] geoAsciiBlock;
         geoAsciiBlock = 0;
      }

      // m_str.seekg(origPos);
   }  

   if (traceDebug())
   {
      ossimNotify(ossimNotifyLevel_DEBUG)
         << M << " exit status = " << (result?"true\n":"false\n");
   }

   return result;
   
} // End: ossimPdalReader::createProjectionFromLasGeotiff

bool ossimPdalReader::createProjectionFromLasWkt( const ossimLasHdr& hdr )
{
   static const char M[] = "ossimPdalReader::createProjectionFromLasWkt";

   if (traceDebug()) ossimNotify(ossimNotifyLevel_DEBUG) << M << " entered...\n";

   bool result = false;
   
   // Seek to the end of the header:
   std::streamoff pos = static_cast<std::streamoff>(hdr.getHeaderSize());
   m_str.clear();
   m_str.seekg(pos, std::ios_base::beg);
   
   // Variable length records(vlr):
   ossim_uint32 vlrCount = hdr.getNumberOfVlrs();
   if ( vlrCount )
   {
      ossim_uint16 reserved;
      char uid[17];
      uid[16]='\n';
      ossim_uint16 recordId;
      ossim_uint16 length;
      char des[33];
      des[32] = '\n';
      
      ossimEndian* endian = 0;
      // LAS LITTLE ENDIAN:
      if ( ossim::byteOrder() == OSSIM_BIG_ENDIAN )
      {
         endian = new ossimEndian;
      }
      
      for ( ossim_uint32 i = 0; i < vlrCount; ++i )
      {
         m_str.read((char*)&reserved, 2);
         m_str.read(uid, 16);
         m_str.read((char*)&recordId, 2);
         m_str.read((char*)&length, 2);
         m_str.read(des, 32);
         
         // LAS LITTLE ENDIAN:
         if ( endian )
         {
            endian->swap(recordId);
            endian->swap(length);
         }
         
         if ( traceDebug() )
         {
            ossimNotify(ossimNotifyLevel_DEBUG)
               << "uid:      " << uid
               << "\nrecordId: " << recordId
               << "\nlength:   " << length
               << "\ndes:      " << des
               << std::endl;
         }

         //---
         // Record ID:
         // 2111 = OGC Math Transform WKT Record
         // 2112 = OCC Coordinate Sytem WKT Record
         //---
         std::string wktTransorm;
         std::string wktProjection;
         if ( ( recordId == 2111 ) || ( recordId == 2112 ) )
         {
            // Get the field:
            char* record = new char[length];
            m_str.read(record, length);

            // Check for null terminations.
            if ( record[length-1] != '\0' )
            {
               // Not to spec, send warning?
               record[length-1] = '\0';
            }

            if ( recordId == 2111 )
            {
               wktTransorm = record;
            }
            else // 2112
            {
               wktProjection = record;
            }

            // Cleanup:
            delete record;
            record = 0;

            ossimNotify(ossimNotifyLevel_WARN)
               << M
               << " Notice:  Please provide sample data with wkt variale length record "
               << "to ossim software team at Radiantblue Technologies.\n";
         }
              
      } // End loop through variable length records.
   }
      
   if (traceDebug())
   {
      ossimNotify(ossimNotifyLevel_DEBUG)
         << M << " exit status = " << (result?"true\n":"false\n");
   }

   return result;
   
} // End: ossimPdalReader::createProjectionFromLasGeotiff

void ossimPdalReader::completeOpen()
{
   establishDecimationFactors();
}

void ossimPdalReader::close()
{
   m_entry = 0;
   m_tile  = 0;
   m_proj  = 0;
   if ( m_unitConverter )
   {
      delete m_unitConverter;
      m_unitConverter = 0;
   }
   if ( m_rdr )
   {
      delete m_rdr;
      m_rdr = 0;
   }
   if ( isOpen() )
   {
      m_str.close();
      ossimImageHandler::close();
   }
}

ossimRefPtr<ossimImageData> ossimPdalReader::getTile(
   const  ossimIrect& tile_rect, ossim_uint32 resLevel)
{
   if ( m_tile.valid() )
   {
      // Image rectangle must be set prior to calling getTile.
      m_tile->setImageRectangle(tile_rect);

      if ( getTile( m_tile.get(), resLevel ) == false )
      {
         if (m_tile->getDataObjectStatus() != OSSIM_NULL)
         {
            m_tile->makeBlank();
         }
      }
   }
   return m_tile;
}

bool ossimPdalReader::getTile(ossimImageData* result, ossim_uint32 resLevel)
{
   bool status = false;

   if ( m_rdr && result && (result->getScalarType() == OSSIM_FLOAT32) &&
        (result->getDataObjectStatus() != OSSIM_NULL) &&
        !m_ul.hasNans() && !m_scale.hasNans() )
   {
      const pdal::Schema& schema = m_rdr->getSchema();

      // Not sure what capacity should be. One for now.
      pdal::PointBuffer data(schema, 1);
      
      pdal::StageSequentialIterator* iter = m_rdr->createSequentialIterator(data);
      if ( iter )
      {
         status = true;
         
         const ossimIrect  TILE_RECT   = result->getImageRectangle();
         const ossim_int32 TILE_HEIGHT = static_cast<ossim_int32>(TILE_RECT.height());
         const ossim_int32 TILE_WIDTH  = static_cast<ossim_int32>(TILE_RECT.width());
         const ossim_int32 TILE_SIZE   = static_cast<ossim_int32>(TILE_RECT.area());
         
         const ossim_uint16 ENTRY = m_entry+1;
         
         // Get the scale for this resLevel:
         ossimDpt scale;
         getScale(scale, resLevel);
         
         // Set the starting upper left of upper left pixel for this tile.
         const ossimDpt UL_PROG_PT( m_ul.x - scale.x / 2.0 + TILE_RECT.ul().x * scale.x,
                                    m_ul.y + scale.y / 2.0 - TILE_RECT.ul().y * scale.y);
         
         //---
         // Set the lower right to the edge of the tile boundary.  This looks like an
         // "off by one" error but it's not.  We want the ossimDrect::pointWithin to
         // catch any points in the last line sample.
         //---
         const ossimDpt LR_PROG_PT( UL_PROG_PT.x + TILE_WIDTH  * scale.x,
                                    UL_PROG_PT.y - TILE_HEIGHT * scale.y);
         
         const ossimDrect PROJ_RECT(UL_PROG_PT, LR_PROG_PT, OSSIM_RIGHT_HANDED);
         
#if 0  /* Please leave for debug. (drb) */
         cout << "m_ul: " << m_ul
              << "\nm_scale: " << m_scale
              << "\nscale:   " << scale
              << "\nresult->getScalarType(): " << result->getScalarType()
              << "\nresult->getDataObjectStatus(): " << result->getDataObjectStatus()
              << "\nPROJ_RECT: " << PROJ_RECT
              << "\nTILE_RECT: " << TILE_RECT
              << "\nUL_PROG_PT: " << UL_PROG_PT << endl;
#endif
         
         // Create array of buckets.
         std::vector<ossimPdalReader::Bucket> bucket( TILE_SIZE );
         
         // Loop through the point data.
         ossimDpt xyPt;
         while ( !iter->atEnd() )
         {
            const boost::uint32_t numRead = iter->read(data);
            if (numRead == 1)
            {
               // Check the return number:
               ossim_uint16 returnNumber =
                  data.getField<boost::uint8_t>(m_rdr->getSchema().
                  getDimension("ReturnNumber"), 0);

               if ( returnNumber == ENTRY )
               {
                  boost::int32_t xi =
                     data.getField<boost::int32_t>(m_rdr->getSchema().getDimension("X"), 0);
                  boost::int32_t yi =
                     data.getField<boost::int32_t>(m_rdr->getSchema().getDimension("Y"), 0);
                  boost::int32_t zi =
                     data.getField<boost::int32_t>(m_rdr->getSchema().getDimension("Z"), 0);
                  xyPt.x = m_rdr->getSchema().getDimension("X").applyScaling<boost::int32_t>(xi);
                  xyPt.y = m_rdr->getSchema().getDimension("Y").applyScaling<boost::int32_t>(yi);
                  if ( m_unitConverter )
                  {
                     convertToMeters(xyPt.x);
                     convertToMeters(xyPt.y);
                  }
                  
                  if ( PROJ_RECT.pointWithin( xyPt ) )
                  {
                     // Compute the bucket index:
                     ossim_int32 line = static_cast<ossim_int32>((UL_PROG_PT.y - xyPt.y) / scale.y);
                     ossim_int32 samp = static_cast<ossim_int32>((xyPt.x - UL_PROG_PT.x) / scale.x );
                     ossim_int32 bucketIndex = line * TILE_WIDTH + samp;
                     
                     // Range check and add if in there.
                     if ( ( bucketIndex >= 0 ) && ( bucketIndex < TILE_SIZE ) )
                     {
                        ossim_float64 z =
                           m_rdr->getSchema().getDimension("Z").applyScaling<boost::int32_t>(zi);
                        if ( m_unitConverter ) convertToMeters(z);
                        bucket[bucketIndex].add( z );
                     }
                  }
               }
            }
            
         } // End: while ( !iter->atEnd() )

         //---
         // We must always blank out the tile as we may not have a point in the bucket
         // for every tile point.
         //---
         result->makeBlank();
         
         ossim_float32* buf = result->getFloatBuf(); // Tile buffer to fill.
         
         // Fill the tile.  Currently no band loop:
         for (ossim_int32 i = 0; i < TILE_SIZE; ++i)
         {
            buf[i] = bucket[i].getValue();
         }
         
         // Revalidate.
         result->validate();

         // Cleanup:
         delete  iter;
         iter = 0;
         
      } // Matches: if ( iter )

   } // Matches: if ( m_rdr && result ... )

   return status;
   
} // End: bool ossimPdalReader::getTile(ossimImageData* result, ossim_uint32 resLevel)

ossim_uint32 ossimPdalReader::getNumberOfInputBands() const
{
   return 1; // tmp
}

ossim_uint32 ossimPdalReader::getNumberOfLines(ossim_uint32 resLevel) const
{
   ossim_uint32 result = 0;
   if ( isOpen() )
   {
      result = static_cast<ossim_uint32>(ceil(m_ul.y - m_lr.y) / m_scale.y);
      if (resLevel) result = (result>>resLevel);
   }
   return result;
}

ossim_uint32 ossimPdalReader::getNumberOfSamples(ossim_uint32 resLevel) const
{
   ossim_uint32 result = 0;
   if ( isOpen() )
   {
      result = static_cast<ossim_uint32>(ceil(m_lr.x - m_ul.x) / m_scale.x);
      if (resLevel) result = (result>>resLevel);
   }
   return result;
}

ossim_uint32 ossimPdalReader::getImageTileWidth() const
{
   return 0; // Todo: Add check for tiled reader.
}

ossim_uint32 ossimPdalReader::getImageTileHeight() const
{
   return 0; // Todo: Add check for tiled reader.
}

ossim_uint32 ossimPdalReader::getTileWidth() const
{
   ossimIpt ipt;
   ossim::defaultTileSize(ipt);
   return ipt.x;
}

ossim_uint32 ossimPdalReader::getTileHeight() const
{
   ossimIpt ipt;
   ossim::defaultTileSize(ipt);
   return ipt.y; 
}

ossimScalarType ossimPdalReader::getOutputScalarType() const
{
   return OSSIM_FLOAT32;
}

void ossimPdalReader::getEntryList(std::vector<ossim_uint32>& entryList)const
{
   if ( isOpen() )
   {
      entryList = m_entryList;
   }
   else
   {
      entryList.clear();
   }
}

ossim_uint32 ossimPdalReader::getCurrentEntry() const
{
   return static_cast<ossim_uint32>(m_entry);
}

bool ossimPdalReader::setCurrentEntry(ossim_uint32 entryIdx)
{
   bool result = false;
   if ( m_entry != entryIdx)
   {
      if ( isOpen() )
      {
         std::vector<ossim_uint32>::const_iterator i = m_entryList.begin();
         while ( i != m_entryList.end() )
         {
            if ( (*i) == entryIdx )
            {
               m_entry = entryIdx;
               result = true;
            }
            ++i;
         }
      }
   }
   return result;
}

ossimString ossimPdalReader::getShortName() const
{
   return ossimString("pdal");
}
   
ossimString ossimPdalReader::getLongName()  const
{
   return ossimString("ossim pdal reader");
}

ossimRefPtr<ossimImageGeometry> ossimPdalReader::getImageGeometry()
{
   if ( !theGeometry )
   {
      // Check for external geom:
      theGeometry = getExternalImageGeometry();
      
      if ( !theGeometry )
      {
         theGeometry = new ossimImageGeometry();
         if ( m_proj.valid() )
         {
            theGeometry->setProjection( m_proj.get() );
         }
         else
         {
            //---
            // WARNING:
            // Must create/set theGeometry at this point or the next call to 
            // ossimImageGeometryRegistry::extendGeometry will put us in an infinite loop
            // as it does a recursive call back to ossimImageHandler::getImageGeometry().
            //---         

            // Try factories for projection.
            ossimImageGeometryRegistry::instance()->extendGeometry(this);
         }
      }
      
      // Set image things the geometry object should know about.
      initImageParameters( theGeometry.get() );
   }
   
   return theGeometry;
}

double ossimPdalReader::getMinPixelValue(ossim_uint32 /* band */) const
{
   return m_minZ;
}

double ossimPdalReader::getMaxPixelValue(ossim_uint32 /* band */) const
{
   return m_maxZ;
}

double ossimPdalReader::getNullPixelValue(ossim_uint32 /* band */) const
{
   return -99999.0;
}

ossim_uint32 ossimPdalReader::getNumberOfDecimationLevels() const
{
   // Can support any number of rlevels.
   ossim_uint32 result = 1;
   const ossim_uint32 STOP_DIMENSION = 16;
   ossim_uint32 largestImageDimension = getNumberOfSamples(0) > getNumberOfLines(0) ?
      getNumberOfSamples(0) : getNumberOfLines(0);
   while(largestImageDimension > STOP_DIMENSION)
   {
      largestImageDimension /= 2;
      ++result;
   }
   return result;
}

bool ossimPdalReader::saveState(ossimKeywordlist& kwl, const char* prefix)const
{
   kwl.add( prefix, SCALE_KW, m_scale.toString().c_str(), true );
   kwl.add( prefix, SCAN_KW,  ossimString::toString(m_scan).c_str(), true );
   return ossimImageHandler::saveState(kwl, prefix);
}

bool ossimPdalReader::loadState(const ossimKeywordlist& kwl, const char* prefix)
{
   bool result = false;
   if ( ossimImageHandler::loadState(kwl, prefix) )
   {
      result = open();
      if ( result )
      {
         // Get our keywords:
         const char* lookup = kwl.find(prefix, SCALE_KW);
         if ( lookup )
         {
            m_scale.toPoint( ossimString(lookup) );
         }
         lookup = kwl.find(prefix, SCAN_KW);
         if ( lookup )
         {
            ossimString s = lookup;
            m_scan = s.toBool();
         }
      }
   }
   return result;
}

void ossimPdalReader::setProperty(ossimRefPtr<ossimProperty> property)
{
   if ( property.valid() )
   {
      if ( property->getName() == SCALE_KW )
      {
         ossimString s;
         property->valueToString(s);
         ossim_float64 d = s.toFloat64();
         if ( ossim::isnan(d) == false )
         {
            setScale( d );
         }
      }
      else if ( property->getName() == SCAN_KW )
      {
         ossimString s;
         property->valueToString(s);
         m_scan = s.toBool();
      }
      else
      {
         ossimImageHandler::setProperty(property);
      }
   }
}

ossimRefPtr<ossimProperty> ossimPdalReader::getProperty(const ossimString& name)const
{
   ossimRefPtr<ossimProperty> prop = 0;
   if ( name == SCALE_KW )
   {
      ossimString value = ossimString::toString(m_scale.x);
      prop = new ossimStringProperty(name, value);
   }
   else if ( name == SCAN_KW )
   {
      prop = new ossimBooleanProperty(name, m_scan);
   }
   else
   {
      prop = ossimImageHandler::getProperty(name);
   }
   return prop;
}

void ossimPdalReader::getPropertyNames(std::vector<ossimString>& propertyNames)const
{
   propertyNames.push_back( ossimString(SCALE_KW) );
   propertyNames.push_back( ossimString(SCAN_KW) );
   ossimImageHandler::getPropertyNames(propertyNames);
}

#if 0
bool ossimPdalReader::init()
{
   bool result = false;

   if ( isOpen() )
   {
      // result = parseVarRecords();

      if ( !result )
      {
         result = initFromExternalMetadata(); // Checks for external FGDC text file.
      }
      
      // There is nothing we can do if parseVarRecords fails.
      if ( result )
      {
         initTile();
      }
   }
   
   return result;
}
#endif

void ossimPdalReader::initEntryList( const ossimLasHdr& hdr )
{
   m_entryList.clear();
   if ( isOpen() )
   {
      for ( ossim_uint32 entry = 0; entry < 15; ++entry )
      {
         if ( hdr.getNumberOfPoints(entry) ) m_entryList.push_back(entry);
      }
   }
}

bool ossimPdalReader::initProjection()
{
   bool result = true;
   
   ossimMapProjection* proj = dynamic_cast<ossimMapProjection*>( m_proj.get() );
   if ( proj )
   {
      //---
      // Set the tie and scale:
      // Note the scale can be set in other places so only set here if it
      // has nans.
      //---
      if ( proj->isGeographic() )
      {
         ossimGpt gpt(m_ul.y, m_ul.x, 0.0, proj->getDatum() );
         proj->setUlTiePoints( gpt );

         if ( m_scale.hasNans() )
         {
            m_scale = proj->getDecimalDegreesPerPixel();
            if ( m_scale.hasNans() || !m_scale.x || !m_scale.y )
            {
               // Set to some default:
               m_scale.x = 0.000008983; // About 1 meter at the Equator.
               m_scale.y = m_scale.x;
               proj->setDecimalDegreesPerPixel( m_scale );
            }
         }
      }
      else
      {
         proj->setUlTiePoints(m_ul);

         if ( m_scale.hasNans() )
         {
            m_scale = proj->getMetersPerPixel();
            if ( m_scale.hasNans() || !m_scale.x || !m_scale.y )
            {
               // Set to some default:
               m_scale.x = 1.0;
               m_scale.y = 1.0;
               proj->setMetersPerPixel( m_scale );
            }
         }
      }
   }
   else
   {
      result = false;
      m_ul.makeNan();
      m_lr.makeNan();
      m_scale.makeNan();
      
      ossimNotify(ossimNotifyLevel_WARN)
         << "ossimPdalReader::initProjection WARN Could not cast to map projection!"
         << std::endl;
   }

   return result;
   
} // bool ossimPdalReader::initProjection()

void ossimPdalReader::initTile()
{
   const ossim_uint32 BANDS = getNumberOfOutputBands();

   m_tile = new ossimImageData(this,
                               getOutputScalarType(),
                               BANDS,
                               getTileWidth(),
                               getTileHeight());

   for(ossim_uint32 band = 0; band < BANDS; ++band)
   {
      m_tile->setMinPix(getMinPixelValue(band),   band);
      m_tile->setMaxPix(getMaxPixelValue(band),   band);
      m_tile->setNullPix(getNullPixelValue(band), band);
   }

   m_tile->initialize();
}

void ossimPdalReader::initUnits(const ossimKeywordlist& geomKwl)
{
   ossimMapProjection* proj = dynamic_cast<ossimMapProjection*>( m_proj.get() );
   if ( proj )
   {
      if ( proj->isGeographic() )
      {
         m_units = OSSIM_DEGREES;
      }
      else
      {
         const char* lookup = geomKwl.find("image0.linear_units");
         if ( lookup )
         {
            std::string units = lookup;
            if ( units == "meters" )
            {
               m_units = OSSIM_METERS;
            }  
            else if ( units == "feet" )
            {
               m_units = OSSIM_FEET;
            }
            else if ( units == "us_survey_feet" )
            {
               m_units = OSSIM_US_SURVEY_FEET;
            }
            else
            {
               ossimNotify(ossimNotifyLevel_DEBUG)
                  << "ossimPdalReader::initUnits WARN:\n"
                  << "Unhandled linear units code: " << units << std::endl;
            }
         }
      }
   }

   if ( m_units != OSSIM_METERS && !m_unitConverter )
   {
      m_unitConverter = new ossimUnitConversionTool();
   }
}

void ossimPdalReader::initValues( const ossimLasHdr& hdr )
{
   static const char M[] = "ossimPdalReader::initValues";
   
   if ( m_scan )
   {
      // Set to bogus values to start.
      m_ul.x = numeric_limits<ossim_float64>::max();
      m_ul.y = numeric_limits<ossim_float64>::min();
      m_lr.x = numeric_limits<ossim_float64>::min();
      m_lr.y = numeric_limits<ossim_float64>::max();
      m_maxZ = numeric_limits<ossim_float64>::min();
      m_minZ = numeric_limits<ossim_float64>::max();

      const pdal::Schema& schema = m_rdr->getSchema();
      pdal::PointBuffer data(schema, 1);
      pdal::StageSequentialIterator* iter = m_rdr->createSequentialIterator(data);

      if ( iter )
      {
         // Loop through the point data.
         ossimDpt xyPt;
         while ( !iter->atEnd() )
         {
            const boost::uint32_t numRead = iter->read(data);
            if (numRead == 1)
            {
               boost::int32_t xi =
                  data.getField<boost::int32_t>(m_rdr->getSchema().getDimension("X"), 0);
               boost::int32_t yi =
                  data.getField<boost::int32_t>(m_rdr->getSchema().getDimension("Y"), 0);
               boost::int32_t zi =
                  data.getField<boost::int32_t>(m_rdr->getSchema().getDimension("Z"), 0);
               ossim_float64 x =
                  m_rdr->getSchema().getDimension("X").applyScaling<boost::int32_t>(xi);
               ossim_float64 y =
                  m_rdr->getSchema().getDimension("Y").applyScaling<boost::int32_t>(yi);
               ossim_float64 z =
                  m_rdr->getSchema().getDimension("Z").applyScaling<boost::int32_t>(zi);
               ossim_uint16 returnNumber =
                  data.getField<boost::uint8_t>(m_rdr->getSchema().
                  getDimension("ReturnNumber"), 0);

               if ( x < m_ul.x ) m_ul.x = x;
               if ( x > m_lr.x ) m_lr.x = x;
               if ( y > m_ul.y ) m_ul.y = y;
               if ( y < m_lr.y ) m_lr.y = y;
               if ( z > m_maxZ ) m_maxZ = z;
               if ( z < m_minZ ) m_minZ = z;
            }
            
         } // End: while ( !iter->atEnd() )

         // Cleanup:
         delete  iter;
         iter = 0;
         
      } // Matches: if ( iter )
   }
   else // Set from header file:
   {
      // Set the upper left (tie).
      m_ul.x = hdr.getMinX();
      m_ul.y = hdr.getMaxY();
      
      // Set the lower right.
      m_lr.x = hdr.getMaxX();
      m_lr.y = hdr.getMinY();
      
      // Set the min/max:
      m_minZ = hdr.getMinZ();
      m_maxZ = hdr.getMaxZ();
   }
   
   if ( m_unitConverter ) // Need to convert to meters.
   {
      convertToMeters(m_ul.x);
      convertToMeters(m_ul.y);
      
      convertToMeters(m_lr.x);
      convertToMeters(m_lr.y);
      
      convertToMeters(m_maxZ);
      convertToMeters(m_minZ);
   }
   
   if ( traceDebug() )
   {
      ossimNotify(ossimNotifyLevel_DEBUG)
         << M << " Bounds:"
         << "\nul:   " << m_ul
         << "\nlr:   " << m_lr
         << "\nminZ: " << m_minZ
         << "\nmaxZ: " << m_maxZ << "\n";
   }
   
} // End: ossimPdalReader::initValues( const ossimLasHdr& hdr )

#if 0
bool ossimPdalReader::initFromExternalMetadata()
{
   static const char M[] = "ossimPdal::initFromExternalMetadata";
   if (traceDebug()) ossimNotify(ossimNotifyLevel_DEBUG) << M << " entered...\n";
   
   bool result = false;

   ossimFilename fgdcFile = theImageFile;
   fgdcFile.setExtension("txt");
   if ( fgdcFile.exists() == false )
   {
      fgdcFile.setExtension("TXT");
   }

   if ( fgdcFile.exists() )
   {
      ossimRefPtr<ossimFgdcTxtDoc> fgdcDoc = new ossimFgdcTxtDoc();
      if ( fgdcDoc->open( fgdcFile ) )
      {
         fgdcDoc->getProjection( m_proj );
         if ( m_proj.valid() )
         {
            // Units must be set before initValues and initProjection.
            std::string units;
            fgdcDoc->getAltitudeDistanceUnits(units);
            if ( ( units == "feet" ) || ( units == "international feet" ) )
            {
               m_units = OSSIM_FEET;
            }
            else if ( units == "survey feet" )
            {
               m_units = OSSIM_US_SURVEY_FEET;
            }
            else
            {
               m_units = OSSIM_METERS;
            }
            
            // Must be called before initProjection.
            initValues();
            
            result = initProjection();  // Sets the ties and scale...
            
            if (traceDebug())
            {
               m_proj->print(ossimNotify(ossimNotifyLevel_DEBUG));
            }
         }
      }
   }

   if (traceDebug())
   {
      ossimNotify(ossimNotifyLevel_DEBUG) << M << " exit status = " << (result?"true\n":"false\n");
   }   
   return result;
}
#endif

void ossimPdalReader::getScale(ossimDpt& scale, ossim_uint32 resLevel) const
{
   // std::pow(2.0, 0) returns 1.
   ossim_float64 d = std::pow(2.0, static_cast<double>(resLevel));
   scale.x = m_scale.x * d;
   scale.y = m_scale.y * d;
}

void ossimPdalReader::setScale( const ossim_float64& scale )
{
   m_scale.x = scale;
   m_scale.y = m_scale.x;

   if ( m_proj.valid() )
   {
      ossimMapProjection* proj = dynamic_cast<ossimMapProjection*>( m_proj.get() );
      if ( proj  && ( m_scale.hasNans() == false ) )
      {
         if ( proj->isGeographic() )
         {
            proj->setDecimalDegreesPerPixel( m_scale );
         }
         else
         {
            proj->setMetersPerPixel( m_scale );
         }
      }
   }
}

pdal::Stage* ossimPdalReader::createReader( const ossimFilename& file )
{
   pdal::Stage* reader = 0;
   
   pdal::Options options;
   {
      options.add<std::string>( "filename", file.string() );

      // Please leave for debug. (drb)
      // options.add<bool>("debug", isDebug());
      // options.add<boost::uint32_t>("verbose", getVerboseLevel());
   }

   pdal::StageFactory factory;
   std::string driver = factory.inferReaderDriver(file.string(), options);

   if (traceDebug())
   {
      ossimNotify(ossimNotifyLevel_DEBUG)
         << "ossimPdalReader::createReader driver: " << driver << "\n";
   }
   
   if ( driver.size() )
   {
      reader = factory.createReader(driver, options);
   }
   
   return reader;
}
