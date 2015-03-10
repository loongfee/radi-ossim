

 #include <ossim/radi/vsi.h>


int  GetDataTypeSize( TIFFDataType eDataType )

{
    switch( eDataType )
    {
      case TIFF_BYTE:
        return 8;
	  case TIFF_ASCII:
		   return 8;

      case TIFF_SHORT: 
		  return 16;
      case TIFF_LONG:
        return 32;

      case TIFF_RATIONAL:
		  return 64;
      case TIFF_SBYTE:
		   return 8;
      case TIFF_UNDEFINED:
		  return 8;
      case TIFF_SSHORT:
         return 16;

      case TIFF_SLONG:
		  return 32;
      case TIFF_SRATIONAL:
		  return 64;
      case TIFF_FLOAT:
        return 32;

      case TIFF_DOUBLE:
        return 64;

	   case TIFF_IFD:
        return 32;

      case TIFF_LONG8:
        return 64;
	  case TIFF_SLONG8:
        return 64;
	  case TIFF_IFD8:
        return 64;
      default:
        return 0;
    }
}

 int WW_VSIFCloseL( HANDLE hFile )
 {
 
 
  return CloseHandle( hFile ) ? 0 : -1;
 }



 GUIntBig WW_VSIFTellL(HANDLE hFile)

{
    LARGE_INTEGER   li;

    li.HighPart = 0;
    li.LowPart = SetFilePointer( hFile, 0, (PLONG) &(li.HighPart), FILE_CURRENT );

    return (static_cast<GUIntBig>(li.QuadPart));
}
int WW_VSIFSeekL( GUIntBig nOffset, int nWhence ,HANDLE hFile)

{
    GUInt32       dwMoveMethod, dwMoveHigh;
    GUInt32       nMoveLow;
    LARGE_INTEGER li;

 
    switch(nWhence)
    {
        case SEEK_CUR:
            dwMoveMethod = FILE_CURRENT;
            break;
        case SEEK_END:
            dwMoveMethod = FILE_END;
            break;
        case SEEK_SET:
        default:
            dwMoveMethod = FILE_BEGIN;
            break;
    }

    li.QuadPart = nOffset;
    nMoveLow = li.LowPart;
    dwMoveHigh = li.HighPart;

    SetLastError( 0 );
    SetFilePointer(hFile, (LONG) nMoveLow, (PLONG)&dwMoveHigh,
                       dwMoveMethod);

    if( GetLastError() != NO_ERROR )
    {
#ifdef notdef
        LPVOID      lpMsgBuf = NULL;
        
        FormatMessage( FORMAT_MESSAGE_ALLOCATE_BUFFER 
                       | FORMAT_MESSAGE_FROM_SYSTEM
                       | FORMAT_MESSAGE_IGNORE_INSERTS,
                       NULL, GetLastError(), 
                       MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), 
                       (LPTSTR) &lpMsgBuf, 0, NULL );
 
        printf( "[ERROR %d]\n %s\n", GetLastError(), (char *) lpMsgBuf );
        printf( "nOffset=%u, nMoveLow=%u, dwMoveHigh=%u\n", 
                (GUInt32) nOffset, nMoveLow, dwMoveHigh );
#endif
       
        return -1;
    }
    else
        return 0;
}

 size_t WW_VSIFWriteL( const void *pBuffer, size_t nSize, size_t nCount, HANDLE hFile)

{
    DWORD       dwSizeWritten;
    size_t      nResult;

    if( !WriteFile(hFile, (void *)pBuffer,(DWORD)(nSize*nCount),&dwSizeWritten,NULL) )
    {
        nResult = 0;
 
    }
    else if( nSize == 0)
        nResult = 0;
    else
        nResult = dwSizeWritten / nSize;

    return nResult;
}
 
//size_t VSIWin32Handle::Read( void * pBuffer, size_t nSize, size_t nCount )
size_t WW_VSIFReadL( void * pBuffer, size_t nSize, size_t nCount, HANDLE hFile )

{
    DWORD       dwSizeRead;
    size_t      nResult;

    if( !ReadFile( hFile, pBuffer, (DWORD)(nSize*nCount), &dwSizeRead, NULL ) )
    {
        nResult = 0;
       
    }
    else if( nSize == 0 )
        nResult = 0;
    else
        nResult = dwSizeRead / nSize;

    //if( nResult != nCount )
    //    bEOF = TRUE;

    return nResult;
}

HANDLE WW_VSIFOpenL(const char *pszFilename ,const char *pszAccess)
{

DWORD dwDesiredAccess, dwCreationDisposition, dwFlagsAndAttributes;
    HANDLE hFile;

    if( strchr(pszAccess, '+') != NULL ||
        strchr(pszAccess, 'w') != 0 ||
        strchr(pszAccess, 'a') != 0 )
        dwDesiredAccess = GENERIC_READ | GENERIC_WRITE;
    else
        dwDesiredAccess = GENERIC_READ;

    if( strstr(pszAccess, "w") != NULL )
        dwCreationDisposition = CREATE_ALWAYS;
    else if( strstr(pszAccess, "a") != NULL )
        dwCreationDisposition = OPEN_ALWAYS;
    else
        dwCreationDisposition = OPEN_EXISTING;

    dwFlagsAndAttributes = (dwDesiredAccess == GENERIC_READ) ? FILE_ATTRIBUTE_READONLY : FILE_ATTRIBUTE_NORMAL;
	
 
	        hFile = CreateFile( pszFilename, dwDesiredAccess, 
                            FILE_SHARE_READ | FILE_SHARE_WRITE, 
                            NULL, dwCreationDisposition,  dwFlagsAndAttributes,
                            NULL );

			return hFile;

			}