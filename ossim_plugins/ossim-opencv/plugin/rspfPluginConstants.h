//----------------------------------------------------------------------------
// Copyright (c) 2005, David Burken, all rights reserved.
//
// License:  LGPL
// 
// See LICENSE.txt file in the top level directory for more details.
//
// Author:  David Burken
//
// Description: Constants file for rspf plugins.
//
// $Id: rspfPluginConstants.h 7530 2005-05-23 17:21:24Z dburken $
//----------------------------------------------------------------------------
#ifndef rspfPluginConstants_HEADER
#define rspfPluginConstants_HEADER

#if defined(__MINGW32__) || defined(__CYGWIN__) || defined(_MSC_VER) || defined(__VISUALC__) || defined(__BORLANDC__) || defined(__WATCOMC__)
#  define RSPF_PLUGINS_DLL __declspec(dllexport)
#else
# define RSPF_PLUGINS_DLL
#endif

#endif /* #ifndef rspfPluginConstants_HEADER */
