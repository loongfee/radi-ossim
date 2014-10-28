//----------------------------------------------------------------------------
// Copyright (c) 2005, David Burken, all rights reserved.
//
// License:  LGPL
// 
// See LICENSE.txt file in the top level directory for more details.
//
// Author:  David Burken
//
// Description: Constants file for ossim plugins.
//
// $Id: ossimPluginConstants.h 16578 2010-02-10 12:46:11Z gpotts $
//----------------------------------------------------------------------------
#ifndef ossimPluginConstants_HEADER
#define ossimPluginConstants_HEADER

#if defined (__MINGW32__) || defined(__CYGWIN__) || defined(_MSC_VER) || defined(__VISUALC__) || defined(__BORLANDC__) || defined(__WATCOMC__)
#  ifdef OSSIM_PLUGINS_IMPORT
#    define OSSIM_PLUGINS_DLL __declspec(dllimport)
#  else
#    define OSSIM_PLUGINS_DLL __declspec(dllexport)
#  endif
#else
#    define OSSIM_PLUGINS_DLL
#endif

#endif /* #ifndef ossimPluginConstants_HEADER */
