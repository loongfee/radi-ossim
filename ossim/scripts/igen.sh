#!/bin/bash
#--------------------------------------------------------------------
# -*-Shell-script-*-
#
# Script wrapper for ossim igen application.  Intention is to set
# environment up for application, then call it.
#
# License: LGPL
#
# $Id: igen.sh 7891 2005-08-05 12:54:29Z dburken $
#--------------------------------------------------------------------

# Source the environment file.
. /usr/local/ossim/bin/ossim_environment

# Call the igen application passing all args.
igen $*
