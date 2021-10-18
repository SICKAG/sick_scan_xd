#!/bin/bash

# 
# Build settings
# 

BUILDLOGFILE=sick_scan_xd_build.log
ERRORLOGFILE=sick_scan_xd_build_errors.log
USECORES=4

# 
# Build libsick_ldmrs on Linux
# 

if [ -d ../../../libsick_ldmrs ] ; then
  pushd ../../../libsick_ldmrs
  if [ ! -d ./build ] ; then mkdir -p ./build ; fi
  cd ./build
  cmake -G "Unix Makefiles" .. 2>&1 | tee -a $BUILDLOGFILE
  make -j$USECORES                        2>&1 | tee -a $BUILDLOGFILE
  echo -e "build libsick_ldmrs: run \"make install\" requires sudo ..."
  sudo make -j$USECORES install
  popd
fi

# 
# Build sick_scan_xd on Linux
# 

pushd ../..
if [ ! -d ./build_linux ] ; then mkdir -p ./build_linux ; fi

cd ./build_linux
rm -f $BUILDLOGFILE
rm -f $ERRORLOGFILE
cmake -DROS_VERSION=0 -G "Unix Makefiles" .. 2>&1 | tee -a $BUILDLOGFILE
make -j$USECORES                             2>&1 | tee -a $BUILDLOGFILE

# Check build errors and warnings
grep "warning:" $BUILDLOGFILE   2>&1 | tee -a $ERRORLOGFILE
grep "undefined:" $BUILDLOGFILE 2>&1 | tee -a $ERRORLOGFILE
grep "error:" $BUILDLOGFILE     2>&1 | tee -a $ERRORLOGFILE
echo -e "---" >> $ERRORLOGFILE
echo -e "\nbuild warnings and errors:"
cat $ERRORLOGFILE

if [ ! -f ./sick_generic_caller ] ; then
    echo -e "\n## ERROR building sick_scan_xd\n"
else
    echo -e "\nmake sick_scan_xd finished."
fi
ls -al ./sick_generic_caller
echo -e "\n" 

popd 

