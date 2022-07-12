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

echo -e "sudo build/install libsick_ldmrs ..."
sudo echo -e "build/install libsick_ldmrs..."
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

echo -e "build sick_scan_xd ..."
pushd ../..
if [ ! -d ./build_linux ] ; then mkdir -p ./build_linux ; fi

cd ./build_linux
rm -f $BUILDLOGFILE
rm -f $ERRORLOGFILE
export ROS_VERSION=0
# cmake -DROS_VERSION=0 -DCMAKE_ENABLE_EMULATOR=1 -DSCANSEGMENT_XD=0 -G "Unix Makefiles" .. 2>&1 | tee -a $BUILDLOGFILE
cmake -DROS_VERSION=0 -DCMAKE_ENABLE_EMULATOR=1 -G "Unix Makefiles" .. 2>&1 | tee -a $BUILDLOGFILE
make -j$USECORES                                                       2>&1 | tee -a $BUILDLOGFILE

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
if [ ! -f ./libsick_scan_shared_lib.so ] ; then
    echo -e "\n## ERROR building libsick_scan_shared_lib.so\n"
else
    echo -e "\nmake libsick_scan_shared_lib.so finished."
fi
if [ ! -f ./sick_scan_xd_api_test ] ; then
    echo -e "\n## ERROR building sick_scan_xd_api_test\n"
else
    echo -e "make sick_scan_xd_api_test finished."
fi
ls -al ./sick_generic_caller
ls -al ./libsick_scan_shared_lib.so
ls -al ./sick_scan_xd_api_test
ldd -r ./libsick_scan_shared_lib.so # print undefined symbols in libsick_scan_shared_lib.so
echo -e "\n" 

popd 

