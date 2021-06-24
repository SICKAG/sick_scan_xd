#
#
# Doxygen preparation for github.io
#
cd ..
mkdir ./html
cd html
lastpart=$(echo $(basename $PWD))
if [ "$lastpart" = "html" ]
then
  echo "html subdir found - starting cleanup process"
else
  echo "The script does not run in an html subdir - aborting"
  exit 0
fi
# Remove any documentation that is currently in this folder
rm -rf *
rm -rf .git
rm -rf .gitignore
rm -rf .*

git clone https://github.com/SICKAG/sick_scan.git .
git checkout master
cd ..
mkdir html/docs
cd html/docs/
rm -rf *
cp ../../doxygen/docs/* .
cd ..
git checkout gh-pages
git checkout master -- include
git checkout master -- driver
doxygen ./docs/Doxyfile
cd ./html
git add .
cd ..
git add include
git add driver
git commit -m "Added Doxygen output to repo"
git push origin gh-pages
cd ../doxygen

