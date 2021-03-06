# Git Large File Storage

Large scanner datafiles (i.e. converted pcapng-files) for test and development of sick_scan_xd have been added using the "Git Large File Storage" (git lfs).

1. Run the following steps to install git lfs:
   - Download and install git lfs from https://git-lfs.github.com/
   - Integrate to git by running `git lfs install`

2. Add folder "scandata" with large files:
   ```
   git add ./test/emulator/scandata/readme.txt
   git commit -m "added scandata folder"
   git push
   ```

3. Add large files to folder "scandata" using git lfs:
   ```
   git lfs track "./test/emulator/scandata/*.*"
   git add .gitattributes
   ```
   Note: If your current working directory is on a Windows-drive generated by subst, change to the original folder first.

4. Add, commit and push the new files normally:
   ```
   git add ./test/emulator/scandata/*.*
   git commit -m "added scandata files using git lfs"
   git push
   ```

5. Test by cloning sick_scan_xd into a new folder:
   ```
   cd /tmp
   git clone https://github.com/michael1309/sick_scan_xd
   ```
   Note: Use `git clone` to make a local copy of the git lfs files. Downloading the zip file https://github.com/SICKAG/sick_scan_xd/archive/refs/heads/main.zip just creates link-files.

Helpful links:
- https://git-lfs.github.com/
- https://docs.github.com/en/github/managing-large-files/versioning-large-files/configuring-git-large-file-storage
