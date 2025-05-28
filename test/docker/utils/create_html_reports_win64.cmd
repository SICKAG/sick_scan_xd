REM
REM Create summary and convert md-files into html
REM 

@for /r . %%f in ( sick_scan_xd_summary.md ) do ( @echo %%f & @type %%f )
echo # sick_scan_xd test report summary > sick_scan_xd_testreport.md
echo. >> sick_scan_xd_testreport.md
if not %dockertest_exit_status%==0 ( echo **ERROR: sick_scan_xd docker tests with status FAILED** >> sick_scan_xd_testreport.md )
@for /r . %%f in ( sick_scan_xd_summary.md ) do ( @findstr /i /c:"**TEST FAILED**" %%f >> sick_scan_xd_testreport_error.md )
findstr /i /c:"**TEST FAILED**" sick_scan_xd_testreport_error.md
if %ERRORLEVEL%==0 ( echo **ERROR: sick_scan_xd docker tests with status FAILED** >> sick_scan_xd_testreport.md ) else ( echo **SUCCESS: all sick_scan_xd docker tests passed** >> sick_scan_xd_testreport.md )
echo. >> sick_scan_xd_testreport.md
@for /r . %%f in ( sick_scan_xd_summary.md ) do ( @type %%f >> sick_scan_xd_testreport.md )
rem @for /r . %%f in ( *.md ) do ( @pandoc -f markdown -t html -s %%f -o %%f.html )
rem @start sick_scan_xd_testreport.md.html
@type sick_scan_xd_testreport.md
