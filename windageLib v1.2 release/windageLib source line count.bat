@echo off

set MyDate=%date:~2%
echo -------------------------------------------------------------------------------
echo                  windage library source code line counter
echo                                                      - 20%MyDate% %time%
echo -------------------------------------------------------------------------------
echo finding files
echo         in "include" "src" "Examples" "Test Programs" "Research Projects"
cloc.exe "include" "src" "Examples" "Test Programs" "Research Projects"

pause