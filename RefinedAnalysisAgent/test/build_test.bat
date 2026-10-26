@echo off
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
cd /d C:\ARP\PGSuper\RefinedAnalysisAgent\test
cl /nologo /O2 /EHsc /std:c++20 /W3 /Fe:%1.exe %1.cpp ..\fem\RfaFem.cpp %2 %3 %4
