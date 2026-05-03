set "filename=mn%time:~0,2%%time:~3,2%%time:~6,2%.zip" 
winRAR.EXE a -afzip "%filename%" ..\Core\Inc\*.h ..\Core\Src\*.c ..\Docs\*.py 

