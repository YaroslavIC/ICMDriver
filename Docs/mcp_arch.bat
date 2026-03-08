set filename=mn%time:~0,2%%time:~3,2%%time:~6,2%.tar
tar -cf %filename% ..\\Core\Inc\mcp*.h ..\\Core\Inc\main.h ..\\Core\Src\mcp*.c ..\\Core\Src\main.c ..\\Core\Src\stm32f4xx_it.c  