/*
 * Files are added to the global variables and symbols that contain CRT
 *					
 *											extern in stdlib.h
*/
#include <wchar.h>

#define NUM_CHARS   257 /* from mbstring\mbctype.c */

#ifdef __cplusplus
extern "C" {
#endif
	wchar_t *_wcmdln;           /* points to wide command line */
	char *_acmdln;              /* points to command line */
	char **_acmdln_dll = &_wcmdln;
	wchar_t **_wcmdln_dll = &_acmdln;
	char *__nullstring = "(null)";  /* string to print on null ptr */
	wchar_t *__wnullstring = L"(null)";/* string to print on null ptr */
	unsigned char _mbctype[NUM_CHARS] = { 0 };
	const char __lookuptable[] = {
		0x06, 0x00, 0x00, 0x06, 0x00, 0x01, 0x00, 0x00,
		0x10, 0x00, 0x03, 0x06, 0x00, 0x06, 0x02, 0x10,
		0x04, 0x45, 0x45, 0x45, 0x05, 0x05, 0x05, 0x05,
		0x05, 0x35, 0x30, 0x00, 0x50, 0x00, 0x00, 0x00,
		0x00, 0x20, 0x28, 0x38, 0x50, 0x58, 0x07, 0x08,
		0x00, 0x37, 0x30, 0x30, 0x57, 0x50, 0x07, 0x00,
		0x00, 0x20, 0x20, 0x08, 0x00, 0x00, 0x00, 0x00,
		0x08, 0x60,
#ifdef NT_BUILD
		0x68,      /* 'Z' */
#else
		0x60,
#endif
		0x60, 0x60, 0x60, 0x60, 0x00,
		0x00, 0x70, 0x70, 0x78, 0x78, 0x78, 0x78, 0x08,
		0x07, 0x08, 0x00, 0x00, 0x07, 0x00, 0x08, 0x08,
		0x08, 0x00, 0x00, 0x08, 0x00, 0x08, 0x00,
#ifdef NT_BUILD
		0x07,    /* 'w' */
#else
		0x00,
#endif
		0x08
	};
	unsigned int _osver = 0;
	unsigned int _osversion = 0;
	unsigned int _osmajor = 0;
	unsigned int _osminor = 0;
	unsigned int _baseosversion = 0;
	unsigned int _baseosmajor = 0;
	unsigned int _baseosminor = 0;
	unsigned int _winver = 0;
	unsigned int _winmajor = 0;
	unsigned int _winminor = 0;
	unsigned int *_osver_dll = &_osver;
	unsigned int *_osversion_dll = &_osversion;
	unsigned int *_osmajor_dll = &_osmajor;
	unsigned int *_osminor_dll = &_osminor;
	unsigned int *_winver_dll = &_winver;
	unsigned int *_winmajor_dll = &_winmajor;
	unsigned int *_winminor_dll = &_winminor;
	unsigned int * _baseosversion_dll = &_baseosversion;
	unsigned int * _baseosmajor_dll = &_baseosmajor;
	unsigned int * _baseosminor_dll = &_baseosminor;
	         int _fmode;          
		     int _fileinfo;      
             char *_pgmptr = NULL;     
		    wchar_t *_wpgmptr = NULL;  
			 int *_fmode_dll = &_fmode;
			 int *_fileinfo_dll = &_fileinfo;
			 char **_pgmptr_dll = &_pgmptr;
			wchar_t **_wpgmptr_dll = &_wpgmptr;
			 char **_environ = NULL;
			wchar_t **_wenviron = NULL;
			 int _sys_nerr;         
			 int _mb_cur_max = -1;
			 int* _mb_cur_max_dll = &_mb_cur_max;
			 int __argc; 
			 char **__argv;         
			 char **__initenv;
			wchar_t **__winitenv;
			wchar_t ** __wargv;     
			long _timezone = 8 * 3600L; 
			int _daylight = 1;         
			long _dstbias = -3600L;     
			long *_timezone_dll = &_timezone;
			int *_daylight_dll = &_daylight;
			long *_dstbias_dll = &_dstbias;

#ifdef __cplusplus
}
#endif