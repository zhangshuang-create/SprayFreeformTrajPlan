#pragma once

#ifdef RIBBONWIDGET_LIB
# define RibbonWidget_SHAREDLIB __declspec(dllexport)
#else
# define RibbonWidget_SHAREDLIB __declspec(dllimport)
#endif
