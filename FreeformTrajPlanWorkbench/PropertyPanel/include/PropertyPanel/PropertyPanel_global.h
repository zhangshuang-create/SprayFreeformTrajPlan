#pragma once

#ifdef PROPERTYPANEL_LIB
# define PropertyPanel_SHAREDLIB __declspec(dllexport)
#else
# define PropertyPanel_SHAREDLIB __declspec(dllimport)
#endif
