#pragma once

#if defined _cplusplus
extern "C" {
#endif  


#ifdef _WIN32
    #ifdef SlicingWidgets_EXPORTS
        #define SlicingWidgets_SHAREDLIB __declspec(dllexport)
    #else
        #define SlicingWidgets_SHAREDLIB __declspec(dllimport)
    #endif
#else
    #define SlicingWidgets_SHAREDLIB
#endif  

#ifdef _cplusplus
}
#endif