#pragma once

#if defined _cplusplus
extern "C" {
#endif


#ifdef _WIN32
    #ifdef SlicingAdjustDialog_EXPORTS
        #define SlicingAdjustDialog_SHAREDLIB __declspec(dllexport)
    #else
        #define SlicingAdjustDialog_SHAREDLIB __declspec(dllimport)
    #endif
#else
    #define SlicingAdjustDialog_SHAREDLIB
#endif

#ifdef _cplusplus
}
#endif