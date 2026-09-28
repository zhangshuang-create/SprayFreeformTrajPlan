#pragma once

#if defined _cplusplus
extern "C" {
#endif  


#ifdef _WIN32
    #ifdef ProcessOptimizationDialog_EXPORTS
        #define ProcessOptimizationDialog_SHAREDLIB __declspec(dllexport)
    #else
        #define ProcessOptimizationDialog_SHAREDLIB __declspec(dllimport)
    #endif
#else
    #define ProcessOptimizationDialog_SHAREDLIB
#endif  

#ifdef _cplusplus
}
#endif