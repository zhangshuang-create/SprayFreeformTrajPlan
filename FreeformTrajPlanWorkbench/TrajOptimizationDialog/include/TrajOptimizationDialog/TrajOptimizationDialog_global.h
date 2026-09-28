#pragma once

#if defined _cplusplus
extern "C" {
#endif  


#ifdef _WIN32
    #ifdef TrajOptimizationDialog_EXPORTS
        #define TrajOptimizationDialog_SHAREDLIB __declspec(dllexport)
    #else
        #define TrajOptimizationDialog_SHAREDLIB __declspec(dllimport)
    #endif
#else
    #define TrajOptimizationDialog_SHAREDLIB
#endif  

#ifdef _cplusplus
}
#endif