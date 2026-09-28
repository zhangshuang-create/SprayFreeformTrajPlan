#pragma once

#if defined _cplusplus
extern "C" {
#endif  


#ifdef _WIN32
    #ifdef BuringTrajDialog_EXPORTS
        #define BuringTrajDialog_SHAREDLIB __declspec(dllexport)
    #else
        #define BuringTrajDialog_SHAREDLIB __declspec(dllimport)
    #endif
#else
    #define BuringTrajDialog_SHAREDLIB
#endif  

#ifdef _cplusplus
}
#endif