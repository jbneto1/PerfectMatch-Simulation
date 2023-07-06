set(LOCALIZATION_SRC
        Localization.cpp
        PerfectMatch/PerfectMatch.cpp
        ExtendedKalmanFilter/ExtendedKalmanFilter.cpp
        PerfectMatch/Map/Map.h PerfectMatch/Map/Map.cpp)

add_library(Localization ${LOCALIZATION_SRC})
