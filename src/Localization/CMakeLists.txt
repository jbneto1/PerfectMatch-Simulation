set(LOCALIZATION_SRC
        Localization.cpp
        PerfectMatch/PerfectMatch.cpp
        ExtendedKalmanFilter/ExtendedKalmanFilter.cpp
        )

add_library(Localization ${LOCALIZATION_SRC})
