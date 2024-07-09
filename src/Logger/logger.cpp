// Logger.cpp
#include "logger.h"

Logger::Logger(spdlog::level::level_enum level)
{
    std::vector<spdlog::sink_ptr> sinks;
    try
    {
        sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
        logger = std::make_shared<spdlog::logger>("logger", begin(sinks), end(sinks));
        logger->set_level(level);
        this->trace("Console colored logger initialized.");
    }
    catch (const spdlog::spdlog_ex &ex)
    {
        std::cerr << "Console Logger initialization failed: " << ex.what() << '\n';
    }
    catch (const std::exception &ex)
    {
        std::cerr << "Console Logger initialization failed. General exception: " << ex.what() << '\n';
    }
}

// Logger::Logger(spdlog::level::level_enum level)
// {
//     std::vector<spdlog::sink_ptr> sinks;
//     try
//     {
//         // Create and add a file sink instead of a console sink
//         auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("logfile.txt", true); // 'true' for appending to file
//         sinks.push_back(file_sink);

//         logger = std::make_shared<spdlog::logger>("logger", begin(sinks), end(sinks));
//         logger->set_level(level);
//         this->trace("File logger initialized."); // Log message indicating file logger setup
//     }
//     catch (const spdlog::spdlog_ex &ex)
//     {
//         std::cerr << "File Logger initialization failed: " << ex.what() << '\n';
//     }
//     catch (const std::exception &ex)
//     {
//         std::cerr << "File Logger initialization failed. General exception: " << ex.what() << '\n';
//     }
// }

void Logger::createOnlineLoggers()
{
    try
    {
        auto filename_data = fmt::format("../docs/logs/sensor_data_{}.txt", current_datetime());
        auto file_sink_data = std::make_shared<spdlog::sinks::basic_file_sink_mt>(filename_data, true);
        fileLogger_data = std::make_shared<spdlog::logger>("SensorData", file_sink_data);
        fileLogger_data->set_level(spdlog::level::trace);
        fileLogger_data->set_pattern(std::string("%v"));
    }
    catch (const spdlog::spdlog_ex &ex)
    {
        logger->error("Online logger initialization failed: " + std::string(ex.what()));
    }
    catch (const std::exception &ex)
    {
        logger->error("Online logger initialization failed. General Exception: " + std::string(ex.what()));
    }
}

void Logger::createOfflineLoggers()
{
    std::ostringstream oss;
    oss << "EKF_x"
        << ","
        << "EKF_y"
        << ","
        << "EKF_theta"
        << ","
        << "PM_x"
        << ","
        << "PM_y"
        << ","
        << "PM_theta"
        << ","
        << "errorEKF_x"
        << ","
        << "errorEKF_y"
        << ","
        << "errorEKF_theta"
        << ","
        << "errorPM"
        << ","
        << "EKFCovXX"
        << ","
        << "EKFCovXY"
        << ","
        << "EKFCovXTheta"
        << ","
        << "EKFCovYX"
        << ","
        << "EKFCovYY"
        << ","
        << "EKFCovYTheta"
        << ","
        << "EKFCovThetaX"
        << ","
        << "EKFCovThetaY"
        << ","
        << "EKFCovThetaTheta";

    try
    {
        auto filename_data = fmt::format("../docs/logs/logs_offlineAnalysis/offlineAnalysis_{}.txt", current_datetime());
        auto file_sink_data = std::make_shared<spdlog::sinks::basic_file_sink_mt>(filename_data, true);
        fileLogger_offline = std::make_shared<spdlog::logger>("OfflineAnalysis", file_sink_data);
        fileLogger_offline->set_level(spdlog::level::trace);
        fileLogger_offline->set_pattern(std::string("%v"));
        fileLogger_offline->trace(oss.str());

        oss << ","
            << "counter";

        auto filename_data_semantics = fmt::format("../docs/logs/logs_offlineAnalysis/offlineAnalysis_w_semantics{}.txt", current_datetime());
        auto file_sink_data_semantics = std::make_shared<spdlog::sinks::basic_file_sink_mt>(filename_data_semantics, true);
        fileLogger_offline_w_semantics = std::make_shared<spdlog::logger>("OfflineAnalysis_w_semantics", file_sink_data_semantics);
        fileLogger_offline_w_semantics->set_level(spdlog::level::trace);
        fileLogger_offline_w_semantics->set_pattern(std::string("%v"));
        fileLogger_offline_w_semantics->trace(oss.str());
    }
    catch (const spdlog::spdlog_ex &ex)
    {
        logger->error("Offline Loggers initialization failed: " + std::string(ex.what()));
    }
    catch (const std::exception &ex)
    {
        logger->error("Offline Loggers initialization failed. General Exception: " + std::string(ex.what()));
    }
}

Logger &Logger::getInstance(spdlog::level::level_enum level)
{
    static Logger instance = Logger(level);
    return instance;
}

std::string Logger::current_datetime()
{
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
#if defined(__unix__)
    localtime_r(&time, &tm); // POSIX
#elif defined(_MSC_VER)
    localtime_s(&tm, &time); // MSVC
#endif
    std::stringstream ss;
    ss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    return ss.str();
}

std::string Logger::getHighPrecisionTimestamp()
{
    auto now = std::chrono::system_clock::now();
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
    auto epoch = now_ms.time_since_epoch();
    auto duration = epoch.count();

    std::stringstream ss;
    ss << duration;
    return ss.str();
}

void Logger::trace(const std::string &message)
{
    try
    {
        logger->trace(message);
    }
    catch (const spdlog::spdlog_ex &ex)
    {
        std::cout << "Log failed: " << ex.what() << std::endl;
    }
}

void Logger::debug(const std::string &message)
{
    try
    {
        logger->debug(message);
    }
    catch (const spdlog::spdlog_ex &ex)
    {
        std::cout << "Log failed: " << ex.what() << std::endl;
    }
}

void Logger::info(const std::string &message)
{
    try
    {
        logger->info(message);
    }
    catch (const spdlog::spdlog_ex &ex)
    {

        std::cout << "Log failed: " << ex.what() << std::endl;
    }
}

void Logger::warn(const std::string &message)
{
    try
    {
        logger->warn(message);
    }
    catch (const spdlog::spdlog_ex &ex)
    {
        std::cout << "Log failed: " << ex.what() << std::endl;
    }
}

void Logger::error(const std::string &message)
{
    try
    {
        logger->error(message);
    }
    catch (const spdlog::spdlog_ex &ex)
    {
        std::cout << "Log failed: " << ex.what() << std::endl;
    }
}

void Logger::set_level(const spdlog::level::level_enum log_level)
{
    logger->set_level(log_level);
}

void Logger::setPattern(const std::string &format)
{
    logger->set_pattern(format);
}

void Logger::deactivate_Loggers()
{
    logger->set_level(spdlog::level::off);
    fileLogger_data->set_level(spdlog::level::off);
}

void Logger::fileLog_bag(const std::array<int, 4UL> &encs, const Pose &GT_pose,
                         const std::optional<std::vector<LaserPoint>> &laserReadings, const std::string &yoloData)
{
    std::ostringstream oss;
    std::string now_c;

    now_c = getHighPrecisionTimestamp();

    oss << GT_pose.getX() << ',' << GT_pose.getY() << ',' << GT_pose.getTheta() << ',';

    oss << encs[0] << ',' << encs[1] << ',' << encs[2] << ',' << encs[3];

    if (laserReadings.has_value())
    {

        for (const auto &reading : laserReadings.value())
        {
            oss << ',' << reading.getD();
        }
    }

    if (!yoloData.empty())
    {
        oss << ',' << yoloData;
    }

    oss << ',' << now_c;

    this->fileLogger_data->trace(oss.str());
}

void Logger::fileLog_offlineAnalysis(const std::tuple<Pose, Pose, Pose, double, Matrix3d> &localization,
                                     const std::tuple<Pose, Pose, Pose, double, Matrix3d, u_int> &localization_w_semantics)
{
    /* It is assumed that although the windows scheduler makes the reception of datagrams of simtwo not constant at 40Hz, the internal clock
    of simtwo guarantees a constant 40 Hz simulation period. Thus, the measurements happens at their due frequencies. Therefore,
    The timestamp of the graphs will be assuming 25ms each data log line.*/
    std::ostringstream oss, oss_semantics;

    {
        auto [EKFPose, PMPose, EKFerror, PMError, EKFCov] = localization;

        oss << EKFPose.getX()
            << ","
            << EKFPose.getY()
            << ","
            << EKFPose.getTheta()
            << ","
            << PMPose.getX()
            << ","
            << PMPose.getY()
            << ","
            << PMPose.getTheta()
            << ","
            << EKFerror.getX()
            << ","
            << EKFerror.getY()
            << ","
            << EKFerror.getTheta()
            << ","
            << PMError
            << ","
            << EKFCov(0, 0)
            << ","
            << EKFCov(0, 1)
            << ","
            << EKFCov(0, 2)
            << ","
            << EKFCov(1, 0)
            << ","
            << EKFCov(1, 1)
            << ","
            << EKFCov(1, 2)
            << ","
            << EKFCov(2, 0)
            << ","
            << EKFCov(2, 1)
            << ","
            << EKFCov(2, 2);
    }

    fileLogger_offline->trace(oss.str());

    {
        auto [EKFPose, PMPose, EKFerror, PMError, EKFCov, counter] = localization_w_semantics;

        oss_semantics << EKFPose.getX()
                      << ","
                      << EKFPose.getY()
                      << ","
                      << EKFPose.getTheta()
                      << ","
                      << PMPose.getX()
                      << ","
                      << PMPose.getY()
                      << ","
                      << PMPose.getTheta()
                      << ","
                      << EKFerror.getX()
                      << ","
                      << EKFerror.getY()
                      << ","
                      << EKFerror.getTheta()
                      << ","
                      << PMError
                      << ","
                      << EKFCov(0, 0)
                      << ","
                      << EKFCov(0, 1)
                      << ","
                      << EKFCov(0, 2)
                      << ","
                      << EKFCov(1, 0)
                      << ","
                      << EKFCov(1, 1)
                      << ","
                      << EKFCov(1, 2)
                      << ","
                      << EKFCov(2, 0)
                      << ","
                      << EKFCov(2, 1)
                      << ","
                      << EKFCov(2, 2)
                      << ","
                      << counter;
    }

    fileLogger_offline_w_semantics->trace(oss_semantics.str());
}
