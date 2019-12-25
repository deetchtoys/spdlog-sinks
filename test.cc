#include <iostream>
#include <memory>

#include <spdlog/spdlog.h>
#include "rsyslog_sink.h"

using namespace std;

int main()
{
  std::shared_ptr<spdlog::logger> logger = spdlog::rsyslog_logger_mt("my-logger-name",
                                                                     "rsyslog-ident",
                                                                     "172.16.20.23",
                                                                     "local6",
                                                                     "info");
  logger->set_pattern("[%Y-%m-%d %H:%M:%S:%e] [%n] [%l] [%P] %@ : %v");
  logger->flush_on(spdlog::level::info);
  SPDLOG_LOGGER_INFO(logger, "this is my first log with spdlog");

  return 0;
}
