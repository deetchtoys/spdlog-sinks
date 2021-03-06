// Copyright(c) 2015-present, Gabi Melman & spdlog contributors.
// Distributed under the MIT License (http://opensource.org/licenses/MIT)

#pragma once

#include <syslog.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <array>
#include <string>
#include <map>
#include <sstream>

#include "spdlog/sinks/base_sink.h"
#include "spdlog/details/null_mutex.h"
#include "spdlog/common.h"
#include "spdlog/details/synchronous_factory.h"

namespace spdlog {
namespace sinks {
/**
 * Sink that write to rsyslog using udp.
 */
template<typename Mutex>
class rsyslog_sink final : public base_sink<Mutex>
{
  std::map<std::string, uint> severities_ = {
    { "alert", LOG_ALERT },
    { "crit", LOG_CRIT },
    { "debug", LOG_DEBUG },
    { "emerg", LOG_EMERG },
    { "err", LOG_ERR },
    { "info", LOG_INFO },
    { "notice", LOG_NOTICE },
    { "warning", LOG_WARNING },
  };
  std::map<std::string, uint> facilities_ = {
    { "auth", LOG_AUTH },
    { "authpriv", LOG_AUTHPRIV },
    { "cron", LOG_CRON },
    { "daemon", LOG_DAEMON },
    { "ftp", LOG_FTP },
    { "kern", LOG_KERN },
    { "lpr", LOG_LPR },
    { "mail", LOG_MAIL },
    { "news", LOG_NEWS },
    { "syslog", LOG_SYSLOG },
    { "user", LOG_USER },
    { "uucp", LOG_UUCP },
    { "local0", LOG_LOCAL0 },
    { "local1", LOG_LOCAL1 },
    { "local2", LOG_LOCAL2 },
    { "local3", LOG_LOCAL3 },
    { "local4", LOG_LOCAL4 },
    { "local5", LOG_LOCAL5 },
    { "local6", LOG_LOCAL6 },
    { "local7", LOG_LOCAL7 },
  };
  const int log_buffer_max_size_;
  struct sockaddr_in sockaddr_;
  int log_fd_;
  std::string log_buffer_;
  std::string log_header_;

 public:
  rsyslog_sink(const std::string &ident,
               const std::string &rsyslog_ip,
               const std::string &facility,
               const std::string &severity,
               int log_buffer_max_size,
               int port,
               bool enable_formatting)
      : enable_formatting_(enable_formatting)
      , log_buffer_max_size_(log_buffer_max_size)
  {
    if (facilities_.find(facility) == facilities_.end() ||
        severities_.find(severity) == severities_.end())
    {
      SPDLOG_THROW(spdlog_ex("invalid facility or severity"));
      return;
    }

    if (log_buffer_max_size_ > std::numeric_limits<int>::max())
    {
      SPDLOG_THROW(spdlog_ex("too large maxLogSize"));
      return;
    }

    std::stringstream ss;
    // <%u>%s:
    ss << "<" << facilities_[facility] + severities_[severity] << ">" << ident << ": ";
    log_header_ = ss.str();
    log_buffer_.reserve(log_buffer_max_size_);
    memset(&sockaddr_, 0, sizeof(sockaddr_));
    sockaddr_.sin_family = AF_INET;
    sockaddr_.sin_port = htons(port);
    inet_pton(AF_INET, rsyslog_ip.c_str(), &sockaddr_.sin_addr);
    InitLogSocket();
  }

  ~rsyslog_sink() override
  {
    close(log_fd_);
  }

  rsyslog_sink(const rsyslog_sink &) = delete;
  rsyslog_sink &operator=(const rsyslog_sink &) = delete;

 protected:
  void sink_it_(const details::log_msg &msg) override
  {
    string_view_t payload;
    memory_buf_t formatted;
    if (enable_formatting_)
    {
      base_sink<Mutex>::formatter_->format(msg, formatted);
      payload = string_view_t(formatted.data(), formatted.size());
    }
    else
    {
      payload = msg.payload;
    }
    size_t length = payload.size();
    // limit to max int
    length = length > static_cast<size_t>(std::numeric_limits<int>::max()) ? static_cast<size_t>(std::numeric_limits<int>::max()) : length;
    log_buffer_ += log_header_;
    length = length > log_buffer_max_size_ - log_buffer_.size() ? log_buffer_max_size_ - log_buffer_.size() : length;
    if (length > 0)
    {
      log_buffer_.append(payload.data(), length);
    }
    size_t sendSize = write(log_fd_, log_buffer_.c_str(), log_buffer_.size());
    log_buffer_.clear();
  }

  void flush_() override {}
  bool enable_formatting_ = false;

 private:
  void InitLogSocket()
  {
    if ((log_fd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
      SPDLOG_THROW(spdlog_ex("failed create socket"));
      return;
    }

    int nb;
    nb = 1;
    if (ioctl(log_fd_, FIONBIO, &nb) == -1)
    {
      SPDLOG_THROW(spdlog_ex("failed ioctl socket FIONBIO"));
      return;
    }

    if (connect(log_fd_, reinterpret_cast<struct sockaddr *>(&sockaddr_), sizeof(sockaddr_)) < 0)
    {
      SPDLOG_THROW(spdlog_ex("failed connect socket"));
      return;
    }
  }
};

using rsyslog_sink_mt = rsyslog_sink<std::mutex>;
using rsyslog_sink_st = rsyslog_sink<details::null_mutex>;
} // namespace sinks

// Create and register a syslog logger
template<typename Factory = synchronous_factory>
inline std::shared_ptr<logger> rsyslog_logger_mt(const std::string &logger_name,
                                                 const std::string &ident,
                                                 const std::string &rsyslog_ip,
                                                 const std::string &facility,
                                                 const std::string &severity,
                                                 int log_buffer_max_size = 1024 * 1024 * 16,
                                                 int port = 514,
                                                 bool enable_formatting = true)
{
  return Factory::template create<sinks::rsyslog_sink_mt>(logger_name, ident, rsyslog_ip, facility, severity, log_buffer_max_size, port, enable_formatting);
}

template<typename Factory = synchronous_factory>
inline std::shared_ptr<logger> rsyslog_logger_st(const std::string &logger_name,
                                                 const std::string &ident,
                                                 const std::string &rsyslog_ip,
                                                 const std::string &facility,
                                                 const std::string &severity,
                                                 int log_buffer_max_size = 1024 * 1024 * 16,
                                                 int port = 514,
                                                 bool enable_formatting = true)
{
  return Factory::template create<sinks::rsyslog_sink_st>(logger_name, ident, rsyslog_ip, facility, severity, log_buffer_max_size, port, enable_formatting);
}

} // namespace spdlog
