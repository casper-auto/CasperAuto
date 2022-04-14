/**
 * @file status.h
 * @brief The class of Status.
 **/

#ifndef STATUS_H
#define STATUS_H

#include <string>

namespace planning {

/**
 * @class Status
 *
 * @brief A general class to denote the return status of an API call. It
 * can either be an OK status for success, or a failure status with error
 * message.
 */
class Status {
public:
  /**
   * @brief Create a success status.
   */
  Status() : code_("Success"), msg_() {}
  ~Status() = default;

  /**
   * @brief Create a status with the specified status code and msg as a
   * human-readable string containing more detailed information.
   * @param code the status code.
   * @param msg the message associated with the error.
   */
  Status(std::string code, const std::string &msg) : code_(code), msg_(msg) {}

  /**
   * @brief Create a status with the specified status code and empty msg
   * @param code the status code.
   */
  explicit Status(std::string code) : code_(code), msg_("") {}

  /**
   * @brief generate a success status.
   * @returns a success status
   */
  static Status OK() { return Status(); }

  /**
   * @brief check whether the return status is OK.
   * @returns true if the code is ErrorCode::OK
   *          false otherwise
   */
  bool ok() const { return code_ == "Success"; }

  /**
   * @brief get the status code
   * @returns the status code
   */
  std::string s() const { return code_; }

  /**
   * @brief defines the logic of testing if two Status are equal
   */
  bool operator==(const Status &rh) const {
    return (this->code_ == rh.code_) && (this->msg_ == rh.msg_);
  }

  /**
   * @brief defines the logic of testing if two Status are unequal
   */
  bool operator!=(const Status &rh) const { return !(*this == rh); }

  /**
   * @brief returns the error message of the status, empty if the status is OK.
   * @returns the error message
   */
  const std::string &error_message() const { return msg_; }

  /**
   * @brief returns a string representation in a readable format.
   * @returns the string "OK" if success.
   *          the internal error message otherwise.
   */
  std::string ToString() const {
    if (ok()) {
      return "OK";
    }
    return code_ + ": " + msg_;
  }

private:
  std::string code_;
  std::string msg_;
};

inline std::ostream &operator<<(std::ostream &os, const Status &s) {
  os << s.ToString();
  return os;
}

} // namespace planning

#endif // STATUS_H
