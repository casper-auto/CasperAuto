/**
 * @file log.h
 * @brief Debug print
 */

#ifndef LOG_H
#define LOG_H

#define DEBUG
#ifdef DEBUG
#include <iostream>
#define DEBUG_MSG(str)                                                         \
  do {                                                                         \
    std::cout << str << std::endl;                                             \
  } while (false)
#else
#define DEBUG_MSG(str)                                                         \
  do {                                                                         \
  } while (false)
#endif

#endif // LOG_H
