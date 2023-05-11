/*
 * Copyright(c)2018 ChangSha XingShen Technology Ltd.
 *
 * All rights reserved
 * Author:    wanghao
 *-------------------------------
 *Changes:
 *-------------------------------
 * v1.0 2018-06-28 :created by wanghao
 *
 */

#ifndef __common_struct__
#define __common_struct__

#include <arpa/inet.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <pwd.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/prctl.h>
#include <sys/sem.h>
#include <sys/shm.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include <boost/cstdint.hpp>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <queue>
#include <string>

class Timer {
 public:
  inline void SetStart();

  inline uint64_t GetElapsedMinute();

  inline uint64_t GetElapsedSecond();

  inline uint64_t GetElapsedMillisecond();

  inline uint64_t GetElapsedMicrosecond();

 private:
  struct timeval _start_time;
};

inline void Timer::SetStart() { gettimeofday(&_start_time, NULL); }

inline uint64_t Timer::GetElapsedMinute() {
  struct timeval _end_time;
  gettimeofday(&_end_time, NULL);
  return ((_end_time.tv_sec - _start_time.tv_sec) +
          (_end_time.tv_usec - _start_time.tv_usec) / 1000000) /
         60;
}

inline uint64_t Timer::GetElapsedSecond() {
  struct timeval _end_time;
  gettimeofday(&_end_time, NULL);
  return _end_time.tv_sec - _start_time.tv_sec +
         (_end_time.tv_usec - _start_time.tv_usec) / 1000000;
}

inline uint64_t Timer::GetElapsedMillisecond() {
  struct timeval _end_time;
  gettimeofday(&_end_time, NULL);
  return 1000 * (_end_time.tv_sec - _start_time.tv_sec) +
         (_end_time.tv_usec - _start_time.tv_usec) / 1000;
}

inline uint64_t Timer::GetElapsedMicrosecond() {
  struct timeval _end_time;
  gettimeofday(&_end_time, NULL);
  return 1000000 * (_end_time.tv_sec - _start_time.tv_sec) +
         (_end_time.tv_usec - _start_time.tv_usec);
}

#endif
