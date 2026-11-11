#pragma once
#include <cstdio>
extern int g_log_errors;
#define ESP_LOGE(tag, ...) \
  do { \
    g_log_errors++; \
  } while (0)
#define ESP_LOGW(tag, ...) \
  do { \
  } while (0)
