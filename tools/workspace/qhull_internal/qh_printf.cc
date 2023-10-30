#include <stdarg.h>

#include <exception>

#include "libqhull_r/libqhull_r.h"

extern "C" void qh_fprintf(qhT* qh, FILE* fp, int msgcode, const char* fmt,
                           ...) {
  if (qh == nullptr || !qh->ISqhullQh) {
    std::terminate();
  }
  QhullQh* qhullQh = static_cast<QhullQh*>(qh);
  char message[MSG_MAXLEN];
  int length = 0;
  length = snprintf(message, sizeof(message), "QH%.4d ", msgcode);
  vsnprintf(message + length, sizeof(message) - length, fmt, args);
  if (msgcode < MSG_OUTPUT || fp == qh_FILEstderr) {
    if (msgcode >= MSG_ERROR && msgcode < MSG_WARNING) {
      qh->last_errcode = msgcode;
      if (qhullQh->qhull_status < MSG_ERROR ||
          qhullQh->qhull_status >= MSG_WARNING) {
        qhullQh->qhull_status = msgcode;
      }
    }
  }
  qhullQh->appendQhullMessage(message);
}
