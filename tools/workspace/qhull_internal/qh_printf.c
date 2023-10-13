#include <stdarg.h>

#include "libqhull_r/libqhull_r.h"

void qh_fprintf(qhT* qh, FILE* fp, int msgcode, const char* fmt, ...) {
  fprintf(fp, "QH%.5d ", msgcode);
  va_list args;
  va_start(args, fmt);
  vfprintf(fp, fmt, args);
  va_end(args);
  if (qh) {
    if (msgcode >= MSG_ERROR && msgcode < MSG_WARNING)
      qh->last_errcode = msgcode;
  }
}
