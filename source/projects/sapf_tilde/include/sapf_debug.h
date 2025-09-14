#ifndef SAPF_DEBUG_H
#define SAPF_DEBUG_H

#ifdef SAPF_TILDE
#include "ext.h"
#include "ext_obex.h"

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define sapf_debug(func_name) post("SAPF %s[%d]: %s", __FILENAME__, __LINE__, func_name)

#endif

#endif // SAPF_DEBUG_H
