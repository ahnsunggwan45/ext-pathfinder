#ifndef PHP_PATHFINDER_H
#define PHP_PATHFINDER_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

extern "C" {
#include "php.h"
}

extern zend_module_entry pathfinder_module_entry;
#define phpext_pathfinder_ptr &pathfinder_module_entry

#define PHP_PATHFINDER_VERSION  "0.1.0"
#define PHP_PATHFINDER_EXTNAME  "pathfinder"

#if defined(ZTS) && defined(COMPILE_DL_PATHFINDER)
ZEND_TSRMLS_CACHE_EXTERN()
#endif

#endif /* PHP_PATHFINDER_H */
