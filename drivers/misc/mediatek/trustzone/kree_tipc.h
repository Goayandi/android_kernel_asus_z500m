
#ifndef __KREE_TIPC_H
#define __KREE_TIPC_H

#include <linux/types.h>

typedef void *tipc_k_handle;
int tipc_k_connect(tipc_k_handle *h, const char *port);
int tipc_k_disconnect(tipc_k_handle h);
ssize_t tipc_k_read(tipc_k_handle h, void *buf, size_t buf_len, unsigned int flags);
ssize_t tipc_k_write(tipc_k_handle h, void *buf, size_t len, unsigned int flags);

#endif
