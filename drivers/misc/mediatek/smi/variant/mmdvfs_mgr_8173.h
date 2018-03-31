#ifndef __MMDVFS_MGR_H__
#define __MMDVFS_MGR_H__

#include "aee.h"


#define MMDVFS_LOG_TAG	"MMDVFS"

#define MMDVFSMSG(string, args...) \
	pr_err("[MMDVFS][pid=%d]"string, current->tgid, ##args)

#define MMDVFSERR(string, args...) do {\
	pr_err("[MMDVFS]error: "string, ##args); \
	aee_kernel_warning(MMDVFS_LOG_TAG, "error: "string, ##args);\
} while (0)


/* screen size */
extern unsigned int DISP_GetScreenWidth(void);
extern unsigned int DISP_GetScreenHeight(void);

extern void mmdvfs_init(MTK_SMI_BWC_MM_INFO *info);
extern void mmdvfs_notify_scenario_enter(MTK_SMI_BWC_SCEN scen);
extern void mmdvfs_notify_scenario_exit(MTK_SMI_BWC_SCEN scen);
extern void mmdvfs_notify_scenario_concurrency(unsigned int u4Concurrency);
extern void mmdvfs_handle_cmd(MTK_MMDVFS_CMD *cmd);

#endif				/* __MMDVFS_MGR_H__ */
