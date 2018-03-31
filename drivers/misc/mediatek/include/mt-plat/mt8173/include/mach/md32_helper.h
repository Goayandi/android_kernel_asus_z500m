#ifndef __MD32_HELPER_H__
#define __MD32_HELPER_H__

extern struct md32_regs md32reg;

enum SEMAPHORE_FLAG {
	SEMAPHORE_CLK_CFG_5 = 0,
	SEMAPHORE_PTP,
	NR_FLAG = 8,
};

void __iomem *md32_get_dtcm(void);
int get_md32_semaphore(int flag);
int release_md32_semaphore(int flag);
#endif
