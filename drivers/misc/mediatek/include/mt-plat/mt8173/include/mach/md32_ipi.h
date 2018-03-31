#ifndef __MD32_IPI_H
#define __MD32_IPI_H

enum ipi_id {
	IPI_WDT = 0,
	IPI_TEST1,
	IPI_TEST2,
	IPI_LOGGER,
	IPI_SWAP,
	IPI_ANC,
	IPI_SPK_PROTECT,
	IPI_THERMAL,
	IPI_SPM,
	IPI_DVT_TEST,
	IPI_BUF_FULL,
	IPI_VCORE_DVFS,
	MD32_NR_IPI,
};

enum ipi_status {
	ERROR = -1,
	DONE,
	BUSY,
};

typedef void (*ipi_handler_t) (int id, void *data, unsigned int len);

enum ipi_status md32_ipi_registration(enum ipi_id id, ipi_handler_t handler,
				      const char *name);
enum ipi_status md32_ipi_send(enum ipi_id id, void *buf, unsigned int  len,
			      unsigned int wait);
#endif	/* __MD32_IPI_H */
