#include "asus_fp_id.h"
#include <linux/kernel.h>
#include <linux/types.h>

char of_get_pcbid_data(void)
{
	struct device_node *np;
	const char *pcbidstr;
	char pcbid;

	np = of_find_compatible_node(NULL, NULL, "android,firmware");
	if (np == NULL) {
		pr_err("get pcbid failed\n");
		return np;
	}

	pcbidstr = of_get_property(np, "fp_id", NULL);
	if (pcbidstr == NULL) {
		pr_err("get fp pcbid failed\n");
		return pcbidstr;
	} else {
		pcbid = *pcbidstr;
		pr_info("fp_id read %c\n", pcbid);
	}

	return pcbid;
}
