#include <linux/fs.h>
#include <linux/random.h>

#define WIFI_MAC_ADDR_FILE	"/persist/wifi/mac.txt"
#define WIFI_RAN_MAC_ADDR_FILE	"/data/misc/wifi/mac.txt"

int check_mac(char *str)
{
	int i;

	if (strlen(str) != 12) {
		pr_err("%s: bad mac address file len %zu < 12\n",
				__func__, strlen(str));
		return -1;
	}
	for (i = 0; i < strlen(str); i++) {
		if (!strchr("1234567890abcdefABCDEF", str[i])) {
			pr_err("%s: illegal wifi mac\n", __func__);
			return -1;
		}
	}
	return 0;
}

void string_to_mac(char *str, unsigned char *buf)
{
	char temp[3]="\0";
	int mac[6];
	int i;

	for (i = 0; i < 6; i++) {
		strncpy(temp, str+(i*2), 2);
		sscanf(temp, "%x", &mac[i]);
	}
	pr_info("%s: using wifi mac %02x:%02x:%02x:%02x:%02x:%02x\n",
		__func__, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

	buf[0] = (unsigned char) mac[0];
	buf[1] = (unsigned char) mac[1];
	buf[2] = (unsigned char) mac[2];
	buf[3] = (unsigned char) mac[3];
	buf[4] = (unsigned char) mac[4];
	buf[5] = (unsigned char) mac[5];
}

#define ETHER_ADDR_LEN 6

int platform_wifi_get_mac_addr(unsigned char *buf)
{
	struct file *fp;
	char str[32];
	char temp[4]="\0";
	int i;
	uint rand_mac;
	//uint rand_mac;

	pr_debug("%s\n", __func__);

	memset(buf, 0x00, ETHER_ADDR_LEN);

	/* open wifi mac address file */
	fp = filp_open(WIFI_MAC_ADDR_FILE, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		pr_err("%s: cannot open %s\n", __func__, WIFI_MAC_ADDR_FILE);
		/* open wifi tmp mac address file*/
		fp = filp_open(WIFI_RAN_MAC_ADDR_FILE, O_RDONLY, 0);
		if (IS_ERR(fp)) {
			pr_err("%s: cannot open %s\n", __func__, WIFI_RAN_MAC_ADDR_FILE);
			goto random_mac;
		}
	} else {
		fp->f_pos += strlen("MacAddress0=");
	}

	/* read wifi mac address file */
	memset(str, 0, sizeof(str));
	kernel_read(fp, fp->f_pos, str, 12);

	if (check_mac(str)) {
		filp_close(fp, NULL);
		goto random_mac;
	}
	string_to_mac(str, buf);
	filp_close(fp, NULL);
	return 0;

random_mac:
	/* random create wifi mac address */

	prandom_seed((uint)jiffies);
	rand_mac = prandom_u32();
	buf[0] = 0x00;
	buf[1] = 0x08;
	buf[2] = 0x22;
	buf[3] = (unsigned char)rand_mac;
	buf[4] = (unsigned char)(rand_mac >> 8);
	buf[5] = (unsigned char)(rand_mac >> 16);


	memset(str, 0, sizeof(str));

	for(i=0; i<6; i++) {
		sprintf(temp, "%02x", buf[i]);
		strcat(str, temp);
	}

	fp = filp_open(WIFI_RAN_MAC_ADDR_FILE, O_CREAT | O_RDWR, 0644);

	if (IS_ERR(fp)) {
		pr_err("%s: cannot create %s\n", __func__, WIFI_RAN_MAC_ADDR_FILE);
	} else {
		pr_err("%s: created %s\n", __func__, WIFI_RAN_MAC_ADDR_FILE);
		fp->f_op->write(fp, str, sizeof(str), &fp->f_pos);
		filp_close(fp, NULL);
	}
	return 0;
}
