#ifndef __HUAWEI_PLANCK_EC_H__
#define __HUAWEI_PLANCK_EC_H__

struct planck_ec;
struct notifier_block;

#define PLANCK_MOD_NAME			"huawei_planck_ec"
#define PLANCK_DEV_PSY			"psy"
#define PLANCK_DEV_UCSI			"ucsi"

enum planck_ec_transfer_operation {
	PLANCK_EC_READ_EVENT,
	PLANCK_EC_READ_COMMON,
	PLANCK_EC_READ_UCSI,
	PLANCK_EC_WRITE_BACKLIGHT,
	PLANCK_EC_WRITE_UCSI
};

int planck_ec_transfer(struct planck_ec *ec, u8 cmd,
					   u8 *data, u8 data_len, enum planck_ec_transfer_operation op);

int planck_ec_register_notify(struct planck_ec *ec, struct notifier_block *nb);
void planck_ec_unregister_notify(struct planck_ec *ec, struct notifier_block *nb);

#endif
