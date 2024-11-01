#ifndef MULTITECH_LORAWAN_CONFIG_H_
#define MULTITECH_LORAWAN_CONFIG_H_

struct perisistent_config {
	uint32_t magic;
	uint8_t deveui[8];
	uint8_t joineui[8];
	uint8_t appkey[16];
	uint16_t devnonce;
};

int init_config();
int save_config();


#endif