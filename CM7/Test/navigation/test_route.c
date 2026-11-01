#include "test_check.h"
#include "navigationRoute.h"

// CRC-16/CCITT-FALSE check value. The ground station's binascii.crc_hqx(b"123456789", 0xFFFF)
// must give the same, or no route will ever commit.
static void route_crc(void)
{
    CHECK(crc16_ccitt((const uint8_t *)"123456789", 9) == 0x29B1);
}

/////////////////////////////////////////////////////////////////////////////////////////

void test_route(void)
{
    route_crc();
}
