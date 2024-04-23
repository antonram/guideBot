#include "coordinates.h"
#include "gps.h"

void serial_init(unsigned short);
char serial_in();
int check_sats();
Location get_coord();
unsigned char recv_string(char *rp);
void serial_stringout(char *s);
void serial_txchar(char ch);
GPS process_gps_data();