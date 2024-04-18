#include "coordinates.h"

void serial_init(unsigned short);
char serial_in();
int check_sats();
Location get_coord();
unsigned char recv_string(char *rp);
void serial_stringout(char *s);
void serial_txchar(char ch);
void check_vr();