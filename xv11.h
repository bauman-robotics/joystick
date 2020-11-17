//
// Created by root on 17.05.2020.
//

#ifndef FULL_FEATURED_RC_XV11_H
#define FULL_FEATURED_RC_XV11_H


void init_serial_port(int tty_fd);
void print_all_data(unsigned char *buf);
int count_errors(unsigned char *buf);

#endif //FULL_FEATURED_RC_XV11_H
