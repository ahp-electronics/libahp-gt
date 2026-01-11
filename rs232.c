#include <rs232.h>
#include <serial.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>

void serial_connect(char* port, int baudrate, const char *mode){
    ahp_serial_OpenComport(port);
    ahp_serial_SetupPort(baudrate, mode, 0);
}

int serial_is_open() {
    return serial_get_fd() != -1;
}

int serial_write(char* buf, int len) {
    return ahp_serial_SendBuf(buf, len);
}

int serial_read(char* buf, int len) {
    return ahp_serial_RecvBuf(buf, len);
}

void serial_flush() {
    ahp_serial_flushRXTX();
}

void serial_flush_rx() {
    ahp_serial_flushRX();
}

void serial_flush_tx() {
    ahp_serial_flushTX();
}

void serial_set_fd(int fd, int bauds) {
    ahp_serial_SetFD(fd, bauds);
}

int serial_get_fd() {
    return ahp_serial_GetFD();
}

void serial_close() {
    ahp_serial_CloseComport();
}
