#include <rs232.h>
#include <serial/serial.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>

using namespace serial;

Serial *serialport;

void serial_connect(char* port, int baudrate, int stopbits, int parity, int bytesize){
    serialport = new Serial(port, 9600, Timeout(), (bytesize_t) bytesize, (parity_t) parity, (stopbits_t) stopbits);
}

int serial_is_open() {
    return serialport->isOpen();
}

int serial_write(char* buf, int len) {
    return serialport->write((unsigned char*)buf, len);
}

int serial_read(char* buf, int len) {
    return serialport->read((unsigned char*)buf, len);
}

void serial_flush() {
    serialport->flush();
}

void serial_flush_rx() {
    serialport->flushInput();
}

void serial_flush_tx() {
    serialport->flushOutput();
}

void serial_set_fd(int fd) {
    serialport->setFD(fd);
}

int serial_get_fd() {
    return serialport->getFD();
}

void serial_close() {
    serialport->close();
    serialport->~Serial();
}
