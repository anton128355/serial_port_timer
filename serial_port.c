#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

#define SERIAL_PORT "/dev/ttyS0"
#define BAUD_RATE B9600
#define DATA "Test"
#define INTERVAL 10

int main() {
    int serial_port;
    struct termios tty;

    // Open the serial port
    serial_port = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_port < 0) {
        perror("Error opening serial port");
        return 1;
    }

    // Get the current configuration of the serial port
    if (tcgetattr(serial_port, &tty) != 0) {
        perror("Error getting tty attributes");
        close(serial_port);
        return 1;
    }

    // Set the baud rate
    cfsetispeed(&tty, BAUD_RATE);
    cfsetospeed(&tty, BAUD_RATE);

    // Configure the serial port settings
    tty.c_cflag &= ~PARENB; // No parity bit
    tty.c_cflag &= ~CSTOPB; // Only one stop bit
    tty.c_cflag &= ~CSIZE;  // Clear the current data size setting
    tty.c_cflag |= CS8;     // 8 data bits
    tty.c_cflag &= ~CRTSCTS; // Disable hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control lines

    // Set raw input and output mode
    tty.c_lflag &= ~ICANON; // Disable canonical mode
    tty.c_lflag &= ~ECHO;   // Disable echo
    tty.c_lflag &= ~ECHOE;  // Disable erase
    tty.c_lflag &= ~ISIG;   // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    // Apply the configuration to the serial port
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        perror("Error setting tty attributes");
        close(serial_port);
        return 1;
    }

    while (1) {
        // Write data to the serial port
        int bytes_written = write(serial_port, DATA, strlen(DATA));
        if (bytes_written < 0) {
            perror("Error writing to serial port");
        } else {
            printf("Sent: %s\n", DATA);
        }

        // Sleep for the specified interval
        sleep(INTERVAL);
    }

    // Close the serial port
    close(serial_port);
    return 0;
}
