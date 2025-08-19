#include <iostream>

/* LIBRARIES */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>

/* CONSTANTS */
#define INO_DEFAULT_WAIT 3500000
#define INO_NO_WAIT 0
#define INO_NO_TIMEOUT 0

/**
 * Data type for converting between signed and unsigned 16-bit integers and byte arrays.
 * @since 0.1
 */
typedef union _ino_int16
{
	int16_t sig; 						/**< signed integer */
	uint16_t unsig;					/**< unsigned integer */
	unsigned char bytes[2];	/**< byte array */
}
ino_int16;

/**
 * Data type for converting between signed and unsigned 32-bit integers and byte arrays.
 * @since 0.1
 */
typedef union _ino_int32
{
	int32_t sig;						/**< signed integer */
	uint32_t unsig;					/**< unsigned integer */
	unsigned char bytes[4];	/**< byte array */
}
ino_int32;

/**
 * Data type for storing an Arduino connection's settings.
 * @since 0.1
 */
typedef struct _ino_connection_s
{
	char *port;					/**< tty(COM) port */
	speed_t baud;				/**< bit rate */
	int connect_wait;		/**< length of time in microseconds to wait for arduino to reboot
											   on connection. set to zero if arduino model does not reboot
											   automatically on each connection or if autoreboot is disabled. */
	int raw;						/**< whether or not this is a raw connection type.
											   0 = processed/ASCII, terminated by new line character.
											   1 = raw, unprocessed, read an exact number of bytes. */
	int raw_size;				/**< number of bytes expected in each frame. zero and ignored
											   unless raw is set to 1. */
	int raw_timeout;		/**< length of time to wait for arduino between bytes.
											   raw_timeout should be the number of 1/10ths of a second,
											   e.g., raw_timeout=5 would be 5/10 sec or 1/2 sec.
											   0 = never timeout, wait until enough bytes are read.
											   ignored unless raw is set to 1 (TRUE) */
	int fd;							/**< open port; -1 if closed or on error */
}
ino_connection_t;

/**
 * Create a new character-oriented Arduino connection configuration object.
 * Use this function to configure a connection for transfering character data
 * to and from the Arduino through the serial port.  You can receive as many
 * characters as your buffer will hold, but each frame of character data should
 * be terminated by a new line (line feed) character.  This connection can be
 * opened and closed as many times as you like, but must be freed from memory
 * when it is no longer needed by calling ino_connection_destroy(). If your
 * Arduino model does not automatically reboot each time a serial connection is
 * made to it or this feature has been disabled for your Arduino, then you may
 * use INO_NO_WAIT for your connect_wait for faster connection times.  You may
 * also use INO_DEFAULT_WAIT which should be a safe number of microseconds to
 * wait for the Arduino to reboot before trying to configure the connection.
 * The baud constants are part of the C standard library and are included in
 * the termios.h or bits/termios.h library header, e.g., B9600 or B115200.
 * @param port String representation of port to use, e.g., ttyACM0 or COM1.
 * @param baud Bit-rate, i.e., one of B4800, B9600, B19200,... B115200, etc.
 * @param connect_wait Microseconds to wait for Arduino to auto-reboot on connect.
 * @return Pointer to new ino_connection_t object containing new configuration.
 * @since 0.1
 */
ino_connection_t *ino_connection_new(char *port, speed_t baud, int connect_wait);

/**
 * Create a new raw, unprocessed Arduino connection configuration object.
 * Use this function to configure a connection for transfering raw byte or integer
 * data to and from the Arduino through the serial port.  This connection can be
 * opened and closed as many times as you like, but must be freed from memory
 * when it is no longer needed by calling ino_connection_destroy(). If your
 * Arduino model does not automatically reboot each time a serial connection is
 * made to it or this feature has been disabled for your Arduino, then you may
 * use INO_NO_WAIT for your connect_wait for faster connection times.  You may
 * also use INO_DEFAULT_WAIT which should be a safe number of microseconds to
 * wait for the Arduino to reboot before trying to configure the connection.
 * The baud constants are part of the C standard library and are included in
 * the termios.h or bits/termios.h library header, e.g., B9600 or B115200.
 * read_timeout is the time to wait between bytes before timing out.  You may
 * use INO_NO_TIMEOUT if you are waiting for an exact number of bytes. Otherwise
 * the read_timeout should be set to the number of 1/10ths of a second to wait,
 * e.g., a read_timeout of 5 will wait 5/10ths or 1/2 of a second between bytes
 * before giving up altogether.
 * @param port String representation of port to use, e.g., ttyACM0 or COM1.
 * @param baud Bit-rate, i.e., one of B4800, B9600, B19200,... B115200, etc.
 * @param connect_wait Microseconds to wait for Arduino to auto-reboot on connect.
 * @param frame_size_bytes Byte-length of frame(s) to be read.
 * @param read_timeout 1/10ths of a second to wait between bytes before timeout.
 * @return Pointer to new ino_connection_t object containing new configuration.
 * @since 0.1
 */
ino_connection_t *ino_connection_raw(char *port, speed_t baud, int connect_wait, int frame_size_bytes, int read_timeout);

/**
 * Close Arduino connection and free config object from memory.
 * Use this function when the connection is no longer needed.  To merely
 * close the connection and retain it to be opened again later, use the
 * ino_close() function instead.
 * @param cnx Pointer to ino_connection_t object to be destroyed.
 * @since 0.1
 */
void ino_connection_destroy(ino_connection_t *cnx);

/**
 * Open a configured serial port to an Arduino.
 * Use this function to open a connection created using either of
 * ino_connection_new() or ino_connection_raw().
 * @param cnx Pointer to ino_connection_t object to open.
 * @return -1 on error and zero on success.
 * @since 0.1
 */
int ino_open(ino_connection_t *cnx);

/**
 * Close Arduino connection, but retain the ino_connection_t configuration.
 * Use this function to close the Arduino connection port temporarily.
 * To completely destroy the ino_connection_t object, freeing memory, use
 * the ino_connection_destroy() function instead.
 * @param cnx Pointer to ino_connection_t object to close.
 * @return -1 on error and zero on success.
 * @since 0.1
 */
int ino_close(ino_connection_t *cnx);

/**
 * Test whether the given ino_connection_t is open.
 * Use this function to determine whether the port is open for reading and
 * writing or if there was an error opening the connection or if the
 * connection has been closed.
 * @param cnx Pointer to ino_connection_t object to test.
 * @return 1 if open and 0 if closed.
 * @since 0.1
 */
int ino_is_open(ino_connection_t *cnx);

/**
 * Read a string from the Arduino.
 * Read processed, canonical, string-oriented i/o, terminated by new line character.
 * The connection must be created using the ino_connection_new() function and
 * opened using ino_open().
 * @param cnx Pointer to ino_connection_t object from which to be read.
 * @param buf String buffer into which to read the string from the Arduino.
 * @param max_bytes Maximum number of bytes to read, e.g., size of buffer.
 * @return -1 on error or number of bytes read otherwise.
 * @since 0.1
 */
int ino_read(ino_connection_t *cnx, char *buf, int max_bytes);

/**
 * Cast white-space delimited string data to int by position in the string.
 * Extract numeric value at position, "pos," in the string.  E.g., in order
 * to extract an int at the beginning of the string, use pos = 0. In order
 * to extract the second numeric value in the string, use pos = 1, etc.
 * @param frame Frame of characters returned by ino_read().
 * @param pos Position in the frame of numeric values, starting with zero.
 * @return The integer read at the given position, or zero on error.
 * @since 0.1
 */
int ino_cast_int(char *frame, int pos);

/**
 * Cast white-space delimited string data to long by position in the string.
 * Extract numeric value at position, "pos," in the string.  E.g., in order
 * to extract a long at the beginning of the string, use pos = 0. In order
 * to extract the second numeric value in the string, use pos = 1, etc.
 * @param frame Frame of characters returned by ino_read().
 * @param pos Position in the frame of numeric values, starting with zero.
 * @return The long read at the given position, or zero on error.
 * @since 0.1
 */
long ino_cast_long(char *frame, int pos);

/**
 * Cast white-space delimited string data to float by position in the string.
 * Extract numeric value at position, "pos," in the string.  E.g., in order
 * to extract a float at the beginning of the string, use pos = 0. In order
 * to extract the second numeric value in the string, use pos = 1, etc.
 * @param frame Frame of characters returned by ino_read().
 * @param pos Position in the frame of numeric values, starting with zero.
 * @return The float read at the given position, or zero on error.
 * @since 0.1
 */
float ino_cast_float(char *frame, int pos);

/**
 * Cast white-space delimited string data to double by position in the string.
 * Extract numeric value at position, "pos," in the string.  E.g., in order
 * to extract a double at the beginning of the string, use pos = 0. In order
 * to extract the second numeric value in the string, use pos = 1, etc.
 * @param frame Frame of characters returned by ino_read().
 * @param pos Position in the frame of numeric values, starting with zero.
 * @return The double read at the given position, or zero on error.
 * @since 0.1
 */
double ino_cast_double(char *frame, int pos);

/* read raw, non-canonical byte i/o, exact number of bytes known to each end */
/**
 * Read raw bytes from a raw Arduino serial connection.
 * Reads an exact number of raw bytes from the serial port into an array of
 * uint8_t.  Use this function for more complex or custom interpretation
 * of the bytes delivered.  The function requires an ino_connection_t object
 * created using ino_connection_raw().
 * @param cnx Pointer to ino_connection_t object from which to read data.
 * @param buf Pointer to the beginning of an array of uint8_t.
 * @param frame_size_bytes The exact number of bytes to read.
 * @return The number of bytes read, or -1 on error.
 * @since 0.1
 */
int ino_read_raw(ino_connection_t *cnx, uint8_t *buf, int frame_size_bytes);

/**
 * Read a single signed 16-bit integer from the given Arduino raw connection.
 * Reads 2 bytes from the serial port and converts them from network byte order
 * into host byte order and casts them to a signed 16-bit integer.  The
 * corresponding Arduino library "send" function will send the integer byte by
 * byte in network byte order.  The function requires an ino_connection_t object
 * created using ino_connection_raw() with a 2 byte frame size.
 * @param cnx Pointer to ino_connection_t object from which to read data.
 * @return The int16_t read from the serial port.
 * @since 0.1
 */
int16_t ino_read_int16(ino_connection_t *cnx);

/**
 * Read a single unsigned 16-bit integer from the given Arduino raw connection.
 * Reads 2 bytes from the serial port and converts them from network byte order
 * into host byte order and casts them to an unsigned 16-bit integer.  The
 * corresponding Arduino library "send" function will send the integer byte by
 * byte in network byte order.  The function requires an ino_connection_t object
 * created using ino_connection_raw() with a 2 byte frame size.
 * @param cnx Pointer to ino_connection_t object from which to read data.
 * @return The uint16_t read from the serial port.
 * @since 0.1
 */
uint16_t ino_read_uint16(ino_connection_t *cnx);

/**
 * Read multiple signed 16-bit integers from an Arduino raw connection at once.
 * Reads 2 bytes per integer from the serial port and converts them from network
 * byte order into host byte order and casts them to signed 16-bit integers.
 * The corresponding Arduino library "send" function will send the integer byte by
 * byte in network byte order.  The function requires an ino_connection_t object
 * created using ino_connection_raw() with a 2 bytes per integer frame size,
 * i.e., three 2-byte integers would require a connection frame size of 6 bytes.
 * @param cnx Pointer to ino_connection_t object from which to read data.
 * @param int_buf Pointer to the beginning of an array of int16_t.
 * @param int_count The number of integers to attempt reading <= array length.
 * @return The number of integers read from the serial port.
 * @since 0.1
 */
int ino_read_int16s(ino_connection_t *cnx, int16_t *int_buf, int int_count);

/**
 * Read multiple unsigned 16-bit integers from an Arduino raw connection at once.
 * Reads 2 bytes per integer from the serial port and converts them from network
 * byte order into host byte order and casts them to unsigned 16-bit integers.
 * The corresponding Arduino library "send" function will send the integer byte by
 * byte in network byte order.  The function requires an ino_connection_t object
 * created using ino_connection_raw() with a 2 bytes per integer frame size,
 * i.e., three 2-byte integers would require a connection frame size of 6 bytes.
 * @param cnx Pointer to ino_connection_t object from which to read data.
 * @param int_buf Pointer to the beginning of an array of uint16_t.
 * @param int_count The number of integers to attempt reading <= array length.
 * @return The number of integers read from the serial port.
 * @since 0.1
 */
int ino_read_uint16s(ino_connection_t *cnx, uint16_t *int_buf, int int_count);

/**
 * Read a single signed 32-bit integer from the given Arduino raw connection.
 * Reads 4 bytes from the serial port and converts them from network byte order
 * into host byte order and casts them to a signed 32-bit integer.  The
 * corresponding Arduino library "send" function will send the integer byte by
 * byte in network byte order.  The function requires an ino_connection_t object
 * created using ino_connection_raw() with a 4 byte frame size.
 * @param cnx Pointer to ino_connection_t object from which to read data.
 * @return The int32_t read from the serial port.
 * @since 0.1
 */
int32_t ino_read_int32(ino_connection_t *cnx);

/**
 * Read a single unsigned 32-bit integer from the given Arduino raw connection.
 * Reads 4 bytes from the serial port and converts them from network byte order
 * into host byte order and casts them to an unsigned 32-bit integer.  The
 * corresponding Arduino library "send" function will send the integer byte by
 * byte in network byte order.  The function requires an ino_connection_t object
 * created using ino_connection_raw() with a 4 byte frame size.
 * @param cnx Pointer to ino_connection_t object from which to read data.
 * @return The uint32_t read from the serial port.
 * @since 0.1
 */
uint32_t ino_read_uint32(ino_connection_t *cnx);

/**
 * Read multiple signed 32-bit integers from an Arduino raw connection at once.
 * Reads 4 bytes per integer from the serial port and converts them from network
 * byte order into host byte order and casts them to signed 32-bit integers.
 * The corresponding Arduino library "send" function will send the integer byte by
 * byte in network byte order.  The function requires an ino_connection_t object
 * created using ino_connection_raw() with a 4 bytes per integer frame size,
 * i.e., three 4-byte integers would require a connection frame size of 12 bytes.
 * @param cnx Pointer to ino_connection_t object from which to read data.
 * @param int_buf Pointer to the beginning of an array of int32_t.
 * @param int_count The number of integers to attempt reading <= array length.
 * @return The number of integers read from the serial port.
 * @since 0.1
 */
int ino_read_int32s(ino_connection_t *cnx, int32_t *int_buf, int int_count);

/**
 * Read multiple unsigned 32-bit integers from an Arduino raw connection at once.
 * Reads 4 bytes per integer from the serial port and converts them from network
 * byte order into host byte order and casts them to unsigned 32-bit integers.
 * The corresponding Arduino library "send" function will send the integer byte by
 * byte in network byte order.  The function requires an ino_connection_t object
 * created using ino_connection_raw() with a 4 bytes per integer frame size,
 * i.e., three 4-byte integers would require a connection frame size of 12 bytes.
 * @param cnx Pointer to ino_connection_t object from which to read data.
 * @param int_buf Pointer to the beginning of an array of uint32_t.
 * @param int_count The number of integers to attempt reading <= array length.
 * @return The number of integers read from the serial port.
 * @since 0.1
 */
int ino_read_uint32s(ino_connection_t *cnx, uint32_t *int_buf, int int_count);

/**
 * Send an array of raw bytes to the Arduino through the serial port.
 * Sends an array of uint8_t through the given ino_connection_t/serial port.
 * @param cnx Pointer to ino_connection_t object through which to send data.
 * @param buf Array of uint8_t bytes to send to the Arduino.
 * @param frame_size_bytes The number of bytes to be sent to the Arduino.
 * @return The number of bytes actually sent, or -1 on error.
 * @since 0.1
 */
int ino_send_raw(ino_connection_t *cnx, uint8_t *buf, int frame_size_bytes);

/**
 * Send a single signed 16-bit integer to the Arduino through the serial port.
 * Converts an int16_t to network byte order and sends it byte by byte through
 * the given serial port connection to the Arduino on the other end.  Network
 * byte order raw integers can be read by the Arduino using the corresponding
 * "read" function in the Arduino library, arduino_libuino.h .
 * @param cnx Pointer to ino_connection_t object through which to send data.
 * @param val int16_t to be sent to Arduino.
 * @return The number of bytes actually sent, or -1 on error.
 * @since 0.1
 */
int ino_send_int16(ino_connection_t *cnx, int16_t val);

/**
 * Send a single unsigned 16-bit integer to the Arduino through the serial port.
 * Converts an uint16_t to network byte order and sends it byte by byte through
 * the given serial port connection to the Arduino on the other end.  Network
 * byte order raw integers can be read by the Arduino using the corresponding
 * "read" function in the Arduino library, arduino_libuino.h .
 * @param cnx Pointer to ino_connection_t object through which to send data.
 * @param val uint16_t to be sent to Arduino.
 * @return The number of bytes actually sent, or -1 on error.
 * @since 0.1
 */
int ino_send_uint16(ino_connection_t *cnx, uint16_t val);

/**
 * Send a single signed 32-bit integer to the Arduino through the serial port.
 * Converts an int32_t to network byte order and sends it byte by byte through
 * the given serial port connection to the Arduino on the other end.  Network
 * byte order raw integers can be read by the Arduino using the corresponding
 * "read" function in the Arduino library, arduino_libuino.h .
 * @param cnx Pointer to ino_connection_t object through which to send data.
 * @param val int32_t to be sent to Arduino.
 * @return The number of bytes actually sent, or -1 on error.
 * @since 0.1
 */
int ino_send_int32(ino_connection_t *cnx, int32_t val);

/**
 * Send a single unsigned 32-bit integer to the Arduino through the serial port.
 * Converts a uint32_t to network byte order and sends it byte by byte through
 * the given serial port connection to the Arduino on the other end.  Network
 * byte order raw integers can be read by the Arduino using the corresponding
 * "read" function in the Arduino library, arduino_libuino.h .
 * @param cnx Pointer to ino_connection_t object through which to send data.
 * @param val uint32_t to be sent to Arduino.
 * @return The number of bytes actually sent, or -1 on error.
 * @since 0.1
 */
int ino_send_uint32(ino_connection_t *cnx, uint32_t val);

/**
 * Send a single char through the serial port.
 * Sends a single character through the serial port to the Arduino.  This is just
 * a wrapper function for
 * @code
 * 	write(cnx->fd, c, 1);
 * @endcode
 * @param c The character to send to the Arduino.
 * @return 1 on success, or -1 on failure.
 * @since 0.1
 */
 int ino_send_char(ino_connection_t *cnx, char c);

/**
 * Create a new character-oriented Arduino connection configuration object.
 * Use this function to configure a connection for transfering character data
 * to and from the Arduino through the serial port.  You can receive as many
 * characters as your buffer will hold, but each frame of character data should
 * be terminated by a new line (line feed) character.  This connection can be
 * opened and closed as many times as you like, but must be freed from memory
 * when it is no longer needed by calling ino_connection_destroy(). If your
 * Arduino model does not automatically reboot each time a serial connection is
 * made to it or this feature has been disabled for your Arduino, then you may
 * use INO_NO_WAIT for your connect_wait for faster connection times.  You may
 * also use INO_DEFAULT_WAIT which should be a safe number of microseconds to
 * wait for the Arduino to reboot before trying to configure the connection.
 * The baud constants are part of the C standard library and are included in
 * the termios.h or bits/termios.h library header, e.g., B9600 or B115200.
 * @param port String representation of port to use, e.g., ttyACM0 or COM1.
 * @param baud Bit-rate, i.e., one of B4800, B9600, B19200,... B115200, etc.
 * @param connect_wait Microseconds to wait for Arduino to auto-reboot on connect.
 * @return Pointer to new ino_connection_t object containing new configuration.
 * @since 0.1
 */
ino_connection_t *
ino_connection_new(char *port, speed_t baud, int connect_wait)
{
	ino_connection_t *cnx = NULL;

	cnx = (ino_connection_t *)malloc(sizeof(ino_connection_t));
	if ( cnx )
	{
		cnx->port = port;
		cnx->baud = baud;
		cnx->connect_wait = connect_wait;
		cnx->raw = 0;
		cnx->raw_size = 0;
		cnx->raw_timeout = 0;
		cnx->fd = -1;
	}

	return cnx;
}

/**
 * Create a new raw, unprocessed Arduino connection configuration object.
 * Use this function to configure a connection for transfering raw byte or integer
 * data to and from the Arduino through the serial port.  This connection can be
 * opened and closed as many times as you like, but must be freed from memory
 * when it is no longer needed by calling ino_connection_destroy(). If your
 * Arduino model does not automatically reboot each time a serial connection is
 * made to it or this feature has been disabled for your Arduino, then you may
 * use INO_NO_WAIT for your connect_wait for faster connection times.  You may
 * also use INO_DEFAULT_WAIT which should be a safe number of microseconds to
 * wait for the Arduino to reboot before trying to configure the connection.
 * The baud constants are part of the C standard library and are included in
 * the termios.h or bits/termios.h library header, e.g., B9600 or B115200.
 * read_timeout is the time to wait between bytes before timing out.  You may
 * use INO_NO_TIMEOUT if you are waiting for an exact number of bytes. Otherwise
 * the read_timeout should be set to the number of 1/10ths of a second to wait,
 * e.g., a read_timeout of 5 will wait 5/10ths or 1/2 of a second between bytes
 * before giving up altogether.
 * @param port String representation of port to use, e.g., ttyACM0 or COM1.
 * @param baud Bit-rate, i.e., one of B4800, B9600, B19200,... B115200, etc.
 * @param connect_wait Microseconds to wait for Arduino to auto-reboot on connect.
 * @param frame_size_bytes Byte-length of frame(s) to be read.
 * @param read_timeout 1/10ths of a second to wait between bytes before timeout.
 * @return Pointer to new ino_connection_t object containing new configuration.
 * @since 0.1
 */
ino_connection_t *
ino_connection_raw(char *port, speed_t baud, int connect_wait, int frame_size_bytes, int read_timeout)
{
	ino_connection_t *cnx = NULL;

	cnx = (ino_connection_t *)malloc(sizeof(ino_connection_t));
	if ( cnx )
	{
		cnx->port = port;
		cnx->baud = baud;
		cnx->connect_wait = connect_wait;
		cnx->raw = 1;
		cnx->raw_size = frame_size_bytes;
		cnx->raw_timeout = read_timeout;
		cnx->fd = -1;
	}

	return cnx;
}

/**
 * Close Arduino connection and free config object from memory.
 * Use this function when the connection is no longer needed.  To merely
 * close the connection and retain it to be opened again later, use the
 * ino_close() function instead.
 * @param cnx Pointer to ino_connection_t object to be destroyed.
 * @since 0.1
 */
void
ino_connection_destroy(ino_connection_t *cnx)
{
	if ( cnx )
	{
		/* close the connection */
		ino_close(cnx);
		/* free the connection's memory */
		free(cnx);
	}
}

/**
 * Open a configured serial port to an Arduino.
 * Use this function to open a connection created using either of
 * ino_connection_new() or ino_connection_raw().
 * @param cnx Pointer to ino_connection_t object to open.
 * @return -1 on error and zero on success.
 * @since 0.1
 */
int
ino_open(ino_connection_t *cnx)
{
  int error = -1;
  struct termios toptions;

  if ( cnx )
  {
  	/* if this connection is already open, close it. */
  	if ( cnx->fd > -1 ) ino_close(cnx);

		/* open serial port */
		cnx->fd = open(cnx->port, O_RDWR | O_NOCTTY);

		if ( cnx->fd >= 0 )
		{
			/* wait for the Arduino to reboot */
			usleep(cnx->connect_wait);

			/* init toptions with current port settings */
			tcgetattr(cnx->fd, &toptions);
			toptions.c_cflag = 0;
			toptions.c_iflag = 0;
			toptions.c_oflag = 0;
			toptions.c_lflag = 0;

			/* new port settings */
			cfsetispeed(&toptions, cnx->baud);	/* input baud*/
			cfsetospeed(&toptions, cnx->baud);	/* output baud*/
			toptions.c_cflag |= ( CS8 | HUPCL | CREAD | CLOCAL );
			if ( cnx->raw )
			{
				/* set min bytes and timeout */
				toptions.c_cc[VMIN]  = cnx->raw_size;
				toptions.c_cc[VTIME] = cnx->raw_timeout;
			}
			else
			{
				/* canonical, turn on processing */
				toptions.c_lflag |= ( ICANON | ECHOE | ECHOK | ECHOCTL | ECHOKE );
			}

			/* commit changes to port settings */
			error = tcsetattr(cnx->fd, TCSANOW, &toptions);
			if ( error == -1 ) ino_close(cnx);
		}
	}

  return error;
}

/**
 * Close Arduino connection, but retain the ino_connection_t configuration.
 * Use this function to close the Arduino connection port temporarily.
 * To completely destroy the ino_connection_t object, freeing memory, use
 * the ino_connection_destroy() function instead.
 * @param cnx Pointer to ino_connection_t object to close.
 * @return -1 on error and zero on success.
 * @since 0.1
 */
int
ino_close(ino_connection_t *cnx)
{
	int error = 0;

	/* if connected */
	if ( cnx && cnx->fd > -1 )
	{
		/* mark connection closed */
		cnx->fd = -1;
		/* close serial port */
		error = close(cnx->fd);
	}

	return error;
}

/**
 * Test whether the given ino_connection_t is open.
 * Use this function to determine whether the port is open for reading and
 * writing or if there was an error opening the connection or if the
 * connection has been closed.
 * @param cnx Pointer to ino_connection_t object to test.
 * @return 1 if open and 0 if closed.
 * @since 0.1
 */
int ino_is_open(ino_connection_t *cnx)
{
	return ( cnx && cnx->fd > -1 );
}

/**
 * Read a string from the Arduino.
 * Read processed, canonical, string-oriented i/o, terminated by new line character.
 * The connection must be created using the ino_connection_new() function and
 * opened using ino_open().
 * @param cnx Pointer to ino_connection_t object from which to be read.
 * @param buf String buffer into which to read the string from the Arduino.
 * @param max_bytes Maximum number of bytes to read, e.g., size of buffer.
 * @return -1 on error or number of bytes read otherwise.
 * @since 0.1
 */
int
ino_read(ino_connection_t *cnx, char *buf, int max_bytes)
{
  int n = -1;		/* return number of bytes read or -1 on error */

	/* if connected */
	if ( cnx && cnx->fd > -1 )
	{
		/* receive string from arduino */
		n = read(cnx->fd, buf, max_bytes);
		/* insert terminating zero in the string */
		buf[n] = '\0';
 	}

  return n;
}

/**
 * Cast white-space delimited string data to int by position in the string.
 * Extract numeric value at position, "pos," in the string.  E.g., in order
 * to extract an int at the beginning of the string, use pos = 0. In order
 * to extract the second numeric value in the string, use pos = 1, etc.
 * @param frame Frame of characters returned by ino_read().
 * @param pos Position in the frame of numeric values, starting with zero.
 * @return The integer read at the given position, or zero on error.
 * @since 0.1
 */
int
ino_cast_int(char *frame, int pos)
{
	int val = 0;
	char *start = frame;
	char *end = NULL;
	char *delims = " \t\r\n;|";
	int len = strlen(frame);
	int j = 0;

	while ( j<pos && start < frame+len )
	{
		start = strpbrk(start,delims) + 1;
		while( start < frame+len && strchr(delims,*start) )
			start++;
		j++;
	}
	if ( start < frame+len )
		val = strtol(start, &end, 10);

	return val;
}

/**
 * Cast white-space delimited string data to long by position in the string.
 * Extract numeric value at position, "pos," in the string.  E.g., in order
 * to extract a long at the beginning of the string, use pos = 0. In order
 * to extract the second numeric value in the string, use pos = 1, etc.
 * @param frame Frame of characters returned by ino_read().
 * @param pos Position in the frame of numeric values, starting with zero.
 * @return The long read at the given position, or zero on error.
 * @since 0.1
 */
long
ino_cast_long(char *frame, int pos)
{
	long val = 0;
	char *start = frame;
	char *end = NULL;
	char *delims = " \t\r\n;|";
	int len = strlen(frame);
	int j = 0;

	while ( j<pos && start < frame+len )
	{
		start = strpbrk(start,delims) + 1;
		while( start < frame+len && strchr(delims,*start) )
			start++;
		j++;
	}
	if ( start < frame+len )
		val = strtol(start, &end, 10);

	return val;
}

/**
 * Cast white-space delimited string data to float by position in the string.
 * Extract numeric value at position, "pos," in the string.  E.g., in order
 * to extract a float at the beginning of the string, use pos = 0. In order
 * to extract the second numeric value in the string, use pos = 1, etc.
 * @param frame Frame of characters returned by ino_read().
 * @param pos Position in the frame of numeric values, starting with zero.
 * @return The float read at the given position, or zero on error.
 * @since 0.1
 */
float
ino_cast_float(char *frame, int pos)
{
	float val = 0;
	char *start = frame;
	char *end = NULL;
	char *delims = " \t\r\n;|";
	int len = strlen(frame);
	int j = 0;

	while ( j<pos && start < frame+len )
	{
		start = strpbrk(start,delims) + 1;
		while( start < frame+len && strchr(delims,*start) )
			start++;
		j++;
	}
	if ( start < frame+len )
		val = strtof(start, &end);

	return val;
}

/**
 * Cast white-space delimited string data to double by position in the string.
 * Extract numeric value at position, "pos," in the string.  E.g., in order
 * to extract a double at the beginning of the string, use pos = 0. In order
 * to extract the second numeric value in the string, use pos = 1, etc.
 * @param frame Frame of characters returned by ino_read().
 * @param pos Position in the frame of numeric values, starting with zero.
 * @return The double read at the given position, or zero on error.
 * @since 0.1
 */
double
ino_cast_double(char *frame, int pos)
{
	double val = 0;
	char *start = frame;
	char *end = NULL;
	char *delims = " \t\r\n;|";
	int len = strlen(frame);
	int j = 0;

	while ( j<pos && start < frame+len )
	{
		start = strpbrk(start,delims) + 1;
		while( start < frame+len && strchr(delims,*start) )
			start++;
		j++;
	}
	if ( start < frame+len )
		val = strtod(start, &end);

	return val;
}

/**
 * Read raw bytes from a raw Arduino serial connection.
 * Reads an exact number of raw bytes from the serial port into an array of
 * uint8_t.  Use this function for more complex or custom interpretation
 * of the bytes delivered.  The function requires an ino_connection_t object
 * created using ino_connection_raw().
 * @param cnx Pointer to ino_connection_t object from which to read data.
 * @param buf Pointer to the beginning of an array of uint8_t.
 * @param frame_size_bytes The exact number of bytes to read.
 * @return The number of bytes read, or -1 on error.
 * @since 0.1
 */
int
ino_read_raw(ino_connection_t *cnx, uint8_t *buf, int frame_size_bytes)
{
  int n = -1;		/* return number of bytes read or -1 on error */

	/* if connected */
	if ( cnx && cnx->fd > -1 )
	{
		/* receive string from arduino */
		n = read(cnx->fd, buf, frame_size_bytes);
	}

  return n;
}

/**
 * Read a single signed 16-bit integer from the given Arduino raw connection.
 * Reads 2 bytes from the serial port and converts them from network byte order
 * into host byte order and casts them to a signed 16-bit integer.  The
 * corresponding Arduino library "send" function will send the integer byte by
 * byte in network byte order.  The function requires an ino_connection_t object
 * created using ino_connection_raw() with a 2 byte frame size.
 * @param cnx Pointer to ino_connection_t object from which to read data.
 * @return The int16_t read from the serial port.
 * @since 0.1
 */
int16_t
ino_read_int16(ino_connection_t *cnx)
{
  int n = 0;												/* total bytes received */
  int j = 0;												/* loop indices */
  ino_int16 tmp;										/* working tmp int */
  int frame_size = 2;								/* number of bytes in frame */
  unsigned char frame[frame_size];	/* frame buffer */

	/* init tmp to zero */
	tmp.unsig = 0;

	/* if connected */
	if ( cnx && cnx->fd > -1 )
	{
		/* prepare raw byte buffer */
		memset(frame,0,frame_size);
		/* receive frame of int bytes from arduino */
		n = read(cnx->fd, frame, frame_size);
		/* for each byte */
		for(j=0; j<n; j++)
		{
			/* shift existing bits to make room for next byte */
			tmp.unsig <<= 8;
			/* add the bits of the next byte to the end of working tmp */
			tmp.unsig |= frame[j];
		}
	}

	/* return as signed int*/
	return tmp.sig;
}

/**
 * Read a single unsigned 16-bit integer from the given Arduino raw connection.
 * Reads 2 bytes from the serial port and converts them from network byte order
 * into host byte order and casts them to an unsigned 16-bit integer.  The
 * corresponding Arduino library "send" function will send the integer byte by
 * byte in network byte order.  The function requires an ino_connection_t object
 * created using ino_connection_raw() with a 2 byte frame size.
 * @param cnx Pointer to ino_connection_t object from which to read data.
 * @return The uint16_t read from the serial port.
 * @since 0.1
 */
uint16_t
ino_read_uint16(ino_connection_t *cnx)
{
  int n;														/* total bytes received */
  int j=0;													/* loop indices */
  ino_int16 tmp;										/* working tmp int */
  int frame_size = 2;								/* number of bytes in frame */
  unsigned char frame[frame_size];	/* frame buffer */

	/* init tmp to zero */
	tmp.unsig = 0;

	/* if connected */
	if ( cnx && cnx->fd > -1 )
	{
		/* prepare raw byte buffer */
		memset(frame,0,frame_size);
		/* receive frame of int bytes from arduino */
		n = read(cnx->fd, frame, frame_size);
		/* for each byte */
		for(j=0; j<n; j++)
		{
			/* shift existing bits to make room for next byte */
			tmp.unsig <<= 8;
			/* add the bits of the next byte to the end of working tmp */
			tmp.unsig |= frame[j];
		}
	}

	/* return as unsigned int*/
	return tmp.unsig;
}

/**
 * Read multiple signed 16-bit integers from an Arduino raw connection at once.
 * Reads 2 bytes per integer from the serial port and converts them from network
 * byte order into host byte order and casts them to signed 16-bit integers.
 * The corresponding Arduino library "send" function will send the integer byte by
 * byte in network byte order.  The function requires an ino_connection_t object
 * created using ino_connection_raw() with a 2 bytes per integer frame size,
 * i.e., three 2-byte integers would require a connection frame size of 6 bytes.
 * @param cnx Pointer to ino_connection_t object from which to read data.
 * @param int_buf Pointer to the beginning of an array of int16_t.
 * @param int_count The number of integers to attempt reading <= array length.
 * @return The number of integers read from the serial port.
 * @since 0.1
 */
int
ino_read_int16s(ino_connection_t *cnx, int16_t *int_buf, int int_count)
{
  int n;														/* total bytes received */
  int i=0, k=0;											/* loop indices */
  ino_int16 tmp;										/* working tmp int */
  int frame_size = int_count * 2;		/* number of bytes in frame */
  unsigned char frame[frame_size];	/* frame buffer */

	/* if connected */
	if ( cnx && cnx->fd > -1 )
	{
		/* prepare raw byte buffer */
		memset(frame,0,frame_size);
		/* receive frame of int's from arduino */
		n = read(cnx->fd, frame, frame_size);
		/* break sets of bytes up into integers */
		/* for each set of bytes corresponding to an arduino int */
		for(i=0; i<n; i+=2)
		{
			/* init tmp to zero */
			tmp.unsig = 0;
			/* add first byte */
			tmp.unsig |= frame[i];
			/* shift in the second byte */
			tmp.unsig <<= 8;
			tmp.unsig |= frame[i+1];
			/* convert to signed int and add to int_buf */
			int_buf[k++] = tmp.sig;
		}
	}

  /* return the number of integers read */
  return k;
}

/**
 * Read multiple unsigned 16-bit integers from an Arduino raw connection at once.
 * Reads 2 bytes per integer from the serial port and converts them from network
 * byte order into host byte order and casts them to unsigned 16-bit integers.
 * The corresponding Arduino library "send" function will send the integer byte by
 * byte in network byte order.  The function requires an ino_connection_t object
 * created using ino_connection_raw() with a 2 bytes per integer frame size,
 * i.e., three 2-byte integers would require a connection frame size of 6 bytes.
 * @param cnx Pointer to ino_connection_t object from which to read data.
 * @param int_buf Pointer to the beginning of an array of uint16_t.
 * @param int_count The number of integers to attempt reading <= array length.
 * @return The number of integers read from the serial port.
 * @since 0.1
 */
int
ino_read_uint16s(ino_connection_t *cnx, uint16_t *int_buf, int int_count)
{
  int n;														/* total bytes received */
  int i=0, k=0;											/* loop indices */
  ino_int16 tmp;										/* working tmp int */
  int frame_size = int_count * 2;		/* number of bytes in frame */
  unsigned char frame[frame_size];	/* frame buffer */

	/* if connected */
	if ( cnx && cnx->fd > -1 )
	{
		/* prepare raw byte buffer */
		memset(frame,0,frame_size);
		/* receive frame of int's from arduino */
		n = read(cnx->fd, frame, frame_size);
		/* break sets of bytes up into integers */
		/* for each set of bytes corresponding to an arduino int */
		for(i=0; i<n; i+=2)
		{
			/* init tmp to zero */
			tmp.unsig = 0;
			/* add first byte */
			tmp.unsig |= frame[i];
			/* shift in the second byte */
			tmp.unsig <<= 8;
			tmp.unsig |= frame[i+1];
			/* take unsigned int value and add to int_buf */
			int_buf[k++] = tmp.unsig;
		}
	}

  /* return the number of integers read */
  return k;
}

/**
 * Read a single signed 32-bit integer from the given Arduino raw connection.
 * Reads 4 bytes from the serial port and converts them from network byte order
 * into host byte order and casts them to a signed 32-bit integer.  The
 * corresponding Arduino library "send" function will send the integer byte by
 * byte in network byte order.  The function requires an ino_connection_t object
 * created using ino_connection_raw() with a 4 byte frame size.
 * @param cnx Pointer to ino_connection_t object from which to read data.
 * @return The int32_t read from the serial port.
 * @since 0.1
 */
int32_t
ino_read_int32(ino_connection_t *cnx)
{
  int n;														/* total bytes received */
  int j=0;													/* loop indices */
  ino_int32 tmp;										/* working tmp int */
  int frame_size = 4;								/* number of bytes in frame */
  unsigned char frame[frame_size];	/* frame buffer */

	/* init tmp to zero */
	tmp.unsig = 0;

	/* if connected */
	if ( cnx && cnx->fd > -1 )
	{
		/* prepare raw byte buffer */
		memset(frame,0,frame_size);
		/* receive frame of int bytes from arduino */
		n = read(cnx->fd, frame, frame_size);
		/* for each byte */
		for(j=0; j<n; j++)
		{
			/* shift existing bits to make room for next byte */
			tmp.unsig <<= 8;
			/* add the bits of the next byte to the end of working tmp */
			tmp.unsig |= frame[j];
		}
	}

	/* return as signed int*/
	return tmp.sig;
}

/**
 * Read a single unsigned 32-bit integer from the given Arduino raw connection.
 * Reads 4 bytes from the serial port and converts them from network byte order
 * into host byte order and casts them to an unsigned 32-bit integer.  The
 * corresponding Arduino library "send" function will send the integer byte by
 * byte in network byte order.  The function requires an ino_connection_t object
 * created using ino_connection_raw() with a 4 byte frame size.
 * @param cnx Pointer to ino_connection_t object from which to read data.
 * @return The uint32_t read from the serial port.
 * @since 0.1
 */
uint32_t
ino_read_uint32(ino_connection_t *cnx)
{
  int n;														/* total bytes received */
  int j=0;													/* loop indices */
  ino_int32 tmp;										/* working tmp int */
  int frame_size = 4;								/* number of bytes in frame */
  unsigned char frame[frame_size];	/* frame buffer */

	/* init tmp to zero */
	tmp.unsig = 0;

	/* if connected */
	if ( cnx && cnx->fd > -1 )
	{
		/* prepare raw byte buffer */
		memset(frame,0,frame_size);
		/* receive frame of int bytes from arduino */
		n = read(cnx->fd, frame, frame_size);
		/* for each byte */
		for(j=0; j<n; j++)
		{
			/* shift existing bits to make room for next byte */
			tmp.unsig <<= 8;
			/* add the bits of the next byte to the end of working tmp */
			tmp.unsig |= frame[j];
		}
	}

	/* return as unsigned int*/
	return tmp.unsig;
}

/**
 * Read multiple signed 32-bit integers from an Arduino raw connection at once.
 * Reads 4 bytes per integer from the serial port and converts them from network
 * byte order into host byte order and casts them to signed 32-bit integers.
 * The corresponding Arduino library "send" function will send the integer byte by
 * byte in network byte order.  The function requires an ino_connection_t object
 * created using ino_connection_raw() with a 4 bytes per integer frame size,
 * i.e., three 4-byte integers would require a connection frame size of 12 bytes.
 * @param cnx Pointer to ino_connection_t object from which to read data.
 * @param int_buf Pointer to the beginning of an array of int32_t.
 * @param int_count The number of integers to attempt reading <= array length.
 * @return The number of integers read from the serial port.
 * @since 0.1
 */
int
ino_read_int32s(ino_connection_t *cnx, int32_t *int_buf, int int_count)
{
  int n;														/* total bytes received */
  int i=0, j=0, k=0;								/* loop indices */
  ino_int32 tmp;										/* working tmp int */
  int frame_size = int_count * 4;		/* number of bytes in frame */
  unsigned char frame[frame_size];	/* frame buffer */

	/* if connected */
	if ( cnx && cnx->fd > -1 )
	{
		/* prepare raw byte buffer */
		memset(frame,0,frame_size);
		/* receive frame of int's from arduino */
		n = read(cnx->fd, frame, frame_size);
		/* break sets of bytes up into integers */
		/* for each set of bytes corresponding to an arduino int */
		for(i=0; i<n; i+=4)
		{
			/* init tmp to zero */
			tmp.unsig = 0;
			/* for each byte */
			for(j=0; j<4; j++)
			{
				/* shift existing bits to make room for next byte */
				tmp.unsig <<= 8;
				/* add the bits of the next byte to the end of working tmp */
				tmp.unsig |= frame[i+j];
			}
			/* convert to signed int and add to int_buf */
			int_buf[k++] = tmp.sig;
		}
	}

  /* return the number of integers read */
  return k;
}

/**
 * Read multiple unsigned 32-bit integers from an Arduino raw connection at once.
 * Reads 4 bytes per integer from the serial port and converts them from network
 * byte order into host byte order and casts them to unsigned 32-bit integers.
 * The corresponding Arduino library "send" function will send the integer byte by
 * byte in network byte order.  The function requires an ino_connection_t object
 * created using ino_connection_raw() with a 4 bytes per integer frame size,
 * i.e., three 4-byte integers would require a connection frame size of 12 bytes.
 * @param cnx Pointer to ino_connection_t object from which to read data.
 * @param int_buf Pointer to the beginning of an array of uint32_t.
 * @param int_count The number of integers to attempt reading <= array length.
 * @return The number of integers read from the serial port.
 * @since 0.1
 */
int
ino_read_uint32s(ino_connection_t *cnx, uint32_t *int_buf, int int_count)
{
  int n;														/* total bytes received */
  int i=0, j=0, k=0;								/* loop indices */
  ino_int32 tmp;										/* working tmp int */
  int frame_size = int_count * 4;		/* number of bytes in frame */
  unsigned char frame[frame_size];	/* frame buffer */

	/* if connected */
	if ( cnx && cnx->fd > -1 )
	{
		/* prepare raw byte buffer */
		memset(frame,0,frame_size);
		/* receive frame of int's from arduino */
		n = read(cnx->fd, frame, frame_size);
		/* break sets of bytes up into integers */
		/* for each set of bytes corresponding to an arduino int */
		for(i=0; i<n; i+=4)
		{
			/* init tmp to zero */
			tmp.unsig = 0;
			/* for each byte */
			for(j=0; j<4; j++)
			{
				/* shift existing bits to make room for next byte */
				tmp.unsig <<= 8;
				/* add the bits of the next byte to the end of working tmp */
				tmp.unsig |= frame[i+j];
			}
			/* take unsigned int value and add to int_buf */
			int_buf[k++] = tmp.unsig;
		}
	}

  /* return the number of integers read */
  return k;
}

/**
 * Send an array of raw bytes to the Arduino through the serial port.
 * Sends an array of uint8_t through the given ino_connection_t/serial port.
 * @param cnx Pointer to ino_connection_t object through which to send data.
 * @param buf Array of uint8_t bytes to send to the Arduino.
 * @param frame_size_bytes The number of bytes to be sent to the Arduino.
 * @return The number of bytes actually sent, or -1 on error.
 * @since 0.1
 */
int
ino_send_raw(ino_connection_t *cnx, uint8_t *buf, int frame_size_bytes)
{
	int n = 0;

	/* if connected */
	if ( cnx && cnx->fd > -1 )
	{
		n = write(cnx->fd, buf, frame_size_bytes);
	}

	return n;
}

/**
 * Send a single signed 16-bit integer to the Arduino through the serial port.
 * Converts an int16_t to network byte order and sends it byte by byte through
 * the given serial port connection to the Arduino on the other end.  Network
 * byte order raw integers can be read by the Arduino using the corresponding
 * "read" function in the Arduino library, arduino_libuino.h .
 * @param cnx Pointer to ino_connection_t object through which to send data.
 * @param val int16_t to be sent to Arduino.
 * @return The number of bytes actually sent, or -1 on error.
 * @since 0.1
 */
int
ino_send_int16(ino_connection_t *cnx, int16_t val)
{
	int n = 0;
	uint8_t buf[2];
	ino_int16 i;

	/* if connected */
	if ( cnx && cnx->fd > -1 )
	{
		i.sig = val;
		buf[0] = (i.unsig >> 8) & 0xFF;
		buf[1] = i.unsig & 0xFF;
		n = write(cnx->fd, buf, 2);
	}

	return n;
}

/**
 * Send a single unsigned 16-bit integer to the Arduino through the serial port.
 * Converts an uint16_t to network byte order and sends it byte by byte through
 * the given serial port connection to the Arduino on the other end.  Network
 * byte order raw integers can be read by the Arduino using the corresponding
 * "read" function in the Arduino library, arduino_libuino.h .
 * @param cnx Pointer to ino_connection_t object through which to send data.
 * @param val uint16_t to be sent to Arduino.
 * @return The number of bytes actually sent, or -1 on error.
 * @since 0.1
 */
int
ino_send_uint16(ino_connection_t *cnx, uint16_t val)
{
	int n = 0;
	uint8_t buf[2];

	/* if connected */
	if ( cnx && cnx->fd > -1 )
	{
		buf[0] = (val >> 8) & 0xFF;
		buf[1] = val & 0xFF;
		n = write(cnx->fd, buf, 2);
	}

	return n;
}

/**
 * Send a single signed 32-bit integer to the Arduino through the serial port.
 * Converts an int32_t to network byte order and sends it byte by byte through
 * the given serial port connection to the Arduino on the other end.  Network
 * byte order raw integers can be read by the Arduino using the corresponding
 * "read" function in the Arduino library, arduino_libuino.h .
 * @param cnx Pointer to ino_connection_t object through which to send data.
 * @param val int32_t to be sent to Arduino.
 * @return The number of bytes actually sent, or -1 on error.
 * @since 0.1
 */
int
ino_send_int32(ino_connection_t *cnx, int32_t val)
{
	int n = 0;
	uint8_t buf[4];
	ino_int32 i;

	/* if connected */
	if ( cnx && cnx->fd > -1 )
	{
		i.sig = val;
		buf[0] = (i.unsig >> 24) & 0xFF;
		buf[1] = (i.unsig >> 16) & 0xFF;
		buf[2] = (i.unsig >> 8) & 0xFF;
		buf[3] = i.unsig & 0xFF;
		n = write(cnx->fd, buf, 4);
	}

	return n;
}

/**
 * Send a single unsigned 32-bit integer to the Arduino through the serial port.
 * Converts a uint32_t to network byte order and sends it byte by byte through
 * the given serial port connection to the Arduino on the other end.  Network
 * byte order raw integers can be read by the Arduino using the corresponding
 * "read" function in the Arduino library, arduino_libuino.h .
 * @param cnx Pointer to ino_connection_t object through which to send data.
 * @param val uint32_t to be sent to Arduino.
 * @return The number of bytes actually sent, or -1 on error.
 * @since 0.1
 */
int
ino_send_uint32(ino_connection_t *cnx, uint32_t val)
{
	int n = 0;
	uint8_t buf[4];

	/* if connected */
	if ( cnx && cnx->fd > -1 )
	{
		buf[0] = (val >> 24) & 0xFF;
		buf[1] = (val >> 16) & 0xFF;
		buf[2] = (val >> 8) & 0xFF;
		buf[3] = val & 0xFF;
		n = write(cnx->fd, buf, 4);
	}

	return n;
}

/**
 * Send a single char through the serial port.
 * Sends a single character through the serial port to the Arduino.
 * @param c The character to send to the Arduino.
 * @return 1 on success, or -1 on failure.
 * @since 0.1
 */
int ino_send_char(ino_connection_t *cnx, char c)
{
 	int n = -1;

	/* if connected */
	if ( cnx && cnx->fd > -1 )
	{
		n = write(cnx->fd, &c, 1);
		if ( n < 1 ) n = -1;
	}

	return n;
}


#define BUFFER_SIZE 255

int main(int argc, char **argv)
{
	std::string dev = argv[1];

	char *port = const_cast<char *>(dev.c_str());	/* default serial port */
	int baud = B115200;							/* bit rate */
	ino_connection_t *cnx = NULL;	/* arduino connection settings */
	char buffer[BUFFER_SIZE];			/* message buffer */

	/* create a 9600 baud serial connection to arduino */
	cnx = ino_connection_new(port, baud, INO_DEFAULT_WAIT);

	/* open the new connection interface */
	ino_open(cnx);

	/* if port opened successfully */
	std::cout << "Is the serial port open?" << std::endl;
	if ( ino_is_open(cnx) )
	{
		std::cout << " Yes." << std::endl;
    while(true)
    {
      /* read the arduino's response into the buffer */
      ino_read(cnx, buffer, BUFFER_SIZE);
			std::cout << "--------------------" << std::endl;
      std::cout << "readData: " << buffer << std::endl;
    }
	}
	else
	{
		std::cout << "No" << std::endl;
		return 0;
	}

	/* close serial port and free memory */
	ino_connection_destroy(cnx);

	return 0;
}


