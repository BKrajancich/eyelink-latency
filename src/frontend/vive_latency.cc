#include <atomic>
#include <chrono>
#include <cstring>
#include <exception>
#include <fstream>
#include <iostream>
#include <thread>

#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#include <core_expt.h>
#include <eyelink.h>
#include <sdl_expt.h>

#include "display.hh"

#define BOX_DIM 100    /* Dimensions of the white square */
#define DIFF_THRESH 25 /* Abs diff for x or y to change before trigger */
#define SERIAL "/dev/ttyACM0"
#define BAUD B115200
#define NUM_TRIALS 1

using namespace std;
using namespace std::chrono;

/**
 * Setup the serial port interface attributes to 8-bit, no parity, 1 stop bit.
 *
 * @param fd    File descriptor of the serial port.
 * @param speed The baud rate to use.
 */

int set_interface_attribs( int fd, int speed )
{
  struct termios tty;

  if ( !isatty( fd ) ) {
    printf( "fd is not a TTY\n" );
    return -1;
  }

  if ( tcgetattr( fd, &tty ) < 0 ) {
    printf( "Error from tcgetattr: %s\n", strerror( errno ) );
    return -1;
  }

  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;      // 8-bit characters
  tty.c_cflag &= ~PARENB;  // no parity bit
  tty.c_cflag &= ~CSTOPB;  // only need 1 stop bit
  tty.c_cflag &= ~CRTSCTS; // no hardware flowcontrol

  tty.c_lflag |= ICANON | ISIG; // canonical input
  tty.c_lflag &= ~( ECHO | ECHOE | ECHONL | IEXTEN );

  tty.c_iflag &= ~IGNCR; // preserve carriage return
  tty.c_iflag &= ~INPCK;
  tty.c_iflag &= ~( INLCR | ICRNL | IUCLC | IMAXBEL );
  tty.c_iflag &= ~( IXON | IXOFF | IXANY ); // no SW flowcontrol

  tty.c_oflag &= ~OPOST;

  tty.c_cc[VEOL] = 0;
  tty.c_cc[VEOL2] = 0;
  tty.c_cc[VEOF] = 0x04;

  if ( cfsetospeed( &tty, speed ) < 0 || cfsetispeed( &tty, speed ) < 0 ) {
    printf( "unable to set correct baud rates.\n" );
    return -1;
  }

  if ( tcsetattr( fd, TCSANOW, &tty ) != 0 ) {
    printf( "Error from tcsetattr: %s\n", strerror( errno ) );
    return -1;
  }
  return 0;
}

int run_measurement( ofstream& log, int arduino ) {
  std::cout << "trial triggered" << std::endl;
    // Send Arduino the command to switch LEDs
  int wlen = write( arduino, "g", 1 );
  if ( wlen != 1 ) {
    std::cout << "[Error] Unable to send to arduino.\n";
    cerr << "[Error] Unable to send to arduino.\n";
    return TRIAL_ERROR;
  }
  tcdrain( arduino );
  return TRIAL_OK;
}


int run_trials() {
  // To communicate with arduino over serial
  int arduino;

  // Open the serial port to the Arduino
  arduino = open( (char*)SERIAL, O_RDWR | O_NOCTTY | O_SYNC );
  if ( arduino < 0 ) {
    printf( "Error opening %s: %s\n", SERIAL, strerror( errno ) );
    return ABORT_EXPT;
  }

  // baudrate 115200, 8 bits, no parity, 1 stop bit
  if ( set_interface_attribs( arduino, BAUD ) != 0 ) {
    printf( "Error setting serial interface attribs.\n" );
    close( arduino );
    return ABORT_EXPT;
  }

  // Arduino Uno uses DTR line to trigger a reset, so wait for it to boot fully.
  sleep( 5 );


  ofstream log;

  log.open( "results.csv" );
  log << "e2e (us), eyelink (us)\n";

  int i = run_measurement( log, arduino );

  // Report errors
  switch ( i ) {
    case ABORT_EXPT: // handle experiment abort or disconnect
      cout << "EXPERIMENT ABORTED\n";
      close( arduino );
      log.close();
      return ABORT_EXPT;
    case REPEAT_TRIAL: // trial restart requested
      cout << "TRIAL REPEATED\n";
      //trial--;
      break;
    case SKIP_TRIAL: // skip trial
      cout << "TRIAL ABORTED\n";
      break;
    case TRIAL_OK: // successful trial
      cout << "TRIAL OK\n";
      break;
    default: // other error code
      cout << "TRIAL ERROR\n";
      break;
  }

    // clean up
  close( arduino );
  log.close();

  return 0;
}

void program_body()
{
  std::cout << "starting experiment" << std::endl;
  run_trials();
}

int main()
{
  try {
    program_body();
  } catch ( const exception& e ) {
    cerr << "Exception: " << e.what() << "\n";
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
