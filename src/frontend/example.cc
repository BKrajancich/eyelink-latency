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

#include "display.hh"

#define CURSOR_SIZE 5 /* radius of the white dot in px */
#define NUM_TRIALS 1

using namespace std;
using namespace std::chrono;

/**
 * End recording: adds 100 msec of data to catch final events
 */
static void end_trial( void )
{
  pump_delay( 100 ); // provide a small amount of delay for last data
  stop_recording();
  while ( getkey() ) {
  };
}

int get_tracker_sw_version( char* verstr )
{
  int ln = 0;
  int st = 0;
  ln = strlen( verstr );
  while ( ln > 0 && verstr[ln - 1] == ' ' )
    verstr[--ln] = 0; // trim

  // find the start of the version number
  st = ln;
  while ( st > 0 && verstr[st - 1] != ' ' )
    st--;
  return atoi( &verstr[st] );
}

int initialize_eyelink()
{
  char verstr[50];
  int eyelink_ver = 0;
  int tracker_software_ver = 0;

  // Set the address of the tracker, this is hard-coded by the Eyelink
  set_eyelink_address( (char*)"100.1.1.1" );

  // Initialize the EyeLink DLL and connect to the tracker
  // *  0 opens a connection with the eye tracker
  // *  1 will create a dummy connection for simulation
  // * -1 initializes the DLL but does not open a connection
  if ( open_eyelink_connection( 0 ) )
    return -1;

  set_offline_mode();
  flush_getkey_queue();

  // Now configure tracker for display resolution
  eyecmd_printf( "screen_pixel_coords = %ld %ld %ld %ld", 0, 0, 1920, 1080 );

  eyelink_ver = eyelink_get_tracker_version( verstr );
  if ( eyelink_ver == 3 )
    tracker_software_ver = get_tracker_sw_version( verstr );

  // SET UP TRACKER CONFIGURATION
  // set parser saccade thresholds (conservative settings)
  if ( eyelink_ver >= 2 ) {
    eyecmd_printf( "select_parser_configuration 0" ); // 0 = standard sensitivity
    // turn off scenelink camera stuff
    if ( eyelink_ver == 2 ) {
      eyecmd_printf( "scene_camera_gazemap = NO" );
    }
  } else {
    eyecmd_printf( "saccade_velocity_threshold = 35" );
    eyecmd_printf( "saccade_acceleration_threshold = 9500" );
  }

  // set link data (used for gaze cursor)
  eyecmd_printf( "link_event_filter = LEFT,RIGHT,FIXATION,SACCADE,BLINK,BUTTON,INPUT" );
  eyecmd_printf( "link_sample_data = LEFT,RIGHT,GAZE,GAZERES,AREA,STATUS%s,INPUT",
                 ( tracker_software_ver >= 4 ) ? ",HTARGET" : "" );

  // Make sure we're still alive
  if ( !eyelink_is_connected() || break_pressed() ) {
    return -1;
  }
  return 0;
}

int gc_window_trial()
{
  unsigned int frame_count = 0;

  // First, set up all the textures
  VideoDisplay display { 1920, 1080, true }; // fullscreen window @ 1920x1080 luma resolution
  display.window().hide_cursor( true );

  // whether to wait for vertical retrace before swapping buffer
  // *  0 for immediate updates
  // *  1 for updates synchronized with the vertical retrace
  // * -1 for adaptive vsync
  display.window().set_swap_interval( 0 );

  // Used to track gaze samples
  ALLF_DATA evt;
  float x_sample, y_sample;

  // Ensure Eyelink has enough time to switch modes
  set_offline_mode();
  pump_delay( 50 );

  // Start data streaming
  // Note that we are ignoring the EDF file.
  int error = start_recording( 0, 0, 1, 1 );
  if ( error != 0 ) {
    return error;
  }

  // wait for link sample data
  if ( !eyelink_wait_for_block_start( 100, 1, 0 ) ) {
    end_trial();
    cerr << "ERROR: No link samples received!\n";
    return TRIAL_ERROR;
  }

  // determine which eye(s) are available
  int eye_used = eyelink_eye_available();

  // reset keys and buttons from tracker
  eyelink_flush_keybuttons( 0 );

  const auto start_time = steady_clock::now();
  auto ts_prev = steady_clock::now();

  // Poll for new samples until the diff between samples is large enough to signify LEDs switched
  while ( true ) {
    // check for new sample update
    if ( eyelink_newest_float_sample( NULL ) > 0 ) {
      eyelink_newest_float_sample( &evt );

      x_sample = evt.fs.gx[eye_used];
      y_sample = evt.fs.gy[eye_used];

      // make sure pupil is present
      if ( x_sample != MISSING_DATA && y_sample != MISSING_DATA && evt.fs.pa[eye_used] > 0 ) {
        // TODO: Draw a dot where we are looking
        /* top left box white (235 = max luma in typical Y'CbCr colorspace) */
        Raster420 cursor { 1920, 1080 };

        for ( unsigned int y = 0; y < cursor.Y.height(); y++ ) {
          for ( unsigned int x = 0; x < cursor.Y.width(); x++ ) {
            // Check if point is within radius
            bool within_circle = ( ( ( x_sample - x ) * ( x_sample - x ) ) +
                                   ( ( y_sample - y ) * ( y_sample - y ) ) ) <= ( CURSOR_SIZE * CURSOR_SIZE );
            const uint8_t color = within_circle ? 235 : 16;
            cursor.Y.at( x, y ) = color;
          }
        }

        Texture420 cursor_texture { cursor };

        const auto ts = steady_clock::now();
        const auto tdiff = duration_cast<milliseconds>( ts - ts_prev ).count();
        if ( tdiff >= 4 ) {
          // Draw texture. Note this may introduce extra delay since this is
          // computing a texture each update.
          display.draw( cursor_texture );

          frame_count++;
          ts_prev = ts;

          if ( frame_count % 480 == 0 ) {
            const auto now = steady_clock::now();
            const auto ms_elapsed = duration_cast<milliseconds>( now - start_time ).count();
            cout << "Drew " << frame_count << " frames in " << ms_elapsed
                 << " milliseconds = " << 1000.0 * double( frame_count ) / ms_elapsed << " frames per second.\n";
          }
        }
      }
    }
  }
  end_trial();
  return check_record_exit();
}

int run_trials()
{
  for ( unsigned int trial = 0; trial < NUM_TRIALS; trial++ ) {
    // abort if link is closed
    if ( eyelink_is_connected() == 0 || break_pressed() ) {
      return ABORT_EXPT;
    }

    int i = gc_window_trial();

    // Report errors
    switch ( i ) {
      case ABORT_EXPT: // handle experiment abort or disconnect
        cout << "EXPERIMENT ABORTED\n";
        return ABORT_EXPT;
      case REPEAT_TRIAL: // trial restart requested
        cout << "TRIAL REPEATED\n";
        trial--;
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
  }

  return 0;
}

void program_body()
{
  if ( initialize_eyelink() < 0 ) {
    cerr << "[Error] Unable to initialize EyeLink.\n";
    exit( EXIT_FAILURE );
  }
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
