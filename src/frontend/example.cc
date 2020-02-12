#include <atomic>
#include <chrono>
#include <cstring>
#include <exception>
#include <fstream>
#include <iostream>
#include <thread>
#include <stdlib.h>

#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#include <core_expt.h>
#include <eyelink.h>
#include <sdl_expt.h>

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

int exit_eyelink()
{
  close_expt_graphics();      // tell EXPTSPPT to release window
  close_eyelink_connection(); // disconnect from tracker
  return 0;
}

void clear_full_screen_window( SDL_Surface* window, SDL_Color c )
{
  SDL_FillRect( window, NULL, SDL_MapRGB( window->format, c.r, c.g, c.b ) );
  SDL_Flip( window );
  SDL_FillRect( window, NULL, SDL_MapRGB( window->format, c.r, c.g, c.b ) );
}

void do_calibration()
{
  // The colors of the target and background for calibration and drift correct
  SDL_Color target_background_color = { 192, 192, 192, 0 };
  SDL_Color target_foreground_color = { 0, 0, 0, 0 };
  SDL_Surface* window = NULL;

  // register window with EXPTSPPT
  if ( init_expt_graphics( NULL, NULL ) ) {
    exit_eyelink();
    return;
  }

  // Setup calibration type
  eyecmd_printf( "calibration_type = HV9" );

  window = SDL_GetVideoSurface();

  // Size for calibration target and focal spot
  unsigned int i = 1920 / 60;
  unsigned int j = 1920 / 300;
  if ( j < 2 )
    j = 2;
  set_target_size( i, j ); // tell DLL the size of target features

  // tell EXPTSPPT the colors
  set_calibration_colors( &target_foreground_color, &target_background_color );

  clear_full_screen_window( window, target_background_color );

  SDL_Flip( window );

  do_tracker_setup();

  // Wait for user to press escape before continuing
  while ( !escape_pressed() ) {
  };

  close_expt_graphics(); // tell EXPTSPPT to release window
}

int gc_window_trial()
{
  ofstream log;
  log.open( "results.csv" );
  unsigned int frame_count = 0;
  int button; /* the button pressed (0 if timeout)  */

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
    log.close();
    return error;
  }

  // wait for link sample data
  if ( !eyelink_wait_for_block_start( 100, 1, 0 ) ) {
    end_trial();
    cerr << "ERROR: No link samples received!\n";
    log.close();
    return TRIAL_ERROR;
  }

  // determine which eye(s) are available
  int eye_used = eyelink_eye_available();

  // reset keys and buttons from tracker
  eyelink_flush_keybuttons( 0 );

  const auto start_time = steady_clock::now();

  // Poll for new samples, update rendered circle
  while ( true ) {
    // Termination conditions
    if ( ( error = check_recording() ) != 0 ) {
      log.close();
      return error;
    }

    if ( break_pressed() ) /* check for program termination or ALT-F4 or CTRL-C keys */
    {
      end_trial();       /* local function to stop recording */
      log.close();
      return ABORT_EXPT; /* return this code to terminate experiment */
    }

    if ( escape_pressed() ) /* check for local ESC key to abort trial (useful in debugging)    */
    {
      end_trial();       /* local function to stop recording */
      log.close();
      return SKIP_TRIAL; /* return this code if trial terminated */
    }

    /* BUTTON RESPONSE TEST */
    /* Check for eye-tracker buttons pressed */
    /* This is the preferred way to get response data or end trials	 */
    button = eyelink_last_button_press( NULL );
    if ( button != 0 ) /* button number, or 0 if none pressed */
    {
      end_trial(); /* local function to stop recording */
      break;       /* exit trial loop */
    }

    // check for new sample update
    if ( eyelink_newest_float_sample( NULL ) > 0 ) {
      eyelink_newest_float_sample( &evt );

      x_sample = evt.fs.gx[eye_used];
      y_sample = evt.fs.gy[eye_used];

      const auto t_sample = duration_cast<microseconds>( steady_clock::now() - start_time ).count();
      log << t_sample << ", " << x_sample << ", " << y_sample << endl;

      // make sure pupil is present
      if ( x_sample != MISSING_DATA && y_sample != MISSING_DATA && evt.fs.pa[eye_used] > 0 ) {
        // TODO: Draw a dot where we are looking. This is currently very naive,
        // we should be able to draw a dot faster. Right now, this is limiting
        // our FPS to about 77fps.
       
        /* draw cursor location (235 = max luma in typical Y'CbCr colorspace) */
        Raster420 cursor { 1920, 1080 };

        for ( unsigned int y = 0; y < cursor.Y.height(); y++ ) {
          for ( unsigned int x = 0; x < cursor.Y.width(); x++ ) {
            // Check if point is within radius
            // bool within_circle = ( ( ( x_sample - x ) * ( x_sample - x ) ) +
            //                        ( ( y_sample - y ) * ( y_sample - y ) ) ) <= ( CURSOR_SIZE * CURSOR_SIZE );
            bool within_circle = (( ( ( x_sample - x ) * ( x_sample - x ) ) +
                                   ( ( y_sample - y ) * ( y_sample - y ) ) ) >= ( CURSOR_SIZE * CURSOR_SIZE )) &&
                                  ( ( ( x_sample - x ) * ( x_sample - x ) ) +
                                   ( ( y_sample - y ) * ( y_sample - y ) ) ) <= ( 2* CURSOR_SIZE * 2*CURSOR_SIZE );;
            const uint8_t color = within_circle ? 235 : 16;
            cursor.Y.at( x, y ) = color;
          }
        }

        Texture420 cursor_texture { cursor };

        // Draw texture. Note this may introduce extra delay since this is
        // computing a texture each update.
        display.draw( cursor_texture );

        frame_count++;

        if ( frame_count % 480 == 0 ) {
          const auto now = steady_clock::now();
          const auto ms_elapsed = duration_cast<milliseconds>( now - start_time ).count();
          cout << "Drew " << frame_count << " frames in " << ms_elapsed
               << " milliseconds = " << 1000.0 * double( frame_count ) / ms_elapsed << " frames per second.\n";
        }
      }
    }
  }
  end_trial();
  log.close();
  return check_record_exit();
}

Raster420 make_texture(int target1_x, int target1_y)
{
   Raster420 cursor1 { 1920, 1080 };

        for ( unsigned int y = 0; y < cursor1.Y.height(); y++ ) {
          for ( unsigned int x = 0; x < cursor1.Y.width(); x++ ) {
            // Check if point is within radius
            bool within_circle = (( ( ( target1_x - x ) * ( target1_x - x ) ) +
                                   ( ( target1_y - y ) * ( target1_y - y ) ) ) <= ( 10 * 10 )) &&
            (( ( ( target1_x - x ) * ( target1_x - x ) ) +
                                   ( ( target1_y - y ) * ( target1_y - y ) ) ) >= ( 2 * 2));
            const uint8_t color = within_circle ? 235 : 16;
            cursor1.Y.at( x, y ) = color;
          }
        }

    //Texture420 cursor1_texture { cursor1 };
    return cursor1;
}

int accuracy_test()
{
  ofstream log;
  log.open( "Luke1.csv" );
  unsigned int frame_count = 0;
  int button; /* the button pressed (0 if timeout)  */
  int current_x = 0;
  int current_y = 0;

  int target1_x = (1920/2) - 270;
  int target1_y = (1080/2) - 270;
  int target2_x = (1920/2) + 270;
  int target2_y = (1080/2) - 270;
  int target3_x = (1920/2) - 270;
  int target3_y = (1080/2) + 270;
  int target4_x = (1920/2) + 270;
  int target4_y = (1080/2) + 270;
  int target5_x = 1920/2;
  int target5_y = 1080/2;

  int t_last = 0;
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
    log.close();
    return error;
  }

  // wait for link sample data
  if ( !eyelink_wait_for_block_start( 100, 1, 0 ) ) {
    end_trial();
    cerr << "ERROR: No link samples received!\n";
    log.close();
    return TRIAL_ERROR;
  }

  // determine which eye(s) are available
  int eye_used = eyelink_eye_available();

  // reset keys and buttons from tracker
  eyelink_flush_keybuttons( 0 );

  bool done = false;

  const auto start_time = steady_clock::now();

  // Poll for new samples, update rendered circle
  while ( !done ) {
    // Termination conditions
    if ( ( error = check_recording() ) != 0 ) {
      log.close();
      return error;
    }

    if ( break_pressed() ) /* check for program termination or ALT-F4 or CTRL-C keys */
    {
      end_trial();       /* local function to stop recording */
      log.close();
      return ABORT_EXPT; /* return this code to terminate experiment */
    }

    if ( escape_pressed() ) /* check for local ESC key to abort trial (useful in debugging)    */
    {
      end_trial();       /* local function to stop recording */
      log.close();
      return SKIP_TRIAL; /* return this code if trial terminated */
    }

    /* BUTTON RESPONSE TEST */
    /* Check for eye-tracker buttons pressed */
    /* This is the preferred way to get response data or end trials  */
    button = eyelink_last_button_press( NULL );
    if ( button != 0 ) /* button number, or 0 if none pressed */
    {
      end_trial(); /* local function to stop recording */
      break;       /* exit trial loop */
    } 

    // check for new sample update
    if ( eyelink_newest_float_sample( NULL ) > 0 ) {
      eyelink_newest_float_sample( &evt );

      x_sample = evt.fs.gx[eye_used];
      y_sample = evt.fs.gy[eye_used];

      const auto t_sample = duration_cast<microseconds>( steady_clock::now() - start_time ).count();
      log << t_sample << ", " << x_sample << ", " << y_sample << ", " << current_x << ", " << current_y << endl;

      // make sure pupil is present
      if ( x_sample != MISSING_DATA && y_sample != MISSING_DATA && evt.fs.pa[eye_used] > 0 ) {
        // TODO: Draw a dot where we are looking. This is currently very naive,
        // we should be able to draw a dot faster. Right now, this is limiting
        // our FPS to about 77fps.
       
        ///////// CURSOR 1 /////////////////////////////////////////////////////////////////////////////////////////////////
        /* draw cursor location (235 = max luma in typical Y'CbCr colorspace) */
        Raster420 cursor1 { 1920, 1080 };

        for ( unsigned int y = 0; y < cursor1.Y.height(); y++ ) {
          for ( unsigned int x = 0; x < cursor1.Y.width(); x++ ) {
            // Check if point is within radius
            bool within_circle = (( ( ( target1_x - x ) * ( target1_x - x ) ) +
                                   ( ( target1_y - y ) * ( target1_y - y ) ) ) <= ( 10 * 10 )) &&
                                (( ( ( target1_x - x ) * ( target1_x - x ) ) +
                                   ( ( target1_y - y ) * ( target1_y - y ) ) ) >= ( 2 * 2));
            const uint8_t color = within_circle ? 235 : 16;
            cursor1.Y.at( x, y ) = color;
          }
        }

        Texture420 cursor1_texture { cursor1 };

        ///////// CURSOR 2 /////////////////////////////////////////////////////////////////////////////////////////////////
        /* draw cursor location (235 = max luma in typical Y'CbCr colorspace) */
        Raster420 cursor2 { 1920, 1080 };

        for ( unsigned int y = 0; y < cursor2.Y.height(); y++ ) {
          for ( unsigned int x = 0; x < cursor2.Y.width(); x++ ) {
            // Check if point is within radius
            bool within_circle = (( ( ( target2_x - x ) * ( target2_x - x ) ) +
                                   ( ( target2_y - y ) * ( target2_y - y ) ) ) <= ( 10 * 10 )) &&
                                (( ( ( target2_x - x ) * ( target2_x - x ) ) +
                                   ( ( target2_y - y ) * ( target2_y - y ) ) ) >= ( 2 * 2));
            const uint8_t color = within_circle ? 235 : 16;
            cursor2.Y.at( x, y ) = color;
          }
        }

        Texture420 cursor2_texture { cursor2 };

        ///////// CURSOR 3 /////////////////////////////////////////////////////////////////////////////////////////////////
        /* draw cursor location (235 = max luma in typical Y'CbCr colorspace) */
        Raster420 cursor3 { 1920, 1080 };

        for ( unsigned int y = 0; y < cursor3.Y.height(); y++ ) {
          for ( unsigned int x = 0; x < cursor3.Y.width(); x++ ) {
            // Check if point is within radius
            bool within_circle = (( ( ( target3_x - x ) * ( target3_x - x ) ) +
                                   ( ( target3_y - y ) * ( target3_y - y ) ) ) <= ( 10 * 10 )) &&
                                (( ( ( target3_x - x ) * ( target3_x - x ) ) +
                                   ( ( target3_y - y ) * ( target3_y - y ) ) ) >= ( 2 * 2));
            const uint8_t color = within_circle ? 235 : 16;
            cursor3.Y.at( x, y ) = color;
          }
        }

        Texture420 cursor3_texture { cursor3 };

        ///////// CURSOR 4 /////////////////////////////////////////////////////////////////////////////////////////////////
        /* draw cursor location (235 = max luma in typical Y'CbCr colorspace) */
        Raster420 cursor4 { 1920, 1080 };

        for ( unsigned int y = 0; y < cursor4.Y.height(); y++ ) {
          for ( unsigned int x = 0; x < cursor4.Y.width(); x++ ) {
            // Check if point is within radius
            bool within_circle = (( ( ( target4_x - x ) * ( target4_x - x ) ) +
                                   ( ( target4_y - y ) * ( target4_y - y ) ) ) <= ( 10 * 10 )) &&
                                (( ( ( target4_x - x ) * ( target4_x - x ) ) +
                                   ( ( target4_y - y ) * ( target4_y - y ) ) ) >= ( 2 * 2));
            const uint8_t color = within_circle ? 235 : 16;
            cursor4.Y.at( x, y ) = color;
          }
        }

        Texture420 cursor4_texture { cursor4 };

        ///////// CURSOR 5 /////////////////////////////////////////////////////////////////////////////////////////////////
        /* draw cursor location (235 = max luma in typical Y'CbCr colorspace) */
        Raster420 cursor5 { 1920, 1080 };

        for ( unsigned int y = 0; y < cursor5.Y.height(); y++ ) {
          for ( unsigned int x = 0; x < cursor5.Y.width(); x++ ) {
            // Check if point is within radius
            bool within_circle = (( ( ( target5_x - x ) * ( target5_x - x ) ) +
                                   ( ( target5_y - y ) * ( target5_y - y ) ) ) <= ( 10 * 10 )) &&
                                (( ( ( target5_x - x ) * ( target5_x - x ) ) +
                                   ( ( target5_y - y ) * ( target5_y - y ) ) ) >= ( 2 * 2));
            const uint8_t color = within_circle ? 235 : 16;
            cursor5.Y.at( x, y ) = color;
          }
        }

        Texture420 cursor5_texture { cursor5 };

        const auto tt = duration_cast<seconds>( steady_clock::now() - start_time ).count();

        cout << tt << endl;
        if ( tt % 2 == 0 && tt != t_last) {
          t_last = tt;
          int r = rand() % 5;
          if (r == 0) {
            display.draw( cursor1_texture );
            current_y = target1_y;
            current_x = target1_x;
          } else if (r == 1) {
            display.draw( cursor2_texture );
            current_y = target2_y;
            current_x = target2_x;
          } else if (r == 2) {
            display.draw( cursor3_texture );
            current_y = target3_y;
            current_x = target3_x;
          } else if (r == 3) {
            display.draw( cursor4_texture );
            current_y = target4_y;
            current_x = target4_x;
          } else {
            display.draw ( cursor5_texture );
            current_y = target5_y;
            current_x = target5_x;
          }
        }

        // Draw texture. Note this may introduce extra delay since this is
        // computing a texture each update.
        //display.draw( cursor1_texture );

        frame_count++;

        if ( frame_count % 480 == 0 ) {
          const auto now = steady_clock::now();
          const auto ms_elapsed = duration_cast<milliseconds>( now - start_time ).count();
          cout << "Drew " << frame_count << " frames in " << ms_elapsed
               << " milliseconds = " << 1000.0 * double( frame_count ) / ms_elapsed << " frames per second.\n";
        }
      }
    }
  }
  end_trial();
  log.close();
  return check_record_exit();
}

int run_trials()
{

  do_calibration();

  for ( unsigned int trial = 0; trial < NUM_TRIALS; trial++ ) {
    // abort if link is closed
    if ( eyelink_is_connected() == 0 || break_pressed() ) {
      return ABORT_EXPT;
    }

    //int i = gc_window_trial();
    int i = accuracy_test();

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
