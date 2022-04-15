/*! \file defs.hpp
 * \brief List of common definitions.
 *
 * \authors Roman Sukhanov
 * \date March 2022
 */

//=======================================================================================

#pragma once

// STD
#include <algorithm>
#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <array>
#include <map>

// Plane analysis
//#include "plane_worker.h"

// vlibs2
//#include "vapplication.h"
#include "vbyte_buffer.h"
#include "vtime_point.h"
#include "vsettings.h"
#include "vthread.h"
#include "vsignal.h"
#include "vtimer.h"
#include "vlog.h"
#include "vzcm.h"

// ZCM
#include "ZcmBoundBoxesBatch.hpp"
#include "ZcmBoundBoxesMsg.hpp"
#include "ZcmBoundBox.hpp"
#include "ZcmObjectClass.hpp"
#include "ZcmDepthDetectMsg.hpp"
#include "ZcmDepthObjectsBatch.hpp" 
#include "ZcmDepthObjectsMsg.hpp"
#include "ZcmDepthObjectBoundBox.hpp"
#include "ZcmDepthObject.hpp"

#include "ZcmOwen.hpp"
#include "ZcmOwenPorts.hpp"
#include "ZcmOwenPortsState.hpp"
#include "ZcmSensorInfo.hpp"
#include "ZcmSensorType.hpp"
#include "ZcmService.hpp"
#include "ZcmTextFormat.hpp"
#include "ZcmTextData.hpp"

// Computer Vision
#include <opencv2/core.hpp>

//=======================================================================================

#define NUM_TRACK 2  // number of tracks
#define NUM_GAP 4    // number of intercar space
#define NUM_CAR 5    // number of cars
#define NUM_DOOR 2   // number of doors in which car
#define NUM_SIDE 2   // number of outside space

static constexpr float nn_threshold_default = 0.55;     // Default neural network threshold
static constexpr int timeout_get_status_s = 10;         // seconds
static constexpr int hold_empty_msg_s = 6;              // Hold empty msg before reset zone, seconds
static constexpr int hold_empty_msg_num = 10;           // Number of empty msg before add error in output msg
static constexpr int hold_time_ms = 1000;               // Hold time of data, milliseconds

//=======================================================================================

enum ZONES {
    NO_ZONE = 0,
//    RED = 1,
    TRACK = 1,
    INTERCAR = 1,
//    YELLOW = 2,
    EDGE = 2,
    OUTSIDE = 2,
    
    RED_0 = 3,
    E = 3,
    I = 3,
    RED_1 = 4,
    G = 4,
    YELLOW_0 = 5,
    A = 5,
    YELLOW_1 = 6,
    F = 6,
    YELLOW_2 = 7,
    H = 7
};

enum STATE
{
    NONE = -2,              // 
    CONFIGURATION = -1,     // A-  B-  C-  D-  E-  F-  G-  H-  I-  transmit all zones as UNKNOWN
    DEBUG = 0,              // A+  B+  C+  D+  E+  F+  G+  H+  I+  transmit all zones without changes
    EMPTY = 1,              // A-  B-  C-  D+  E+  F-  G+  H+  I-  | B->E, C->E, I->E
    ARRIVAL = 2,            // A+  B-  C-  D+  E+  F+  G+  H+  I-  | B->E, C->E, I->E
//    STOPING = 3,            // A+  B-  C-  D+  E+  F+  G+  H+  I-  | B->E, C->E, I->E
    BOARDING = 4,           // A-  B-  C+  D+  E+  F-  G+  H+  I+  
//    CLOSING_DOORS = 5,      // A-  B+  C+  D+  E+  F-  G+  H+  I+
    LEAVING = 6             // A+  B-  C-  D+  E+  F+  G+  H+  I-  | B->E, C->E, I->E
};

enum ERROR
{
    EMPTY_NN = 1,
    EMPTY_NN_THERMAL = 2,
    EMPTY_DEPTH_DETECT = 3,
    EMPTY_LASER = 4,
    
    OPEN_SERVER_DOOR = 6
};

enum WARNINGS
{
    POWER_OFF = 1,
    SPEAKER_OFF = 2,
    INTERNAL_STATE_OFF = 3
};

enum MOTION
{
    MOTION_ENABLED = 0, 
    SPEED_RESTRICTION = 1, 
    MOTION_DISABLED = 2
};

enum VOICE 
{
    /** @enum VOICE
     *  @brief In order of priority. Number <-> Audio_file
     */
    OCCUPY_TRACK = 1,
    OCCUPY_EDGE = 2,
    OCCUPY_DOOR = 3,
    OCCUPY_INTERCAR = 4,
    OCCUPY_UNDER = 5,
    OCCUPY_OUTSIDE = 6,
    OPEN_SERVER = 7,
    DANGEROUS = 8,
    SILENCE = 28
};

enum LOC
{
    /** @enum LOC
     *  @brief Location of speaker
     */
    TRACK_1 = 0,
    TRACK_2 = 1,
    CAR_1 = 2,
    CAR_2 = 3,
    CAR_3 = 4,
    CAR_4 = 5,
    CAR_5 = 6,
    GAP_1 = 7,
    GAP_2 = 8,
    GAP_3 = 9,
    GAP_4 = 10,
    LEFT_SIDE = 11,
    RIGHT_SIDE = 12,
    SERVER_1 = 13,
    SERVER_2 = 14
};


//=======================================================================================
/*!
 *  \addtogroup loc
 *  @{
 */

//! Location for speaker and vioce message 
namespace loc {
//! 1 <= idx <= 2, first and second track 
static inline uint8_t track( uint8_t idx ) { if (idx<1 || idx>2) throw -1; return idx - 1; }
//! 1 <= idx <= 5, number of car 
static inline uint8_t car( uint8_t idx ) { if (idx<1 || idx>5) throw -1; return idx + 1; }
//! 1 <= idx <= 4, number intercar spice
static inline uint8_t gap( uint8_t idx ) { if (idx<1 || idx>4) throw -1; return idx + 6; }
//! 1 <= idx <= 2, 1 - left outside, 2 - right outside
static inline uint8_t side( uint8_t idx ) { if (idx<1 || idx>2) throw -1; return idx + 10; }
//! 1 <= idx <= 2, first and second server (track control)
static inline uint8_t server( uint8_t idx ) { if (idx<1 || idx>2) throw -1;return idx + 12; }
} // End namespace loc

/*! @} End of Doxygen Groups*/

//=======================================================================================

/*! \brief Data interpretation
//# Sensor sheme type: 1 - (LEFT - back, RIGHT - front), 2 - (LEFT - front, RIGHT - back)
//# All config and all analyze data convert to type 2 
//#========================================================================================
//# --- 1 ---             _____________________station_____________________               |
//#            |   side  |  doors  |  doors  |  doors  |  doors  |  doors  |  side        |
//#  direction |         |   car   |   car   |   car   |   car   |   car   |              |
//#            |         |_______inter__|__inter__|__inter__|__inter__|____|              |
//#----------------------------------------------------------------------------------------
//#            |   left  |  2 | 1  |  2 | 1  |  2 | 1  |  2 | 1  |  2 | 1  |  right       |
//#      -->   |  (back) |    5    |    4    |    3    |    2    |    1    | (front)      |
//#            |         |_________4_________3_________2_________1_________|              |
//#                                             track                                     |
//#========================================================================================
//# --- 2 ---             _____________________station_____________________               |
//#            |   side  |  doors  |  doors  |  doors  |  doors  |  doors  |  side        |
//#  direction |         |   car   |   car   |   car   |   car   |   car   |              |
//#            |         |_______inter__|__inter__|__inter__|__inter__|____|              |
//#----------------------------------------------------------------------------------------
//#            |   left  |  1 | 2  |  1 | 2  |  1 | 2  |  1 | 2  |  1 | 2  |  right       |
//#      <--   | (front) |    1    |    2    |    3    |    4    |    5    |  (back)      |
//#            |         |_________1_________2_________3_________4_________|              |
//#                                             track                                     |
//#========================================================================================
*/

//=======================================================================================
//! \brief Warning and error out
void warning( const std::string & str );
void error_out( const std::string & str, int error_code = 1 );

//=======================================================================================

//! \brief For storing timestamps
struct Time final
{
public:
    
    //! \brief Generates a timestamp  
    //! and takes into account the time zone UTC.
    static vtime_point getTimestamp();
    
    //! \brief Generates a timestamp in iso8601 format 
    //! and takes into account the time zone UTC.
    static std::string getISOCurrentTimestamp();
    
    //! \brief Generates a timestamp in Unix format (microseconds)
    //! and takes into account the time zone UTC.
    static int64_t getUnixTimestamp();
    
    //-----------------------------------------------------------------------------------
    
    //! \brief Data capture time by device
    vtime_point raw { std::chrono::microseconds(0) };

    //! \brief Data capture time by user
    vtime_point rec { std::chrono::microseconds(0) };
};

//=======================================================================================
