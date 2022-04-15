/*! \file defs.cpp
 * \brief Time class implementation..
 *
 * \authors Roman Sukhanov
 * \date March 2022
 */

//=======================================================================================

#include "defs.hpp"

using namespace std;

//=======================================================================================
void error_out( const std::string & str, int error_code )
{
    cerr << "\033[1;31m ERROR. " << str << " !!! \033[0m" << endl;
    exit( error_code );
}
void warning( const std::string & str )
{
    cout << "\033[1;33m WARNING. " << str << " ! \033[0m" << endl;
}

//=======================================================================================

vtime_point Time::getTimestamp()
{
    auto time_now = vtime_point::now();
    
    // +3 MSK
//    time_t now = time(0);
//    tm *ltm = localtime(&now);
//    time_now += std::chrono::seconds(ltm->tm_gmtoff);  // +3 hours (+10800 sec) time zone
    
    return time_now;
}

//=======================================================================================

std::string Time::getISOCurrentTimestamp()
{
    auto time_now = vtime_point::now();
    
    // +3 MSK
//    time_t now = time(0);
//    tm *ltm = localtime(&now);
//    time_now += std::chrono::seconds(ltm->tm_gmtoff);  // +3 hours (+10800 sec) time zone
    
    return (time_now.humanable().iso8601_zzz()); //  + "Z"
}

//=======================================================================================

int64_t Time::getUnixTimestamp()
{
    auto time_now = vtime_point::now();
    
    // +3 MSK
//    time_t now = time(0);
//    tm *ltm = localtime(&now);
//    time_now += std::chrono::seconds(ltm->tm_gmtoff);  // +3 hours (+10800 sec) time zone
    
    return time_now.milliseconds().count();
}

//=======================================================================================
