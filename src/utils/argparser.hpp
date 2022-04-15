#ifndef HPP_ARGPARSER
#define HPP_ARGPARSER

#include <string>
#include <map>
#include <algorithm>
#include <iostream>  // cut
#include <fstream>  // cut

namespace Argparser
{
    const bool Sufficient = true;
    const bool NotSufficient = false;

    typedef std::map< std::string, const bool > ArgumentsDict;

    void exit_with_msg( int code )
    {
        std::cerr << "Bad arguments.";
        exit(code);
    }

    std::map< std::string, std::string > parse(
        int argc, const char** argv,
        ArgumentsDict arg_dict,
        int errcode=22 )
    {
        std::map< std::string, std::string > parsed_arguments;

        for ( int i = 1; i < argc; i++ )
        {
            auto arg_iterator = std::find_if(
                arg_dict.begin(), arg_dict.end(),
                [ i, argv ]( ArgumentsDict::value_type& arg )
                {
                    return arg.first == argv[i];
                });

            if ( arg_iterator == arg_dict.end() )
            {
                continue;
            }

            if ( arg_iterator->second == NotSufficient and i >= argc-1)
                exit_with_msg(errcode);

            parsed_arguments.insert( { arg_iterator->first, "" } );

            if ( arg_iterator->second == NotSufficient )
            {
                i ++;
                parsed_arguments[ arg_iterator->first ] = argv[i];
            }
        }

        return parsed_arguments;
    }
}

#endif
