/*
 * Log.h
 *
 * Created on: 2013. 6. 7.
 * Author: Hae Won Park
 * Description: Declaration and implementation for Logging.
 * Last modified: 2013. 6. 8.
 */

#ifndef _LOG_MODULE_H_
#define _LOG_MODULE_H_

#define DEBUG

#include <stdio.h>
#include <string.h>
#include <ctime>
#include <fstream>

class LOG{
    
public:
    static void write_log(const std::string &text){
        std::ofstream log_file("log_file.txt", std::ios_base::out | std::ios_base::app );
        
        //                time_t ltime = time(NULL);
        //                log_file << asctime(localtime(&ltime)) << "  " << text << std::endl;
        log_file << text << std::endl;
    };
    
};

#endif
