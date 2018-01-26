/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture                          *
*                (c) 2006-2018 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                           Plugin SoftRobots    v1.0                         *
*				                                              *
* This plugin is also distributed under the GNU LGPL (Lesser General          *
* Public License) license with the same conditions than SOFA.                 *
*                                                                             *
* Contributors: Defrost team  (INRIA, University of Lille, CNRS,              *
*               Ecole Centrale de Lille)                                      *
*                                                                             *
* Contact information: https://project.inria.fr/softrobot/contact/            *
*                                                                             *
******************************************************************************/
#ifndef _NETWORK_HEADER_
#define _NETWORK_HEADER_

#if defined(_WIN32) || defined(_WIN64)
    #include "SysTimeWin.h"
#else
    #include <sys/time.h>
    #include <stdbool.h>
    //stdbool not recognized by Visual Studio 2012 but booleans work without it
#endif
#include <stdlib.h> 

#include "SplitFloat.h"

#define BUFFER_MAX_SIZE     30
#define NB_MSG_TYPE         10

#define UNCHANGED           0xFFFFFFFF

#define REQ_OK              1
#define REQ_KO              0

typedef enum {set_mode_inhibition, set_setpoint, set_verbose, get_info_motor, get_info_board, get_verbose, info_ack, info_motor, info_board, info_verbose} msg_type;
 
extern const char msg_type_size[NB_MSG_TYPE];
    
#ifdef __cplusplus
extern "C" {
#endif

int make_new_buffer(char buffer[BUFFER_MAX_SIZE], msg_type type);
int set_field_motor(char buffer[BUFFER_MAX_SIZE], char index_mot);
int set_field_inhibition(char buffer[BUFFER_MAX_SIZE], char inhibition);
int set_field_mode(char buffer[BUFFER_MAX_SIZE], char mode);
int set_field_address(char buffer[BUFFER_MAX_SIZE], short int address);
int set_field_stor(char buffer[BUFFER_MAX_SIZE], char stor);
int set_field_setpoint(char buffer[BUFFER_MAX_SIZE], float setpoint);
int set_field_msg_type(char buffer[BUFFER_MAX_SIZE], msg_type type);
int set_field_id(char buffer[BUFFER_MAX_SIZE], long int id);
int set_field_verbose(char buffer[BUFFER_MAX_SIZE], short int verbose);
int set_field_ack(char buffer[BUFFER_MAX_SIZE], short int ack);
/**
 * @brief Use set_field_time(buf, 0) to set field to actual timelong
 * @details [long description]
 * 
 * @param buffer [description]
 * @param int [description]
 * 
 * @return [description]
 */
int set_field_time(char buffer[BUFFER_MAX_SIZE], long int time_100us);
long int make_id(char buffer[BUFFER_MAX_SIZE]);
// GET
bool isValidData(char buffer[BUFFER_MAX_SIZE]);
char get_field_motor(char buffer[BUFFER_MAX_SIZE]);
char get_field_inhibition(char buffer[BUFFER_MAX_SIZE]);
char get_field_mode(char buffer[BUFFER_MAX_SIZE]);
short int get_field_address(char buffer[BUFFER_MAX_SIZE]);
char get_field_stor(char buffer[BUFFER_MAX_SIZE]);
float get_field_setpoint(char buffer[BUFFER_MAX_SIZE]);
msg_type get_field_type(char buffer[BUFFER_MAX_SIZE]);
long int get_field_id(char buffer[BUFFER_MAX_SIZE]);
short int get_field_verbose(char buffer[BUFFER_MAX_SIZE]);
short int get_field_ack(char buffer[BUFFER_MAX_SIZE]);
long int get_field_time(char buffer[BUFFER_MAX_SIZE]);


//void test_network_functions();
 
#ifdef __cplusplus
}
#endif

#endif // _NETWORK_HEADER_
