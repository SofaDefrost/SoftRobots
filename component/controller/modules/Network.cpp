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
#include "Network.h"

/*==========  Global Variables  ==========*/
// const char msg_type_size[NB_MSG_TYPE] =
// {     9     // set_setpoint
//     , 5     // set_inhibition
//     , 3     // set_verbose
//     , 5     // set_stor
//     , 5     // get_info_motor
//     , 4     // get_info_board
//     , 2     // get_verbose
//     , 3     // info_error_return
//     , 23    // info_motor
//     , 5     // info_board
//     , 3     // info_verbose
// };


int make_new_buffer(char buffer[BUFFER_MAX_SIZE], msg_type type)
{
    int i;

    for(i=0; i<BUFFER_MAX_SIZE; i++)
        buffer[i] = 0;

    buffer[0] = type << 4;

    return 0;
}

int set_field_motor(char buffer[BUFFER_MAX_SIZE], char index_mot)
{
    buffer[9] = (buffer[9] & 0xFC) | (index_mot & 0x03);

    return 0;
}

int set_field_inhibition(char buffer[BUFFER_MAX_SIZE], char inhibition)
{
    buffer[9] = (buffer[9] & 0xF3) | ((inhibition & 0x03) << 2);

    return 0;
}

int set_field_mode(char buffer[BUFFER_MAX_SIZE], char mode)
{
    buffer[9] = (buffer[9] & 0x3F) | ((mode & 0x07) << 5);

    return 0;
}

int set_field_address(char buffer[BUFFER_MAX_SIZE], short int address)
{
    buffer[7] = (char)((address) & 0x00FF);
    buffer[8] = (char)((address >> 8) & 0x00FF);

    return 0;
}

int set_field_stor(char buffer[BUFFER_MAX_SIZE], char stor)
{
    buffer[9] = stor;

    return 0;
}

int set_field_setpoint(char buffer[BUFFER_MAX_SIZE], float setpoint)
{
    // Reset unchanged bit
    buffer[9] = (buffer[9] & 0xEF);
    if(setpoint == UNCHANGED)
        buffer[9] |= (1 << 4);
    else
        Float2Bytes((uint8*)&(buffer[10]), setpoint);

    return 0;
}

int set_field_msg_type(char buffer[BUFFER_MAX_SIZE], msg_type type)
{
    buffer[0] = (char)((buffer[0] & 0x0F) | ((type & 0x0F) << 4));

    return 0;
}

int set_field_id(char buffer[BUFFER_MAX_SIZE], long int id)
{
    buffer[0] = (buffer[0] & 0xF0) | ((id & 0x0F000000) >> 24);
    buffer[1] = (id & 0x00FF0000) >> 16;
    buffer[2] = (id & 0x0000FF00) >> 8;
    buffer[3] = (id & 0x000000FF) >> 0;

    return 0;
}

int set_field_verbose(char buffer[BUFFER_MAX_SIZE], short int verbose)
{
    buffer[7] = (char)((buffer[7] & 0x0F) | ((verbose & 0x0F) << 4));

    return 0;
}

int set_field_ack(char buffer[BUFFER_MAX_SIZE], short int ack)
{
    buffer[7] = (char)((buffer[7] & 0x0F) | (ack & 0x0F));

    return 0;
}

int set_field_time(char buffer[BUFFER_MAX_SIZE], long int time_100us)
{
    struct timeval tv;

    if(time_100us == 0)
    {
        gettimeofday(&tv, NULL);
        time_100us = tv.tv_usec / 100;
    }

    buffer[4] = (time_100us & 0x00FF0000) >> 16;
    buffer[5] = (time_100us & 0x0000FF00) >> 8;
    buffer[6] = (time_100us & 0x000000FF) >> 0;

    return 0;
}

long int make_id(char buffer[BUFFER_MAX_SIZE])
{
    int i;
    long int checksum = 0;

    // Make a 28 bits CRC
    checksum += buffer[0] & 0x0F;
    checksum &= 0x0FFFFFFF;
    for (i = 4; i < BUFFER_MAX_SIZE; ++i)
    {
        checksum = (checksum >> 1) + ((checksum & 1) << 27);
        checksum += buffer[i] & 0xFF;
        checksum &= 0x0FFFFFFF;
    }

    return checksum;
}

bool isValidData(char buffer[BUFFER_MAX_SIZE])
{
    return make_id(buffer) == get_field_id(buffer);
}

char get_field_motor(char buffer[BUFFER_MAX_SIZE])
{
    return buffer[9] & 0x03;
}

char get_field_inhibition(char buffer[BUFFER_MAX_SIZE])
{
    return (buffer[9] >> 2) & 0x03;
}

char get_field_mode(char buffer[BUFFER_MAX_SIZE])
{
    return (buffer[9] >> 5) & 0x07;
}

short int get_field_address(char buffer[BUFFER_MAX_SIZE])
{
    return ((buffer[7] & 0x00FF) | ((buffer[8] << 8)& 0xFF00)) & 0xFFFF;
}

char get_field_stor(char buffer[BUFFER_MAX_SIZE])
{
    return buffer[9] & 0xFF;
}

float get_field_setpoint(char buffer[BUFFER_MAX_SIZE])
{
    if(buffer[9] & 0x10)
        return UNCHANGED;
    else
    {
        float setpoint=0.0;
        Bytes2Float(&setpoint, (uint8*)&(buffer[10]));
        return setpoint;
    }
}

msg_type get_field_type(char buffer[BUFFER_MAX_SIZE])
{
    return (msg_type)((buffer[0] >> 4) & 0x0F);
}

long int get_field_id(char buffer[BUFFER_MAX_SIZE])
{
    long int id = 0;

    id |= (long int)(buffer[0] & 0x0F) << 24;
    id |= (long int)(buffer[1] & 0xFF) << 16;
    id |= (long int)(buffer[2] & 0xFF) << 8;
    id |= (long int)(buffer[3] & 0xFF) << 0;

    return id;
}

short int get_field_verbose(char buffer[BUFFER_MAX_SIZE])
{
    return (buffer[7] >> 4) & 0x0F;
}

short int get_field_ack(char buffer[BUFFER_MAX_SIZE])
{
    return buffer[7] & 0x0F;
}

long int get_field_time(char buffer[BUFFER_MAX_SIZE])
{
    long int time_100us = 0;

    time_100us |= (long int)(buffer[4] & 0xFF) << 16;
    time_100us |= (long int)(buffer[5] & 0xFF) << 8;
    time_100us |= (long int)(buffer[6] & 0xFF) << 0;

    return time_100us;
}

//void test_network_functions()
//{
//     int i = 0;
//     char buf[BUFFER_MAX_SIZE];

//     // Motor
//     printf("motor\n");
//     make_new_buffer(buf, info_ack);
//     set_field_motor(buf, 0xAEB);
//     for(i = 0; i < BUFFER_MAX_SIZE; i++)
//     {
//         printf("%02X ",buf[i]&0xFF);
//         if(i%4 == 3)
//             printf("- ");
//     }
//     printf("\nResult for 0xAEB : %d\n",get_field_motor(buf));
//     puts("\n----\n");

//     // Inhibition
//     printf("inhibition\n");
//     make_new_buffer(buf, info_ack);
//     set_field_inhibition(buf, 0xAEB);
//     for(i = 0; i < BUFFER_MAX_SIZE; i++)
//     {
//         printf("%02X ",buf[i]&0xFF);
//         if(i%4 == 3)
//             printf("- ");
//     }
//     printf("\nResult for 0xAEB : %d\n",get_field_inhibition(buf));
//     puts("\n----\n");

//     // Mode
//     printf("mode\n");
//     make_new_buffer(buf, info_ack);
//     set_field_mode(buf, 0xAEB);
//     for(i = 0; i < BUFFER_MAX_SIZE; i++)
//     {
//         printf("%02X ",buf[i]&0xFF);
//         if(i%4 == 3)
//             printf("- ");
//     }
//     printf("\nResult for 0xAEB : %d\n",get_field_mode(buf));
//     puts("\n----\n");

//     // Address
//     printf("address\n");
//     make_new_buffer(buf, info_ack);
//     set_field_address(buf, 0xEEAEB);
//     for(i = 0; i < BUFFER_MAX_SIZE; i++)
//     {
//         printf("%02X ",buf[i]&0xFF);
//         if(i%4 == 3)
//             printf("- ");
//     }
//     printf("\nResult for 0xEEAEB : %x\n",get_field_address(buf));
//     puts("\n----\n");

//     // Stor
//     printf("stor\n");
//     make_new_buffer(buf, info_ack);
//     set_field_stor(buf, 0xAEB);
//     for(i = 0; i < BUFFER_MAX_SIZE; i++)
//     {
//         printf("%02X ",buf[i]&0xFF);
//         if(i%4 == 3)
//             printf("- ");
//     }
//     printf("\nResult for 0xAEB : %x\n",get_field_stor(buf));
//     puts("\n----\n");

//     // Setpoint
//     printf("setpoint\n");
//     make_new_buffer(buf, info_ack);
//     set_field_setpoint(buf, M_PI);
//     for(i = 0; i < BUFFER_MAX_SIZE; i++)
//     {
//         printf("%02X ",buf[i]&0xFF);
//         if(i%4 == 3)
//             printf("- ");
//     }
//     printf("\nResult for M_PI : %f\n",get_field_setpoint(buf));
//     puts("\n----\n");

//     // Type
//     printf("type\n");
//     make_new_buffer(buf, info_ack);
//     set_field_msg_type(buf, 0xAEB);
//     for(i = 0; i < BUFFER_MAX_SIZE; i++)
//     {
//         printf("%02X ",buf[i]&0xFF);
//         if(i%4 == 3)
//             printf("- ");
//     }
//     printf("\nResult for 0xAEB : %x\n",get_field_type(buf));
//     puts("\n----\n");

//     // Id
//     printf("id\n");
//     make_new_buffer(buf, info_ack);
//     set_field_id(buf, 0xAEB);
//     for(i = 0; i < BUFFER_MAX_SIZE; i++)
//     {
//         printf("%02X ",buf[i]&0xFF);
//         if(i%4 == 3)
//             printf("- ");
//     }
//     printf("\nResult for 0xAEB : %lx\n",get_field_id(buf));
//     puts("\n----\n");

//     // Verbose
//     printf("verbose\n");
//     make_new_buffer(buf, info_ack);
//     set_field_verbose(buf, 0xAEB);
//     for(i = 0; i < BUFFER_MAX_SIZE; i++)
//     {
//         printf("%02X ",buf[i]&0xFF);
//         if(i%4 == 3)
//             printf("- ");
//     }
//     printf("\nResult for 0xAEB : %x\n",get_field_verbose(buf));
//     puts("\n----\n");

//     // Ack
//     printf("ack\n");
//     make_new_buffer(buf, info_ack);
//     set_field_ack(buf, 0xAEB);
//     for(i = 0; i < BUFFER_MAX_SIZE; i++)
//     {
//         printf("%02X ",buf[i]&0xFF);
//         if(i%4 == 3)
//             printf("- ");
//     }
//     printf("\nResult for 0xAEB : %x\n",get_field_ack(buf));
//     puts("\n----\n");
//}

