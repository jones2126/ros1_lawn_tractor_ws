/*

    Control program for SkyTraq GPS data logger.

    Copyright (C) 2008  Jesper Zedlitz, jesper@zedlitz.de

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111 USA

 */

#ifndef datalogger_h
#define datalogger_h

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include "agps-download.h"

enum {ERROR=-1, SUCCESS};
enum {NACK=-1, ACK};

#define gbuint8 unsigned char
#define gbuint32 unsigned long
#define gbuint16 unsigned int

#ifdef DEBUG_ALL
#undef DEBUG
#define DEBUG(fmt, args...) fprintf(stderr, fmt, ##args)
#else
#define DEBUG(fmt, args...)
#endif

#define SKYTRAQ_SPEED_4800      0
#define SKYTRAQ_SPEED_9600      1
#define SKYTRAQ_SPEED_19200     2
#define SKYTRAQ_SPEED_38400     3
#define SKYTRAQ_SPEED_57600     4
#define SKYTRAQ_SPEED_115200    5

typedef struct SkyTraqPackage {
    gbuint8	length;
    gbuint8* 	data;
    gbuint8 	checksum;
} SkyTraqPackage;

typedef struct skytraq_config {
    gbuint32    log_wr_ptr;
    gbuint16    sectors_left;
    gbuint16    total_sectors;
    gbuint32	max_time;
    gbuint32	min_time;
    gbuint32	max_distance;
    gbuint32	min_distance;
    gbuint32	max_speed;
    gbuint32	min_speed;
    gbuint8	datalog_enable;
    gbuint8	log_fifo_mode;
    gbuint8	agps_enabled;
    unsigned	agps_hours_left;
} skytraq_config;

int skytraq_read_software_version( int fd);
int skytraq_read_datalogger_config( int fd, skytraq_config* config);
int skytraq_read_datalog_sector( int fd, gbuint8 sector, gbuint8* buffer );
void skytraq_clear_datalog( int fd);
void skytraq_write_datalogger_config( int fd, skytraq_config* config);
long process_buffer(const gbuint8* buffer,const  int length,const  long last_timestamp);
int skytraq_determine_speed( int fd) ;
unsigned skytraq_mkspeed(unsigned br);
int skytraq_set_serial_speed( int fd, int speed, int permanent);
void skytraq_read_agps_status(int fd, skytraq_config* config);
int skytraq_output_disable( int fd );
int skytraq_output_enable_nmea( int fd );
int skytraq_output_enable_binary( int fd );
int skytraq_send_agps_data( int fd, agps_data* data );

#endif
