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

#ifndef lowlevel_h
#define lowlevel_h

void skytraq_dump_package( SkyTraqPackage* p ) ;
void skytraq_free_package( SkyTraqPackage* p );
SkyTraqPackage* skytraq_new_package( int length );
SkyTraqPackage* skytraq_read_next_package( int fd, unsigned timeout );
int skytraq_write_package_with_response( int fd, SkyTraqPackage* p, unsigned timeout );
int open_port( char* device);
int set_port_speed( int fd, unsigned speed);
int read_with_timeout( int fd, void* buffer, unsigned len, unsigned timeout);

int write_buffer(int fd, gbuint8* buf, int len);

/**
  * Read a zero-terminated string from the GPS device.
  */
int read_string( int fd, gbuint8* buffer, int max_length, unsigned timeout );

#endif
