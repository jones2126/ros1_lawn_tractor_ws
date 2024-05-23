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

#include "datalogger.h"
#include <time.h>


void ecef_to_geo( double X, double Y, double Z, double* longitude, double* latitude, double* height) {
    double a, f,b,e2,ep2,r2,r,E2,F,G,c,s,P,Q,ro,tmp,U,V,zo,h,phi,lambda;

    a = 6378137.0; /* earth semimajor axis in meters */
    f = 1/298.257223563; /* reciprocal flattening */
    b = a*(1-f); /* semi-minor axis */

    e2 = 2*f-f*f; /* first eccentricity squared */
    ep2 = f*(2-f)/((1-f)*(1-f)); /* second eccentricity squared */

    r2 = X*X+Y*Y;
    r = sqrt(r2);
    E2 = a*a - b*b;
    F = 54*b*b*Z*Z;
    G = r2 + (1-e2)*Z*Z - e2*E2;
    c = (e2*e2*F*r2)/(G*G*G);
    s = pow( ( 1 + c + sqrt(c*c + 2*c) ) , 1/3 );
    P = F/(3*(s+1/s+1)*(s+1/s+1)*G*G);
    Q = sqrt(1+2*e2*e2*P);
    ro = -(e2*P*r)/(1+Q) + sqrt((a*a/2)*(1+1/Q) - ((1-e2)*P*Z*Z)/(Q*(1+Q)) - P*r2/2);
    tmp = (r - e2*ro)*(r - e2*ro);
    U = sqrt( tmp + Z*Z );
    V = sqrt( tmp + (1-e2)*Z*Z );
    zo = (b*b*Z)/(a*V);

    h = U*( 1 - b*b/(a*V));
    phi = atan( (Z + ep2*zo)/r );
    lambda = atan2(Y,X);

    *longitude = lambda*180/M_PI;
    *latitude = phi*180/M_PI;
    *height = h;
}

/* TODO: handle leap seconds */
unsigned long gsp_time_to_timestamp( int wno, int tow) {
    return 604800 * wno + 315964800 + tow;
}

void timestamp_to_iso8601str(char *time_string, time_t timestamp) {
    struct tm *tm = gmtime(&timestamp);
    char *format;
    int n;
    /* sample of iso8601 time in UTC: 2008-10-16T14:55:29Z */
    format = "%02d-%02d-%02dT%02d:%02d:%02d";
    n = sprintf(time_string, format,
                tm->tm_year+1900,
                tm->tm_mon+1,
                tm->tm_mday,
                tm->tm_hour,
                tm->tm_min,
                tm->tm_sec);
    time_string[n++] = 'Z';
}

void output_gpx_trk_point( const long timestamp, const double latitude, const double longitude, const double height, const int speed) {
    char iso8601str[] = "2008-10-16T14:55:29Z";
    timestamp_to_iso8601str(iso8601str, timestamp);
    printf(" <trkpt lat=\"%f\" lon=\"%f\"><ele>%f</ele><time>%s</time><speed>%d</speed></trkpt>\n", latitude, longitude,height,iso8601str, speed);
}

void decode_long_entry( const gbuint8* d, long* time, int* ecef_x, int* ecef_y, int* ecef_z, int* speed) {
    int wno, tow;

    *speed = d[1];
    wno = (d[3] | (d[2]&0xf)<<8) + 1024;
    tow = (d[4] << 12) | (d[5] << 4)  | ((d[2] >> 4) & 0xf);
    *ecef_x =  d[7] + ( d[6] << 8) + ( d[9] << 16) + ( d[8] << 24);
    *ecef_y = d[11] + (d[10] << 8) + (d[13] << 16) + (d[12] << 24);
    *ecef_z = d[15] + (d[14] << 8) + (d[17] << 16) + (d[16] << 24);

    *time = gsp_time_to_timestamp(wno,tow);
}

void decode_short_entry( const gbuint8* d, long* time, int* ecef_x, int* ecef_y, int* ecef_z, int* speed ) {
    int dt, dx,dy,dz;
    *speed = d[1];
    dt = (d[2] << 8) + d[3];
    dx = (d[4] << 2) + ((d[5]>>6)&0x3);
    dy = (d[5]&0x3f) | (((d[6]>>4)&0xf)<<6);
    dz = ((d[6]&3) << 8) + d[7];

    /* handle negative values */
    if ( dx >= 512 ) {
        dx = 511-dx;
    }
    if ( dy >= 512 ) {
        dy = 511-dy;
    }
    if ( dz >= 512 ) {
        dz = 511-dz;
    }

    *time = *time + dt;
    *ecef_x = *ecef_x + dx;
    *ecef_y = *ecef_y + dy;
    *ecef_z = *ecef_z + dz;
}

/** returns the last recorded timestamp */
long process_buffer(const gbuint8* buffer, const int length, const long first_timestamp ) {
    int offset = 0;
    long time;
    int ecef_x,  ecef_y, ecef_z,  speed;
    double latitude, longitude, height;
    int tagged_entry = 0;
    long last_timestamp = first_timestamp;

    DEBUG("processing %d bytes\n", length);

    printf("<!-- next sector -->\n");

    while ( offset < length ) {


        if ( buffer[offset] & 0x40 ) {
            /* long entry */
            decode_long_entry(buffer+offset,  &time, &ecef_x,&ecef_y, &ecef_z, &speed);
            ecef_to_geo(ecef_x, ecef_y,ecef_z ,&longitude, &latitude, &height);
            if ( buffer[offset] & 0x20 ) {
                tagged_entry = 1;
            }

            if ( (last_timestamp >  0) && (time > (last_timestamp + 3600l) )) {
                /* start a new track segment if the time difference between
                  two points is more than one hour */
                printf("</trkseg>\n<trkseg>\n");
            }
            output_gpx_trk_point( time, latitude, longitude, height, speed);

            offset += 18;
        } else if (  buffer[offset] == 0x80 ) {
            /* short entry */
            decode_short_entry(buffer+offset,  &time, &ecef_x,&ecef_y, &ecef_z, &speed);
            ecef_to_geo(ecef_x, ecef_y,ecef_z ,&longitude, &latitude, &height);

            if ( (last_timestamp >  0) && (time > (last_timestamp + 3600l) )) {
                /* start a new track segment if the time difference between
                  two points is more than one hour */
                printf("</trkseg>\n<trkseg>\n");
            }
            output_gpx_trk_point( time, latitude, longitude, height, speed);

            offset += 8;
        } else {
            /* search for valid entry */
            offset++;
        }

        last_timestamp = time;
    }

    return last_timestamp;
}
