#ifndef GPS_H
#define GPS_H

/* -----------------------------------------------------------------------
 * gps.h
 * GPS NMEA parser and 1PPS capture
 *
 * LPUART1: PB10=RX, PB11=TX, 115200 8N1
 * GPS_1PPS: PB9, EXTI9, active high pulse
 *
 * Parses: $GNRMC, $GNGGA, $GNGSV
 * ----------------------------------------------------------------------- */

#include <stdint.h>

/* GPS fix data */
typedef struct {
    uint8_t  valid;          /* 1 = fix valid                     */
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    float    lat;            /* decimal degrees, + = N            */
    float    lon;            /* decimal degrees, + = E            */
    float    alt;            /* meters                            */
    uint8_t  numSats;
    float    hdop;
} GPS_Fix_t;

/* Satellite in view */
typedef struct {
    uint8_t  prn;
    uint8_t  elevation;      /* degrees 0-90                      */
    uint16_t azimuth;        /* degrees 0-359                     */
    uint8_t  snr;            /* dBHz                              */
} GPS_Sat_t;

#define GPS_MAX_SATS    16

typedef struct {
    uint8_t   count;
    GPS_Sat_t sats[GPS_MAX_SATS];
} GPS_SatView_t;

/* 1PPS status */
typedef struct {
    uint8_t  fired;          /* set on each 1PPS pulse, clear after reading */
    uint32_t tickCapture;    /* HAL_GetTick() at moment of pulse  */
} GPS_PPS_t;

void          GPS_Init(void);
void          GPS_Run(void);

GPS_Fix_t*    GPS_GetFix(void);
GPS_SatView_t* GPS_GetSats(void);
GPS_PPS_t*    GPS_GetPPS(void);

/* Called from EXTI9 IRQ — do not call directly */
void          GPS_1PPS_Callback(void);

/* Called from LPUART1 IRQ — do not call directly */
void          GPS_UART_Callback(void);

#endif /* GPS_H */
