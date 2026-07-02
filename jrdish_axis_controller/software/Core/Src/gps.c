/* -----------------------------------------------------------------------
 * gps.c
 * GPS NMEA parser and 1PPS capture
 *
 * Parses $GNRMC, $GNGGA, $GNGSV sentences from LPUART1.
 * Captures 1PPS rising edge via EXTI9 on PB9.
 * ----------------------------------------------------------------------- */

#include "gps.h"
#include "modbus.h"
#include "modbus_regs.h"
#include "main.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

extern UART_HandleTypeDef hlpuart1;

/* ----------------------------------------------------------------------- */
/* Private state                                                            */
/* ----------------------------------------------------------------------- */
#define GPS_RX_BUF_SIZE     128
#define GPS_LINE_BUF_SIZE   128

static uint8_t      rxByte;
static char         rxLine[GPS_LINE_BUF_SIZE];
static uint8_t      rxIdx = 0;
static uint8_t      lineReady = 0;

static GPS_Fix_t    fix;
static GPS_SatView_t satView;
static GPS_PPS_t    pps;

/* ----------------------------------------------------------------------- */
/* NMEA utilities                                                           */
/* ----------------------------------------------------------------------- */

/* Validate NMEA checksum — returns 1 if valid */
static uint8_t NMEA_ValidChecksum(const char *sentence)
{
    /* Find $ and * */
    const char *start = strchr(sentence, '$');
    const char *star  = strchr(sentence, '*');
    if (!start || !star || star <= start + 1)
        return 0;

    uint8_t calc = 0;
    for (const char *p = start + 1; p < star; p++)
        calc ^= (uint8_t)*p;

    uint8_t rx = (uint8_t)strtol(star + 1, NULL, 16);
    return calc == rx;
}

/* Get Nth comma-delimited field from NMEA sentence into buf */
static uint8_t NMEA_GetField(const char *sentence, uint8_t fieldNum,
                              char *buf, uint8_t bufLen)
{
    const char *p = sentence;
    uint8_t field = 0;

    /* Skip to start after $ */
    while (*p && *p != '$') p++;
    if (*p == '$') p++;

    while (*p)
    {
        if (field == fieldNum)
        {
            uint8_t i = 0;
            while (*p && *p != ',' && *p != '*' && i < bufLen - 1)
                buf[i++] = *p++;
            buf[i] = '\0';
            return 1;
        }
        if (*p == ',') field++;
        p++;
    }
    buf[0] = '\0';
    return 0;
}

/* Convert NMEA lat/lon ddmm.mmmm to decimal degrees */
static float NMEA_ParseDegMin(const char *val, const char *dir)
{
    if (!val || val[0] == '\0') return 0.0f;

    float raw   = atof(val);
    int   deg   = (int)(raw / 100);
    float min   = raw - deg * 100;
    float result = deg + min / 60.0f;

    if (dir[0] == 'S' || dir[0] == 'W')
        result = -result;

    return result;
}

/* ----------------------------------------------------------------------- */
/* Sentence parsers                                                         */
/* ----------------------------------------------------------------------- */

static void Parse_GNRMC(const char *sentence)
{
    char f[32];

    /* Field 2: status A=valid, V=invalid */
    NMEA_GetField(sentence, 2, f, sizeof(f));
    fix.valid = (f[0] == 'A') ? 1 : 0;

    /* Field 1: UTC time hhmmss.ss */
    NMEA_GetField(sentence, 1, f, sizeof(f));
    if (strlen(f) >= 6)
    {
        fix.hour   = (f[0]-'0')*10 + (f[1]-'0');
        fix.minute = (f[2]-'0')*10 + (f[3]-'0');
        fix.second = (f[4]-'0')*10 + (f[5]-'0');
    }

    /* Field 9: date ddmmyy */
    NMEA_GetField(sentence, 9, f, sizeof(f));
    if (strlen(f) == 6)
    {
        fix.day   = (f[0]-'0')*10 + (f[1]-'0');
        fix.month = (f[2]-'0')*10 + (f[3]-'0');
        fix.year  = 2000 + (f[4]-'0')*10 + (f[5]-'0');
    }

    /* Fields 3-6: lat/lon */
    char lat[16], ns[4], lon[16], ew[4];
    NMEA_GetField(sentence, 3, lat, sizeof(lat));
    NMEA_GetField(sentence, 4, ns,  sizeof(ns));
    NMEA_GetField(sentence, 5, lon, sizeof(lon));
    NMEA_GetField(sentence, 6, ew,  sizeof(ew));
    fix.lat = NMEA_ParseDegMin(lat, ns);
    fix.lon = NMEA_ParseDegMin(lon, ew);

    /* Update Modbus GPS registers */
    Modbus_SetReg(REG_GPS_STATUS,   fix.valid ? 1 : 0);
    Modbus_SetReg(REG_GPS_UTC_HH_MM, fix.hour * 100 + fix.minute);
    Modbus_SetReg(REG_GPS_UTC_SS,    fix.second);

    printf("RMC: %02d:%02d:%02d %02d/%02d/%04d valid=%d lat=%.4f lon=%.4f\r\n",
           fix.hour, fix.minute, fix.second,
           fix.day, fix.month, fix.year,
           fix.valid, fix.lat, fix.lon);
}

static void Parse_GNGGA(const char *sentence)
{
    char f[32];

    /* Field 6: fix quality 0=none,1=GPS,2=DGPS */
    NMEA_GetField(sentence, 6, f, sizeof(f));
    uint8_t quality = atoi(f);

    /* Field 7: number of satellites */
    NMEA_GetField(sentence, 7, f, sizeof(f));
    fix.numSats = atoi(f);

    /* Field 8: HDOP */
    NMEA_GetField(sentence, 8, f, sizeof(f));
    fix.hdop = atof(f);

    /* Field 9: altitude */
    NMEA_GetField(sentence, 9, f, sizeof(f));
    fix.alt = atof(f);

    printf("GGA: quality=%d sats=%d hdop=%.1f alt=%.1fm\r\n",
           quality, fix.numSats, fix.hdop, fix.alt);
}

static void Parse_GNGSV(const char *sentence)
{
    char f[16];

    /* Field 1: total sentences */
    NMEA_GetField(sentence, 1, f, sizeof(f));
    uint8_t totalSentences = atoi(f);

    /* Field 2: sentence number */
    NMEA_GetField(sentence, 2, f, sizeof(f));
    uint8_t sentenceNum = atoi(f);

    /* Field 3: total sats in view */
    NMEA_GetField(sentence, 3, f, sizeof(f));
    uint8_t totalSats = atoi(f);

    /* First sentence — reset count */
    if (sentenceNum == 1)
    {
        satView.count = 0;
        memset(satView.sats, 0, sizeof(satView.sats));
    }

    /* Fields 4-7, 8-11, 12-15, 16-19: up to 4 sats per sentence */
    for (uint8_t i = 0; i < 4; i++)
    {
        if (satView.count >= GPS_MAX_SATS) break;

        uint8_t base = 4 + i * 4;
        char prn[8], elev[8], azim[8], snr[8];
        NMEA_GetField(sentence, base,     prn,  sizeof(prn));
        NMEA_GetField(sentence, base + 1, elev, sizeof(elev));
        NMEA_GetField(sentence, base + 2, azim, sizeof(azim));
        NMEA_GetField(sentence, base + 3, snr,  sizeof(snr));

        if (prn[0] == '\0') break;

        GPS_Sat_t *sat = &satView.sats[satView.count++];
        sat->prn       = atoi(prn);
        sat->elevation = atoi(elev);
        sat->azimuth   = atoi(azim);
        sat->snr       = atoi(snr);
    }

    /* Last sentence — print summary */
    if (sentenceNum == totalSentences)
    {
        printf("GSV: %d satellites in view:\r\n", satView.count);
        for (uint8_t i = 0; i < satView.count; i++)
        {
            printf("  PRN%02d  El:%2d  Az:%3d  SNR:%2d dBHz\r\n",
                   satView.sats[i].prn,
                   satView.sats[i].elevation,
                   satView.sats[i].azimuth,
                   satView.sats[i].snr);
        }
    }
}

/* ----------------------------------------------------------------------- */
/* Sentence dispatcher                                                      */
/* ----------------------------------------------------------------------- */

static void GPS_ProcessLine(void)
{
    if (!NMEA_ValidChecksum(rxLine))
    {
        printf("GPS: bad checksum: %s\r\n", rxLine);
        return;
    }

    if      (strncmp(rxLine, "$GNRMC", 6) == 0) Parse_GNRMC(rxLine);
    else if (strncmp(rxLine, "$GPRMC", 6) == 0) Parse_GNRMC(rxLine);
    else if (strncmp(rxLine, "$GNGGA", 6) == 0) Parse_GNGGA(rxLine);
    else if (strncmp(rxLine, "$GPGGA", 6) == 0) Parse_GNGGA(rxLine);
    else if (strncmp(rxLine, "$GNGSV", 6) == 0) Parse_GNGSV(rxLine);
    else if (strncmp(rxLine, "$GPGSV", 6) == 0) Parse_GNGSV(rxLine);
}

/* ----------------------------------------------------------------------- */
/* UART RX callback — accumulate bytes into lines                          */
/* ----------------------------------------------------------------------- */
void GPS_UART_Callback(void)
{
    char c = (char)rxByte;

    if (c == '\n')
    {
        /* Strip trailing \r if present */
        if (rxIdx > 0 && rxLine[rxIdx-1] == '\r')
            rxIdx--;
        rxLine[rxIdx] = '\0';
        lineReady = 1;
        rxIdx = 0;
    }
    else if (c != '\r')
    {
        if (rxIdx < GPS_LINE_BUF_SIZE - 1)
            rxLine[rxIdx++] = c;
        else
            rxIdx = 0;  /* overflow — reset */
    }

    HAL_UART_Receive_IT(&hlpuart1, &rxByte, 1);
}

/* ----------------------------------------------------------------------- */
/* 1PPS callback — called from EXTI9 IRQ                                   */
/* ----------------------------------------------------------------------- */
void GPS_1PPS_Callback(void)
{
    pps.tickCapture = HAL_GetTick();
    pps.fired = 1;

    /* Update Modbus 1PPS counter */
    static uint16_t ppsCount = 0;
    ppsCount++;
    Modbus_SetReg(REG_GPS_1PPS_COUNT_LO, ppsCount);

    printf("1PPS #%u at tick %lu\r\n", ppsCount, pps.tickCapture);
}

/* ----------------------------------------------------------------------- */
/* Public API                                                               */
/* ----------------------------------------------------------------------- */

void GPS_Init(void)
{
    memset(&fix,     0, sizeof(fix));
    memset(&satView, 0, sizeof(satView));
    memset(&pps,     0, sizeof(pps));

    HAL_UART_Receive_IT(&hlpuart1, &rxByte, 1);
    printf("GPS_Init done\r\n");
}

void GPS_Run(void)
{
/*
    // Try blocking receive — bypasses interrupt 
    uint8_t buf[128];
    HAL_StatusTypeDef ret = HAL_UART_Receive(&hlpuart1, buf, 10, 200);
    if (ret == HAL_OK)
        printf("GPS RX OK: %.*s\r\n", 10, buf);
    else if (ret == HAL_TIMEOUT)
        printf("GPS RX timeout\r\n");
    else
        printf("GPS RX error\r\n");
    */

    if (!lineReady) return;
    lineReady = 0;
    GPS_ProcessLine();
}

GPS_Fix_t*     GPS_GetFix(void)  { return &fix; }
GPS_SatView_t* GPS_GetSats(void) { return &satView; }
GPS_PPS_t*     GPS_GetPPS(void)  { return &pps; }
