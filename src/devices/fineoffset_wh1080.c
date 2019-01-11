/*
 * *** Fine Offset WH1080/WH3080 Weather Station ***
 *
 * This module is based on Stanisław Pitucha ('viraptor' https://github.com/viraptor ) code stub for the Digitech XC0348
 * Weather Station, which seems to be a rebranded Fine Offset WH1080 Weather Station.
 *
 * Some info and code derived from Kevin Sangelee's page:
 * http://www.susa.net/wordpress/2012/08/raspberry-pi-reading-wh1081-weather-sensors-using-an-rfm01-and-rfm12b/ .
 *
 * See also Frank 'SevenW' page ( https://www.sevenwatt.com/main/wh1080-protocol-v2-fsk/ ) for some other useful info.
 *
 * For the WH1080 part I mostly have re-elaborated and merged their works. Credits (and kudos) should go to them all
 * (and to many others too).
 *
 * Reports 1 row, 88 pulses
 * Format: ff ID ?X XX YY ZZ ?? ?? ?? UU CC
 * - ID: device id
 * - ?X XX: temperature, likely in 0.1C steps (.1 e7 == 8.7C, .1 ef == 9.5C)
 * - YY: percent in a single byte (for example 54 == 84%)
 * - ZZ: wind speed (00 == 0, 01 == 1.1km/s, ...)
 * - UU: wind direction: 00 is N, 02 is NE, 04 is E, etc. up to 0F is seems
 * - CC: checksum
 *
 *****************************************
 * WH1080
 *****************************************
 * (aka Watson W-8681)
 * (aka Digitech XC0348 Weather Station)
 * (aka PCE-FWS 20)
 * (aka Elecsa AstroTouch 6975)
 * (aka Froggit WH1080)
 * (aka .....)
 *
 * This weather station is based on an indoor touchscreen receiver, and on a 5+1 outdoor wireless sensors group
 * (rain, wind speed, wind direction, temperature, humidity, plus a DCF77 time signal decoder, maybe capable to decode
 * some other time signal standard).
 * See the product page here: http://www.foshk.com/weather_professional/wh1080.htm .
 * It's a very popular weather station, you can easily find it on eBay or Amazon (just do a search for 'WH1080').
 *
 * The module works fine, decoding all of the data as read into the original console (there is some minimal difference
 * sometime on the decimals due to the different architecture of the console processor, which is a little less precise).
 *
 * Please note that the pressure sensor (barometer) is enclosed in the indoor console unit, NOT in the outdoor
 * wireless sensors group.
 * That's why it's NOT possible to get pressure data by wireless communication. If you need pressure data you should try
 * an Arduino/Raspberry solution wired with a BMP180/280 or BMP085 sensor.
 *
 * Data are transmitted in a 48 seconds cycle (data packet, then wait 48 seconds, then data packet...).
 *
 * This module is also capable to decode the DCF77/WWVB time signal sent by the time signal decoder
 * (which is enclosed on the sensor tx): around the minute 59 of the even hours the sensor's TX stops sending weather data,
 * probably to receive (and sync with) DCF77/WWVB signals.
 * After around 3-4 minutes of silence it starts to send just time data for some minute, then it starts again with
 * weather data as usual.
 *
 * By living in Europe I can only test DCF77 time decoding, so if you live outside Europe and you find garbage instead
 * of correct time, you should disable/ignore time decoding
 * (or, better, try to implement a more complete time decoding system :) ).
 *
 * To recognize message type (weather or time) you can use the 'msg_type' field on json output:
 * msg_type 0 = weather data
 * msg_type 1 = time data
 *
 * The 'Total rainfall' field is a cumulative counter, increased by 0.3 millimeters of rain at once.
 *
 * The station comes in three TX operating frequency versions: 433, 868.3 and 915 Mhz.
 * The module is tested with a 'Froggit WH1080' on 868.3 Mhz, using '-f 868140000' as frequency parameter and
 * it works fine (compiled in x86, RaspberryPi 1 (v2), Raspberry Pi2 and Pi3, and also on a BananaPi platform. Everything is OK).
 * I don't know if it works also with ALL of the rebranded versions/models of this weather station.
 * I guess it *should* do... Just give it a try! :)
 *
 *
 *****************************************
 * WH3080
 *****************************************
 *
 * The WH3080 Weather Station seems to be basically a WH1080 with the addition of UV/Light sensors onboard.
 * The weather/datetime radio protocol used for both is identical, the only difference is for the addition in the WH3080
 * of the UV/Light part.
 * UV/Light radio messages are disjointed from (and shorter than) weather/datetime radio messages and are transmitted
 * in a 'once-every-60-seconds' cycle.
 *
 * The module is able to decode all kind of data coming from the WH3080: weather, datetime, UV and light plus some
 * error/status code.
 *
 * To recognize message type (weather, datetime or UV/light) you can refer to the 'msg_type' field on json output:
 * msg_type 0 = weather data
 * msg_type 1 = datetime data
 * msg_type 2 = UV/light data
 *
 * While the LCD console seems to truncate/round values in order to best fit to its display, this module keeps entire values
 * as received from externals sensors (exception made for some rounding while converting values from lux to watts/m and fc),
 * so you can see -sometimes- some little difference between module's output and LCD console's values.
 *
 *
 * 2016-2017 Nicola Quiriti ('ovrheat' - 'seven')
 *
 */

#include "decoder.h"
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <ctype.h>
#include "snap7.h"

// ----SNAP7------
S7Object Client;
unsigned char Buffer[65536]; // 64 K buffer

char *Address = "192.168.0.10";     // PLC IP Address hardcoded
int Rack=0,Slot=2; // Default Rack and Slot - hardcoded
int res;
int ok = 0; // Number of test pass
int ko = 0; // Number of test failure
float Temperature_PLC = 0.0;
float Speed_PLC=0.0;
float Gust_PLC=0.0;
float Rain_PLC=0.0;
int Humidity_PLC=0;
int JobDone=false;
int JobResult=0;
float retf=0.0;
byte tempB=0;
int device_id_PLC=8;
int hours_PLC =0;
int minutes_PLC=0;
int seconds_PLC=0;
int year_PLC=0;
int month_PLC=0;
int day_PLC=0;
//int msg_type; // 0=Weather 1=Datetime 2=UV/Light

// Multiwrite buffer
//Msg Type
byte I1[2]="0"; // 2 bytes (1 word)
//Station ID
byte I2[2]="0"; // 2 bytes (1 word)
//Temperature
byte R3[4]="0"; // 4 bytes (1 real)
//Humidity
byte I4[2]="0"; // 2 bytes (1 word)
//Wind direction string
byte B5[3]="0"; // 3 bytes (3 element string)
//Wind speed
byte R6[4]="0"; // 4 bytes (1 real)
//Wind gust
byte R7[4]="0"; // 4 bytes (1 real)
//Total rainfall
byte R8[4]="0"; // 4 bytes (1 real)
//WH1080 sensor battery status
byte B9[3]="0"; // 3 bytes (3 element string)
//Time signal type (DCF77 or WWVB/MSF)
byte B10[8]="0"; // 8 bytes (8 element string)
//Time signal - hours
byte I11[2]="0"; // 2 bytes (1 word)
//Time signal - minutes
byte I12[2]="0"; // 2 bytes (1 word)
//Time signal - seconds
byte I13[2]="0"; // 2 bytes (1 word)
//Time signal - years
byte I14[2]="0"; // 2 bytes (1 word)
//Time signal - months
byte I15[2]="0"; // 2 bytes (1 word)
//Time signal - days
byte I16[2]="0"; // 2 bytes (1 word)








#define CRC_POLY 0x31
#define CRC_INIT 0xff

static char* wind_dir_string[] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW",};
static char* wind_dir_degr[]= {"0", "23", "45", "68", "90", "113", "135", "158", "180", "203", "225", "248", "270", "293", "315", "338",};

static unsigned short get_device_id(const uint8_t* br) {
    return (br[1] << 4 & 0xf0 ) | (br[2] >> 4);
}

static char* get_battery(const uint8_t* br) {
    if ((br[9] >> 4) != 1) {
        return "OK";
	B9[1]="O";
	B9[2]="K";
	B9[3]="!";
    } else {
        return "LOW";
	B9[1]="L";
	B9[2]="O";
	B9[3]="W";
    }
}

// ------------ WEATHER SENSORS DECODING ----------------------------------------------------

static float get_temperature(const uint8_t* br) {
    const int temp_raw = (br[2] << 8) + br[3];
    return ((temp_raw & 0x0fff) - 0x190) / 10.0;
}

static int get_humidity(const uint8_t* br) {
    return br[4];
}

static char* get_wind_direction_str(const uint8_t* br) {
    return wind_dir_string[br[9] & 0x0f];
}

static char* get_wind_direction_deg(const uint8_t* br) {
    return wind_dir_degr[br[9] & 0x0f];
}

static float get_wind_speed_raw(const uint8_t* br) {
    return br[5]; // Raw
}

static float get_wind_avg_ms(const uint8_t* br) {
    return (br[5] * 34.0f) / 100; // Meters/sec.
}

static float get_wind_avg_mph(const uint8_t* br) {
    return ((br[5] * 34.0f) / 100) * 2.23693629f; // Mph
}

static float get_wind_avg_kmh(const uint8_t* br) {
    return ((br[5] * 34.0f) / 100) * 3.6f; // Km/h
}

static float get_wind_avg_knot(const uint8_t* br) {
    return ((br[5] * 34.0f) / 100) * 1.94384f; // Knots
}

static float get_wind_gust_raw(const uint8_t* br) {
    return br[6]; // Raw
}

static float get_wind_gust_ms(const uint8_t* br) {
    return (br[6] * 34.0f) / 100; // Meters/sec.
}

static float get_wind_gust_mph(const uint8_t* br) {
    return ((br[6] * 34.0f) / 100) * 2.23693629f; // Mph

}

static float get_wind_gust_kmh(const uint8_t* br) {
    return ((br[6] * 34.0f) / 100) * 3.6f; // Km/h
}

static float get_wind_gust_knot(const uint8_t* br) {
    return ((br[6] * 34.0f) / 100) * 1.94384f; // Knots
}

static float get_rainfall(const uint8_t* br) {
    unsigned short rain_raw = (((unsigned short)br[7] & 0x0f) << 8) | br[8];
    return (float)rain_raw * 0.3f;
}

// ------------ WH3080 UV SENSOR DECODING ----------------------------------------------------

static unsigned short get_uv_sensor_id(const uint8_t* br) {
    return (br[1] << 4 & 0xf0 ) | (br[2] >> 4);
}

static char* get_uvstatus(const uint8_t* br) {
    if (br[3] == 85) {
        return "OK";
    } else {
        return "ERROR";
    }
}

static unsigned short wh3080_uvi(const uint8_t* br) {
    return (br[2] & 0x0F );
}

// ------------ WH3080 LIGHT SENSOR DECODING -------------------------------------------------

static float get_rawlight(const uint8_t* br) {
    return (((((br[4]) << 16) | ((br[5]) << 8) | br[6])));
}

//----------------- TIME DECODING ----------------------------------------------------

static char* get_signal(const uint8_t* br) {
    if ((br[2] & 0x0F) == 10) {
        return "DCF77";
    } else {
        return "WWVB/MSF";
    }
}

static int get_hours(const uint8_t* br) {
    return ((br[3] >> 4 & 0x03) * 10) + (br[3] & 0x0F);
}

static int get_minutes(const uint8_t* br) {
    return (((br[4] & 0xF0) >> 4) * 10) + (br[4] & 0x0F);
}

static int get_seconds(const uint8_t* br) {
    return (((br[5] & 0xF0) >> 4) * 10) + (br[5] & 0x0F);
}

static int get_year(const uint8_t* br) {
    return (((br[6] & 0xF0) >> 4) * 10) + (br[6] & 0x0F);
}

static int get_month(const uint8_t* br) {
    return ((br[7] >> 4 & 0x01) * 10) + (br[7] & 0x0F);
}

static int get_day(const uint8_t* br) {
    return (((br[8] & 0xF0) >> 4) * 10) + (br[8] & 0x0F);
}

// ----SNAP7------
// ----SNAP7------

//------------------------------------------------------------------------------
//  Async completion callback
//------------------------------------------------------------------------------
// This is a simply text demo, we use callback only to set an internal flag...
void S7API CliCompletion(void *usrPtr, int opCode, int opResult)
{
    JobResult=opResult;
    JobDone = true;
}
//------------------------------------------------------------------------------
// hexdump, a very nice function, it's not mine.
// I found it on the net somewhere some time ago... thanks to the author ;-)
//------------------------------------------------------------------------------
#ifndef HEXDUMP_COLS
#define HEXDUMP_COLS 16
#endif
void hexdump(void *mem, unsigned int len)
{
        unsigned int i, j;

        for(i = 0; i < len + ((len % HEXDUMP_COLS) ? (HEXDUMP_COLS - len % HEXDUMP_COLS) : 0); i++)
        {
                /* print offset */
                if(i % HEXDUMP_COLS == 0)
                {
                        printf("0x%04x: ", i);
                }

                /* print hex data */
                if(i < len)
                {
                        printf("%02x ", 0xFF & ((char*)mem)[i]);
                }
                else /* end of block, just aligning for ASCII dump */
                {
                        printf("   ");
                }

                /* print ASCII dump */
                if(i % HEXDUMP_COLS == (HEXDUMP_COLS - 1))
                {
                        for(j = i - (HEXDUMP_COLS - 1); j <= i; j++)
                        {
                                if(j >= len) /* end of block, not really printing */
                                {
                                        putchar(' ');
                                }
                                else if(isprint((((char*)mem)[j] & 0x7F))) /* printable char */
                                {
                                        putchar(0xFF & ((char*)mem)[j]);
                                }
                                else /* other char */
                                {
                                        putchar('.');
                                }
                        }
                        putchar('\n');
                }
        }
}
//------------------------------------------------------------------------------
// Function: converts input float variable to byte array
//------------------------------------------------------------------------------
void float2Bytes(float val,byte* bytes_array){
  // Create union of shared memory space
  union {
    float float_variable;
    byte temp_array[4];
  } u;
  // Overite bytes of union with float variable
  u.float_variable = val;
  // Assign bytes to input array
  memcpy(bytes_array, u.temp_array, 4);
}
//------------------------------------------------------------------------------
// Function: converts input int(16bit) variable to byte array
//------------------------------------------------------------------------------
void Int2Bytes(int val,byte* bytes_array){
  // Create union of shared memory space
  union {
    int int_variable;
    byte temp_array[2];
  } u;
  // Overite bytes of union with int variable
  u.int_variable = val;
  // Assign bytes to input array
  memcpy(bytes_array, u.temp_array, 2);
}

//------------------------------------------------------------------------------
// Check error
//------------------------------------------------------------------------------
int Check(int Result, char * function)
{
    int ExecTime;
	char text[1024];
    printf("\n");
    printf("+-----------------------------------------------------\n");
    printf("| %s\n",function);
    printf("+-----------------------------------------------------\n");
    if (Result==0) {
        Cli_GetExecTime(Client, &ExecTime);
        printf("| Result         : OK\n");
        printf("| Execution time : %d ms\n",ExecTime);
        printf("+-----------------------------------------------------\n");
        ok++;
    }
    else {
        printf("| ERROR !!! \n");
        if (Result<0)
            printf("| Library Error (-1)\n");
        else
		{
			Cli_ErrorText(Result, text, 1024);
			printf("| %s\n",text);
		}
        printf("+-----------------------------------------------------\n");
        ko++;
    }
    return !Result;
}
//------------------------------------------------------------------------------
// Unit Connection
//------------------------------------------------------------------------------
int CliConnect()
{
    int Requested, Negotiated, res;

    res = Cli_ConnectTo(Client, Address,Rack,Slot);
    if (Check(res,"UNIT Connection")) {
          Cli_GetPduLength(Client, &Requested, &Negotiated);
          printf("  Connected to   : %s (Rack=%d, Slot=%d)\n",Address,Rack,Slot);
          printf("  PDU Requested  : %d bytes\n",Requested);
          printf("  PDU Negotiated : %d bytes\n",Negotiated);
    };
    return !res;
}
//------------------------------------------------------------------------------
// Unit Disconnection
//------------------------------------------------------------------------------
void CliDisconnect()
{
     Cli_Disconnect(Client);
}

//------------------------------------------------------------------------------
// Unit communication
//------------------------------------------------------------------------------

void init_all_the_things_snap7()
{
	Client=Cli_Create();
	Cli_SetAsCallback(Client, CliCompletion,NULL);
	if (CliConnect())
	{
	MultiWrite();
	};
}

void kill_all_the_things_snap7()
{
	CliDisconnect();
	Cli_Destroy(&Client);
}

//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------

/* Model defines, for some values different calculations need to be performed
 *
 * The transmission differences are 8 preamble bits (EPB) and 7 preamble bits (SPB)
 */
#define EPB 0
#define SPB 1

static int fineoffset_wh1080_callback(r_device *decoder, bitbuffer_t *bitbuffer) {
    data_t *data;
    const uint8_t *br;
    int msg_type; // 0=Weather 1=Datetime 2=UV/Light
    int sens_msg = 12; // 12=Weather/Time sensor  8=UV/Light sensor
    uint8_t bbuf[11]; // max 8 / 11 bytes needed
    int model; // 7 or 8 preamble bits


    if (bitbuffer->num_rows != 1) {
        return 0;
    }

    if(bitbuffer->bits_per_row[0] == 88) { // FineOffset WH1080/3080 Weather data msg
        model = EPB;
        sens_msg = 12;
        br = bitbuffer->bb[0];
    } else if(bitbuffer->bits_per_row[0] == 87) { // FineOffset WH1080/3080 Weather data msg (different version (newest?))
        model = SPB;
        sens_msg = 12;
        /* 7 bits of preamble, bit shift the whole buffer and fix the bytestream */
        bitbuffer_extract_bytes(bitbuffer, 0, 7,
        (uint8_t *)&bbuf+1, 10*8);
        br = bbuf;
        bbuf[0] = 0xFF;
    } else if(bitbuffer->bits_per_row[0] == 64) {  // FineOffset WH3080 UV/Light data msg
        model = EPB;
        sens_msg = 8;
        br = bitbuffer->bb[0];

    } else if(bitbuffer->bits_per_row[0] == 63) { // FineOffset WH3080 UV/Light data msg (different version (newest?))
        model = SPB;
        sens_msg = 8;
        /* 7 bits of preamble, bit shift the whole buffer and fix the bytestream */
        bitbuffer_extract_bytes(bitbuffer, 0, 7,
        (uint8_t *) & bbuf +1, 7*8);
        br = bbuf;
        bbuf[0] = 0xFF;
    } else {
        return 0;
    }

    if (decoder->verbose) {
        bitrow_print(bbuf, (sens_msg - 1) * 8);
    }

    if (br[0] != 0xff) {
        // preamble missing
        return 0;
    }

    if (sens_msg == 12) {
    if (br[10] != crc8(br, 10, CRC_POLY, CRC_INIT)) {
        // crc mismatch
        return 0;
    }

    } else {
    if (br[7] != crc8(br, 7, CRC_POLY, CRC_INIT)) {
        // crc mismatch
        return 0;
    }
    }

    if (br[0] == 0xff && (br[1] >> 4) == 0x0a) {
        msg_type = 0; // WH1080/3080 Weather msg
	I1[1] = 0;
    } else if (br[0] == 0xff && (br[1] >> 4) == 0x0b) {
        msg_type = 1; // WH1080/3080 Datetime msg
	I1[1] = 1;
    } else if (br[0] == 0xff && (br[1] >> 4) == 0x07) {
        msg_type = 2; // WH3080 UV/Light msg
	I1[1] = 2;
    } else {
        msg_type = -1;
	I1[1] = 8;
    }

//---------------------------------------------------------------------------------------
//-------- GETTING WEATHER SENSORS DATA -------------------------------------------------

    const float temperature = get_temperature(br);
    Temperature_PLC = get_temperature(br);
    const int humidity = get_humidity(br);
    Humidity_PLC = get_humidity(br);

    const char* direction_str = get_wind_direction_str(br);
    B5[1] = direction_str[0];
    B5[2] = direction_str[1];
    B5[3] = direction_str[2];

    const char* direction_deg = get_wind_direction_deg(br);

    // Select which metric system for *wind avg speed* and *wind gust* :

    // Wind average speed :

    //const float speed = get_wind_avg_ms((br)   // <--- Data will be shown in Meters/sec.
    //const float speed = get_wind_avg_mph((br)  // <--- Data will be shown in Mph
    const float speed = get_wind_avg_kmh(br);  // <--- Data will be shown in Km/h
    Speed_PLC = get_wind_avg_kmh(br);
    //const float speed = get_wind_avg_knot((br) // <--- Data will be shown in Knots

    // Wind gust speed :

    //const float gust = get_wind_gust_ms(br);   // <--- Data will be shown in Meters/sec.
    //const float gust = get_wind_gust_mph(br);  // <--- Data will be shown in Mph
    const float gust = get_wind_gust_kmh(br);  // <--- Data will be shown in km/h
    Gust_PLC = get_wind_gust_kmh(br);
    //const float gust = get_wind_gust_knot(br); // <--- Data will be shown in Knots

    const float rain = get_rainfall(br);
    Rain_PLC = get_rainfall(br);
    const int device_id = get_device_id(br);
    device_id_PLC = get_device_id(br);
    const char* battery = get_battery(br);

    //---------------------------------------------------------------------------------------
    //-------- GETTING UV DATA --------------------------------------------------------------

    const int uv_sensor_id = get_uv_sensor_id(br);
    const char* uv_status = get_uvstatus(br);
    const int uv_index = wh3080_uvi(br);

    //---------------------------------------------------------------------------------------
    //-------- GETTING LIGHT DATA -----------------------------------------------------------
    float wm;
    const float light = get_rawlight(br);
    const float lux = (get_rawlight(br)/10);
    const float fc = ((get_rawlight(br)/10.76)/10.0);
    if (model == SPB)
        wm = (get_rawlight(br)*0.00079);
    else //EPB
        wm = (get_rawlight(br)/6830);

    //---------------------------------------------------------------------------------------
    //-------- GETTING TIME DATA ------------------------------------------------------------

    char* signal = get_signal(br);
   //B10 = get_signal(br);


    const int hours = get_hours(br);
    hours_PLC = get_hours(br);
    const int minutes = get_minutes(br);
    minutes_PLC = get_minutes(br);
    const int seconds = get_seconds(br);
    seconds_PLC = get_seconds(br);
    const int year = 2000 + get_year(br);
    year_PLC = 2000 + get_year(br);
    const int month = get_month(br);
    month_PLC = get_month(br);
    const int day = get_day(br);
    day_PLC = get_day(br);

    //--------- PRESENTING DATA --------------------------------------------------------------

    if (msg_type == 0) {

        data = data_make(
                "model",     "",         DATA_STRING,    "Fine Offset Electronics WH1080/WH3080 Weather Station",
                "msg_type",    "Msg type",    DATA_INT,                    msg_type,
                "id",        "Station ID",    DATA_FORMAT,    "%d",        DATA_INT,    device_id,
                "temperature_C","Temperature",    DATA_FORMAT,    "%.01f C",    DATA_DOUBLE,    temperature,
                "humidity",    "Humidity",    DATA_FORMAT,    "%u %%",    DATA_INT,    humidity,
                "direction_str","Wind string",    DATA_STRING,                    direction_str,
                "direction_deg","Wind degrees",    DATA_STRING,                    direction_deg,
                "speed",    "Wind avg speed",DATA_FORMAT,    "%.02f",    DATA_DOUBLE,    speed,
                "gust",        "Wind gust",    DATA_FORMAT,    "%.02f",    DATA_DOUBLE,     gust,
                "rain",        "Total rainfall",DATA_FORMAT,    "%3.1f",    DATA_DOUBLE,     rain,
                "battery",    "Battery",    DATA_STRING,                    battery,
                "mic",             "Integrity",    DATA_STRING,    "CRC",
                NULL);
        decoder_output_data(decoder, data);
        return 1;

    } else if (msg_type == 1) {

        data = data_make(
                "model",    "",        DATA_STRING,    "Fine Offset Electronics WH1080/WH3080 Weather Station",
                "msg_type",    "Msg type",    DATA_INT,                msg_type,
                "id",        "Station ID",    DATA_FORMAT,    "%d",    DATA_INT,    device_id,
                "signal",    "Signal Type",    DATA_STRING,                signal,
                "hours",    "Hours",    DATA_FORMAT,    "%02d",    DATA_INT,    hours,
                "minutes",    "Minutes",    DATA_FORMAT,    "%02d",    DATA_INT,    minutes,
                "seconds",    "Seconds",    DATA_FORMAT,    "%02d",    DATA_INT,    seconds,
                "year",        "Year",    DATA_FORMAT,    "%02d",    DATA_INT,    year,
                "month",    "Month",    DATA_FORMAT,    "%02d",    DATA_INT,    month,
                "day",        "Day",    DATA_FORMAT,    "%02d",    DATA_INT,    day,
                "mic",       "Integrity",    DATA_STRING,    "CRC",
                NULL);
        decoder_output_data(decoder, data);
        return 1;

    } else {

        data = data_make(
                "model",    "",        DATA_STRING,    "Fine Offset Electronics WH3080 Weather Station",
                "msg_type",    "Msg type",    DATA_INT,                msg_type,
                "uv_sensor_id",    "UV Sensor ID",    DATA_FORMAT,    "%d",    DATA_INT,    uv_sensor_id,
                "uv_status",    "Sensor Status",DATA_STRING,                uv_status,
                "uv_index",    "UV Index",    DATA_INT,                uv_index,
                "lux",        "Lux",    DATA_FORMAT,    "%.1f",    DATA_DOUBLE,    lux,
                "wm",        "Watts/m",    DATA_FORMAT,    "%.2f",    DATA_DOUBLE,    wm,
                "fc",        "Foot-candles",    DATA_FORMAT,    "%.2f",    DATA_DOUBLE,    fc,
                "mic",             "Integrity",    DATA_STRING,    "CRC",
                NULL);
        decoder_output_data(decoder, data);
        return 1;
    }
//------------------------------------------------------------------------------
// Multi Write
//------------------------------------------------------------------------------
void MultiWrite()
{

     Int2Bytes(device_id_PLC,&I2[0]);
//Swap bytes from big endian to little endian
     tempB = I2[0];
     I2[0] = I2[1];
     I2[1] = tempB;

     float2Bytes(Temperature_PLC,&R3[0]);
//Swap bytes from big endian to little endian
     tempB = R3[0];
     R3[0] = R3[3];
     R3[3] = tempB;

     tempB = R3[1];
     R3[1] = R3[2];
     R3[2] = tempB;

     Int2Bytes(Humidity_PLC,&I4[0]);
//Swap bytes from big endian to little endian
     tempB = I4[0];
     I4[0] = I4[1];
     I4[1] = tempB;

//!!for char field battery modify directly the rtl433 function get_battery to write the characters directly to the correct place little-endian format B9
//!!for char field wind direction modify directly the rtl433 function get_wind_direction to write the characters directly to the correct place little-endian format B5
     float2Bytes(Speed_PLC,&R6[0]);
//Swap bytes from big endian to little endian
     tempB = R6[0];
     R6[0] = R6[3];
     R6[3] = tempB;

     tempB = R6[1];
     R6[1] = R6[2];
     R6[2] = tempB;
     float2Bytes(Gust_PLC,&R7[0]);
//Swap bytes from big endian to little endian
     tempB = R7[0];
     R7[0] = R7[3];
     R7[3] = tempB;

     tempB = R7[1];
     R7[1] = R7[2];
     R7[2] = tempB;
     float2Bytes(Rain_PLC,&R8[0]);
//Swap bytes from big endian to little endian
     tempB = R8[0];
     R8[0] = R8[3];
     R8[3] = tempB;

     tempB = R8[1];
     R8[1] = R8[2];
     R8[2] = tempB;

     Int2Bytes(hours_PLC,&I11[0]);
//Swap bytes from big endian to little endian
     tempB = I11[0];
     I11[0] = I11[1];
     I11[1] = tempB;

     Int2Bytes(minutes_PLC,&I12[0]);
//Swap bytes from big endian to little endian
     tempB = I12[0];
     I12[0] = I12[1];
     I12[1] = tempB;
     Int2Bytes(seconds_PLC,&I13[0]);
//Swap bytes from big endian to little endian
     tempB = I13[0];
     I13[0] = I13[1];
     I13[1] = tempB;

     Int2Bytes(year_PLC,&I14[0]);
//Swap bytes from big endian to little endian
     tempB = I14[0];
     I14[0] = I14[1];
     I14[1] = tempB;

     Int2Bytes(month_PLC,&I15[0]);
//Swap bytes from big endian to little endian
     tempB = I15[0];
     I15[0] = I15[1];
     I15[1] = tempB;

     Int2Bytes(day_PLC,&I16[0]);
//Swap bytes from big endian to little endian
     tempB = I16[0];
     I16[0] = I16[1];
     I16[1] = tempB;

     // Prepare struct
     TS7DataItem Items[16];

     // NOTE : *AMOUNT IS NOT SIZE* , it's the number of items
     // I1 - Msg Type
     Items[0].Area     =S7AreaDB;
     Items[0].WordLen  =S7WLByte;
     Items[0].DBNumber =92;
     Items[0].Start    =0;         // Starting from 0
     Items[0].Amount   =2;         // 2 Items (bytes)
     Items[0].pdata    =&I1;
     // I2 - Station ID
     Items[1].Area     =S7AreaDB;
     Items[1].WordLen  =S7WLByte;
     Items[1].DBNumber =92;
     Items[1].Start    =2;         // Starting from 2
     Items[1].Amount   =2;         // 2 Items (bytes)
     Items[1].pdata    =&I2;
     // R3 - Temperature
     Items[2].Area     =S7AreaDB;
     Items[2].WordLen  =S7WLByte;
     Items[2].DBNumber =92;
     Items[2].Start    =4;         // Starting from 4
     Items[2].Amount   =4;         // 4 Items (bytes)
     Items[2].pdata    =&R3;
     // I4 - Humidity
     Items[3].Area     =S7AreaDB;
     Items[3].WordLen  =S7WLByte;
     Items[3].DBNumber =92;
     Items[3].Start    =8;         // Starting from 8
     Items[3].Amount   =2;         // 2 Items (bytes)
     Items[3].pdata    =&I4;
     // B5 - Wind direction string
     Items[4].Area     =S7AreaDB;
     Items[4].WordLen  =S7WLByte;
     Items[4].DBNumber =92;
     Items[4].Start    =10;        // Starting from 10
     Items[4].Amount   =3;         // 3 Items (bytes)
     Items[4].pdata    =&B5;
     // R6 - Wind average speed
     Items[5].Area     =S7AreaDB;
     Items[5].WordLen  =S7WLByte;
     Items[5].DBNumber =92;
     Items[5].Start    =14;        // Starting from 14
     Items[5].Amount   =4;         // 4 Items (bytes)
     Items[5].pdata    =&R6;
     // R7 - Wind gust speed
     Items[6].Area     =S7AreaDB;
     Items[6].WordLen  =S7WLByte;
     Items[6].DBNumber =92;
     Items[6].Start    =18;        // Starting from 18
     Items[6].Amount   =4;         // 4 Items (bytes)
     Items[6].pdata    =&R7;
     // R8 - Total rainfall value increased with 0.3mm increments at once
     Items[7].Area     =S7AreaDB;
     Items[7].WordLen  =S7WLByte;
     Items[7].DBNumber =92;
     Items[7].Start    =22;        // Starting from 22
     Items[7].Amount   =4;         // 4 Items (bytes)
     Items[7].pdata    =&R8;
     // B9 - Battery status indication string
     Items[8].Area     =S7AreaDB;
     Items[8].WordLen  =S7WLByte;
     Items[8].DBNumber =92;
     Items[8].Start    =26;        // Starting from 26
     Items[8].Amount   =3;         // 3 Items (bytes)
     Items[8].pdata    =&B9;
     // B10 - Time signal type indication string
     Items[9].Area     =S7AreaDB;
     Items[9].WordLen  =S7WLByte;
     Items[9].DBNumber =92;
     Items[9].Start    =29;        // Starting from 29
     Items[9].Amount   =8;         // 8 Items (bytes)
     Items[9].pdata    =&B10;
     // I11 - Time - hours
     Items[10].Area     =S7AreaDB;
     Items[10].WordLen  =S7WLByte;
     Items[10].DBNumber =92;
     Items[10].Start    =38;        // Starting from 38
     Items[10].Amount   =2;         // 2 Items (bytes)
     Items[10].pdata    =&I11;
     // I12 - Time - minutes
     Items[11].Area     =S7AreaDB;
     Items[11].WordLen  =S7WLByte;
     Items[11].DBNumber =92;
     Items[11].Start    =40;        // Starting from 40
     Items[11].Amount   =2;         // 2 Items (bytes)
     Items[11].pdata    =&I12;
     // I13 - Time - seconds
     Items[12].Area     =S7AreaDB;
     Items[12].WordLen  =S7WLByte;
     Items[12].DBNumber =92;
     Items[12].Start    =42;        // Starting from 42
     Items[12].Amount   =2;         // 2 Items (bytes)
     Items[12].pdata    =&I13;
     // I14 - Time - years
     Items[13].Area     =S7AreaDB;
     Items[13].WordLen  =S7WLByte;
     Items[13].DBNumber =92;
     Items[13].Start    =44;        // Starting from 44
     Items[13].Amount   =2;         // 2 Items (bytes)
     Items[13].pdata    =&I14;
     // I15 - Time - months
     Items[14].Area     =S7AreaDB;
     Items[14].WordLen  =S7WLByte;
     Items[14].DBNumber =92;
     Items[14].Start    =46;        // Starting from 46
     Items[14].Amount   =2;         // 2 Items (bytes)
     Items[14].pdata    =&I15;
     // I16 - Time - days
     Items[15].Area     =S7AreaDB;
     Items[15].WordLen  =S7WLByte;
     Items[15].DBNumber =92;
     Items[15].Start    =48;        // Starting from 48
     Items[15].Amount   =2;         // 2 Items (bytes)
     Items[15].pdata    =&I16;


    if (msg_type == 0) {
	res=Cli_WriteMultiVars(Client, &Items[0], 9);
	}
    else if (msg_type == 1) {
	res=Cli_WriteMultiVars(Client, &Items[9], 7);
       }



     if (Check(res,"Multiwrite Vars"))
     {
        // Result of Client->WriteMultiVars is the "global result" of
        // the function, it's OK if something was exchanged.

        // But we need to check single Var results.
        // Let shall suppose that we ask for 3 vars, 2 of them are ok but
        // the 3rd is non-existent, we will have 2 results ok and 1 not ok.
	if (msg_type == 0) {
        printf("Dump I1 Msg_type - Var Result : %d\n",Items[0].Result);
        if (Items[0].Result==0)
            hexdump(&I1,2);
        printf("Dump I2 Station ID - Var Result : %d\n",Items[1].Result);
        if (Items[1].Result==0)
            hexdump(&I2,2);
        printf("Dump R3 Temperature - Var Result : %d\n",Items[2].Result);
        if (Items[2].Result==0)
            hexdump(&R3,4);
        printf("Dump I4 Humidity - Var Result : %d\n",Items[3].Result);
        if (Items[3].Result==0)
            hexdump(&I4,2);
        printf("Dump B5 Wind direction string - Var Result : %d\n",Items[4].Result);
        if (Items[4].Result==0)
            hexdump(&B5,3);
        printf("Dump R6 Wind average speed - Var Result : %d\n",Items[5].Result);
        if (Items[5].Result==0)
            hexdump(&R6,4);
        printf("Dump R7 Wind gust speed - Var Result : %d\n",Items[6].Result);
        if (Items[6].Result==0)
            hexdump(&R7,4);
        printf("Dump R8 Rainfall - Var Result : %d\n",Items[7].Result);
        if (Items[7].Result==0)
            hexdump(&R8,4);
        printf("Dump B9 Battery status - Var Result : %d\n",Items[8].Result);
        if (Items[8].Result==0)
            hexdump(&B9,3);
	}
	if (msg_type == 1) {
        printf("Dump I11 Time - hours  - Var Result : %d\n",Items[10].Result);
        if (Items[10].Result==0)
            hexdump(&I11,2);
        printf("Dump I12 Time - minutes  - Var Result : %d\n",Items[11].Result);
        if (Items[11].Result==0)
            hexdump(&I12,2);
        printf("Dump I13 Time - seconds  - Var Result : %d\n",Items[12].Result);
        if (Items[12].Result==0)
            hexdump(&I13,2);
        printf("Dump I14 Time - year  - Var Result : %d\n",Items[13].Result);
        if (Items[13].Result==0)
            hexdump(&I14,2);
        printf("Dump I15 Time - month  - Var Result : %d\n",Items[14].Result);
        if (Items[14].Result==0)
            hexdump(&I15,2);
        printf("Dump I16 Time - day  - Var Result : %d\n",Items[15].Result);
        if (Items[15].Result==0)
            hexdump(&I16,2);
	}
     };
}


}

static char *output_fields[] = {
    "model",
    "id",
    "temperature_C",
    "humidity",
    "direction_str",
    "direction_deg",
    "speed",
    "gust",
    "rain",
    "msg_type",
    "signal",
    "hours",
    "minutes",
    "seconds",
    "year",
    "month",
    "day",
    "battery",
    "sensor_code",
    "uv_status",
    "uv_index",
    "lux",
    "wm",
    "fc",
    NULL
};

r_device fineoffset_wh1080 = {
    .name           = "Fine Offset Electronics WH1080/WH3080 Weather Station",
    .modulation     = OOK_PULSE_PWM,
    .short_width    = 544,     // Short pulse 544µs, long pulse 1524µs, fixed gap 1036µs
    .long_width     = 1524,    // Maximum pulse period (long pulse + fixed gap)
    .reset_limit    = 2800,    // We just want 1 package
    .decode_fn      = &fineoffset_wh1080_callback,
    .disabled       = 0,
    .fields         = output_fields,
};


