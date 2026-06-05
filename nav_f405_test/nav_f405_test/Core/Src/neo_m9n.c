#include "neo_m9n.h"

#include <string.h>

#define NEO_M9N_SPI_FILL_BYTE 0xFFu
#define NEO_M9N_POLL_CHUNK    32u
#define NMEA_FIELD_MAX        20u

static void CS_Low(NEO_M9N_Handle_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static void CS_High(NEO_M9N_Handle_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

static int HexValue(char c)
{
    if ((c >= '0') && (c <= '9')) return c - '0';
    if ((c >= 'A') && (c <= 'F')) return c - 'A' + 10;
    if ((c >= 'a') && (c <= 'f')) return c - 'a' + 10;
    return -1;
}

static uint8_t NMEA_ChecksumOk(const char *sentence)
{
    uint8_t checksum = 0;
    const char *p;
    const char *star;
    int hi;
    int lo;

    if (sentence[0] != '$') {
        return 0;
    }

    p = &sentence[1];
    star = p;
    while ((*star != '\0') && (*star != '*')) {
        checksum ^= (uint8_t)*star;
        star++;
    }

    if (star[0] != '*') {
        return 0;
    }

    hi = HexValue(star[1]);
    lo = HexValue(star[2]);
    if ((hi < 0) || (lo < 0)) {
        return 0;
    }

    return checksum == (uint8_t)((hi << 4) | lo);
}

static uint8_t NMEA_IsType(const char *sentence, const char *type)
{
    return (sentence[0] == '$') &&
           (sentence[3] == type[0]) &&
           (sentence[4] == type[1]) &&
           (sentence[5] == type[2]);
}

static uint8_t NMEA_GetField(const char *sentence,
                             uint8_t index,
                             char *out,
                             uint8_t out_size)
{
    uint8_t current = 0;
    const char *start = &sentence[1];
    const char *end;
    uint8_t len;

    if ((out_size == 0) || (sentence[0] != '$')) {
        return 0;
    }

    out[0] = '\0';
    while ((*start != '\0') && (*start != '*')) {
        end = start;
        while ((*end != '\0') && (*end != ',') && (*end != '*')) {
            end++;
        }

        if (current == index) {
            len = (uint8_t)(end - start);
            if (len >= out_size) {
                len = (uint8_t)(out_size - 1u);
            }
            memcpy(out, start, len);
            out[len] = '\0';
            return 1;
        }

        current++;
        if (*end != ',') {
            break;
        }
        start = end + 1;
    }

    return 0;
}

static uint8_t ParseUint8(const char *s)
{
    uint16_t value = 0;

    while ((*s >= '0') && (*s <= '9')) {
        value = (uint16_t)((value * 10u) + (uint16_t)(*s - '0'));
        if (value > 255u) {
            return 255u;
        }
        s++;
    }

    return (uint8_t)value;
}

static int32_t ParseDecimalCm(const char *s)
{
    int32_t sign = 1;
    int32_t whole = 0;
    int32_t frac = 0;
    uint8_t frac_digits = 0;

    if (*s == '-') {
        sign = -1;
        s++;
    }

    while ((*s >= '0') && (*s <= '9')) {
        whole = (whole * 10) + (*s - '0');
        s++;
    }

    if (*s == '.') {
        s++;
        while ((*s >= '0') && (*s <= '9') && (frac_digits < 2u)) {
            frac = (frac * 10) + (*s - '0');
            frac_digits++;
            s++;
        }
    }

    while (frac_digits < 2u) {
        frac *= 10;
        frac_digits++;
    }

    return sign * ((whole * 100) + frac);
}

static int32_t ParseCoordE7(const char *value, char hemi)
{
    uint32_t whole = 0;
    uint32_t frac = 0;
    uint32_t scale = 1;
    uint32_t degrees;
    uint32_t minutes_whole;
    int64_t minutes_e7;
    int64_t coord_e7;

    if (value[0] == '\0') {
        return 0;
    }

    while ((*value >= '0') && (*value <= '9')) {
        whole = (whole * 10u) + (uint32_t)(*value - '0');
        value++;
    }

    if (*value == '.') {
        value++;
        while ((*value >= '0') && (*value <= '9') && (scale < 1000000u)) {
            frac = (frac * 10u) + (uint32_t)(*value - '0');
            scale *= 10u;
            value++;
        }
    }

    minutes_whole = whole % 100u;
    degrees = whole / 100u;
    minutes_e7 = ((int64_t)minutes_whole * 10000000LL) +
                 (((int64_t)frac * 10000000LL) / (int64_t)scale);
    coord_e7 = ((int64_t)degrees * 10000000LL) + (minutes_e7 / 60LL);

    if ((hemi == 'S') || (hemi == 'W')) {
        coord_e7 = -coord_e7;
    }

    return (int32_t)coord_e7;
}

static void CopyUtcTime(NEO_M9N_Fix_t *fix, const char *time)
{
    uint8_t i = 0;

    while ((time[i] != '\0') && (time[i] != '.') && (i < 10u)) {
        fix->utc_time[i] = time[i];
        i++;
    }
    fix->utc_time[i] = '\0';
}

static void ParseGGA(const char *sentence, NEO_M9N_Fix_t *fix)
{
    char field[NMEA_FIELD_MAX];
    char hemi;

    if (NMEA_GetField(sentence, 1, field, sizeof(field))) {
        CopyUtcTime(fix, field);
    }

    if (NMEA_GetField(sentence, 6, field, sizeof(field))) {
        fix->fix_quality = ParseUint8(field);
        fix->valid = (fix->fix_quality > 0u) ? 1u : 0u;
    }

    if (NMEA_GetField(sentence, 7, field, sizeof(field))) {
        fix->satellites = ParseUint8(field);
    }

    if (NMEA_GetField(sentence, 2, field, sizeof(field))) {
        char lat[NMEA_FIELD_MAX];
        memcpy(lat, field, sizeof(lat));
        if (NMEA_GetField(sentence, 3, field, sizeof(field))) {
            hemi = field[0];
            fix->latitude_e7 = ParseCoordE7(lat, hemi);
        }
    }

    if (NMEA_GetField(sentence, 4, field, sizeof(field))) {
        char lon[NMEA_FIELD_MAX];
        memcpy(lon, field, sizeof(lon));
        if (NMEA_GetField(sentence, 5, field, sizeof(field))) {
            hemi = field[0];
            fix->longitude_e7 = ParseCoordE7(lon, hemi);
        }
    }

    if (NMEA_GetField(sentence, 9, field, sizeof(field))) {
        fix->altitude_cm = ParseDecimalCm(field);
    }

    fix->last_update_ms = HAL_GetTick();
}

static void ParseRMC(const char *sentence, NEO_M9N_Fix_t *fix)
{
    char field[NMEA_FIELD_MAX];
    char hemi;

    if (NMEA_GetField(sentence, 1, field, sizeof(field))) {
        CopyUtcTime(fix, field);
    }

    if (NMEA_GetField(sentence, 2, field, sizeof(field))) {
        fix->valid = (field[0] == 'A') ? 1u : 0u;
    }

    if (fix->valid != 0u) {
        if (NMEA_GetField(sentence, 3, field, sizeof(field))) {
            char lat[NMEA_FIELD_MAX];
            memcpy(lat, field, sizeof(lat));
            if (NMEA_GetField(sentence, 4, field, sizeof(field))) {
                hemi = field[0];
                fix->latitude_e7 = ParseCoordE7(lat, hemi);
            }
        }

        if (NMEA_GetField(sentence, 5, field, sizeof(field))) {
            char lon[NMEA_FIELD_MAX];
            memcpy(lon, field, sizeof(lon));
            if (NMEA_GetField(sentence, 6, field, sizeof(field))) {
                hemi = field[0];
                fix->longitude_e7 = ParseCoordE7(lon, hemi);
            }
        }
    }

    fix->last_update_ms = HAL_GetTick();
}

static void ParseSentence(NEO_M9N_Handle_t *dev, NEO_M9N_Fix_t *fix)
{
    if (!NMEA_ChecksumOk(dev->sentence)) {
        return;
    }

    dev->sentences_rx++;

    if (NMEA_IsType(dev->sentence, "GGA")) {
        ParseGGA(dev->sentence, fix);
    } else if (NMEA_IsType(dev->sentence, "RMC")) {
        ParseRMC(dev->sentence, fix);
    }
}

static void ProcessByte(NEO_M9N_Handle_t *dev, NEO_M9N_Fix_t *fix, uint8_t b)
{
    if ((b == NEO_M9N_SPI_FILL_BYTE) || (b == 0x00u)) {
        return;
    }

    if (b == '$') {
        dev->sentence_len = 0;
        dev->sentence[dev->sentence_len++] = (char)b;
        return;
    }

    if (dev->sentence_len == 0u) {
        return;
    }

    if (b == '\r') {
        return;
    }

    if (b == '\n') {
        dev->sentence[dev->sentence_len] = '\0';
        ParseSentence(dev, fix);
        dev->sentence_len = 0;
        return;
    }

    if ((b < 32u) || (b > 126u)) {
        dev->sentence_len = 0;
        return;
    }

    if (dev->sentence_len >= (sizeof(dev->sentence) - 1u)) {
        dev->sentence_len = 0;
        return;
    }

    dev->sentence[dev->sentence_len++] = (char)b;
}

void NEO_M9N_Init(NEO_M9N_Handle_t *dev,
                  SPI_HandleTypeDef *hspi,
                  GPIO_TypeDef *cs_port,
                  uint16_t cs_pin)
{
    memset(dev, 0, sizeof(*dev));
    dev->hspi = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin = cs_pin;
    CS_High(dev);
}

HAL_StatusTypeDef NEO_M9N_Poll(NEO_M9N_Handle_t *dev,
                               NEO_M9N_Fix_t *fix,
                               uint16_t max_bytes)
{
    uint8_t tx[NEO_M9N_POLL_CHUNK];
    uint8_t rx[NEO_M9N_POLL_CHUNK];
    uint16_t remaining = max_bytes;
    HAL_StatusTypeDef status;

    memset(tx, NEO_M9N_SPI_FILL_BYTE, sizeof(tx));

    while (remaining > 0u) {
        uint16_t chunk = remaining;
        if (chunk > NEO_M9N_POLL_CHUNK) {
            chunk = NEO_M9N_POLL_CHUNK;
        }

        CS_Low(dev);
        status = HAL_SPI_TransmitReceive(dev->hspi, tx, rx, chunk, 10);
        CS_High(dev);

        if (status != HAL_OK) {
            return status;
        }

        for (uint16_t i = 0; i < chunk; i++) {
            ProcessByte(dev, fix, rx[i]);
            if ((rx[i] != NEO_M9N_SPI_FILL_BYTE) && (rx[i] != 0x00u)) {
                dev->bytes_rx++;
            }
        }

        remaining = (uint16_t)(remaining - chunk);
    }

    return HAL_OK;
}
