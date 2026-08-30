#ifndef AIRPORTS_H
#define AIRPORTS_H

#include <stddef.h>
#include <stdint.h>

#define APT_TYPE_CIVIL (1 << 0) // Commercial / major passenger hubs
#define APT_TYPE_MIL   (1 << 1) // Air bases / military
#define APT_TYPE_GA    (1 << 2) // Aeroclubs, small airfields and airstrips (General Aviation / Regional)
#define APT_TYPE_ALL   (APT_TYPE_CIVIL | APT_TYPE_MIL | APT_TYPE_GA)

typedef struct {
    const char icao[5];
    const char name[16];
    uint8_t type;
    float lat;
    float lon;
} airport_t;

extern const airport_t global_airports[];
extern const size_t GLOBAL_AIRPORTS_COUNT;

#endif // AIRPORTS_H
