#include <string.h>
#include <ctype.h>

#include "aircraft_types.h"

// Kody typow ICAO ("t" w danych ADS-B) rozpoznawane jako smiglowiec -
// dopasowanie dokladne (nie substring), bo to sa oficjalne, krotkie
// designatory typu (np. "R44", "H60"), a nie fragmenty nazw.
static const char *s_helicopter_types[] = {
    // Airbus / Eurocopter
    "AS50", "AS55", "AS32", "AS35", "AS65",
    "EC20", "EC30", "EC35", "EC45", "EC55", "EC75",
    "H125", "H130", "H135", "H145", "H160", "H175", "H225",
    // Robinson
    "R22", "R44", "R66",
    // Bell
    "B06", "B206", "B407", "B412", "B429", "B212", "B222", "B430", "B505", "UH1",
    // Leonardo / Agusta
    "A109", "A119", "A139", "AW09", "AW109", "AW119", "AW139", "AW149", "AW169", "AW189", "EH10",
    // PZL / Swidnik / Mil
    "W3", "SW4", "MI2", "MI8", "MI14", "MI17", "MI24", "MI28", "KA52",
    // Sikorsky
    "H60", "UH60", "S70", "S76", "S92", "H53",
    // Inne cywilne
    "CABR", "G2CA", "H269", "S300", "B063", "MD52", "MD50", "MD60",
};
#define NUM_HELICOPTER_TYPES (sizeof(s_helicopter_types) / sizeof(s_helicopter_types[0]))

bool is_helicopter(const char *type) {
    if (!type || !type[0]) return false;

    char norm[16];
    size_t len = 0;
    for (; type[len] && len < sizeof(norm) - 1; len++) {
        norm[len] = (char)toupper((unsigned char)type[len]);
    }
    norm[len] = '\0';
    while (len > 0 && norm[len - 1] == ' ') norm[--len] = '\0';

    for (size_t i = 0; i < NUM_HELICOPTER_TYPES; i++) {
        if (strcmp(norm, s_helicopter_types[i]) == 0) return true;
    }
    return false;
}
