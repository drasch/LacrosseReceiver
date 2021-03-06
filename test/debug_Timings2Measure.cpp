//
// Created by Emanuele on 29/06/2018.
//
#ifdef DEBUG

#include <iostream>
#include <cstring>
#include "Timings2Measure.h"

struct packet : timings_packet {
    long unsigned int timings[200];
    uint32_t peekTiming(size_t pos) override {
        return (pos >= 0 && pos < size)? (uint32_t) timings[pos] : 0xFFFFFFFF;
    }
};

inline const char* mTypeToStr(measureType mType)
{
    switch (mType) {
        case TEMPERATURE: return "TMP";
        case HUMIDITY:    return "HUM";
        default:          return "???";
    }
}

int main( int argc, char **argv) {
    int nTests, nTimings, units, sensorAddr, decimals;
    char mType[4];
    unsigned long msec;
    Timings2Measure t2m;

    freopen("test_Timings2Measure.dat", "r", stdin);
    std::cin >> nTests;
    int ok = 0;
    printf("N. misure di test: %d\r\n", nTests);
    for(int t = 0; t < nTests; t++) {
        scanf("%lu %d %d.%d %d %s", &msec, &nTimings, &units, &decimals, &sensorAddr, mType);
        packet pk;
        pk.msec = (uint32_t) msec;
        pk.size = (uint32_t) nTimings;
        for(int tm = 0; tm < pk.size; tm++) scanf("%lu", &pk.timings[tm]);

        measure m = t2m.getMeasure(&pk);
        bool check = (m.sensorAddr == sensorAddr
                && strcmp(mTypeToStr(m.type), (const char*)mType) == 0
                && m.units == units
                && m.decimals == decimals);
        if (check) ok++;

        printf("%d: %d %s %d.%d ", m.msec, m.sensorAddr, mTypeToStr(m.type), m.units, m.decimals);
        std::cout << (check? "OK" : "NO") << "\n";
    }
    printf("\n OK: %d/%d\n", ok, nTests);
}

#endif
