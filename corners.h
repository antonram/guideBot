#ifndef CORNERS_H
#define CORNERS_H

typedef struct Corner {
    uint32_t latitude;
    uint32_t longitude;
    volatile struct Corner *upper;
    volatile struct Corner *right;
    volatile struct Corner *down;
    volatile struct Corner *left;
    volatile char visited;
} Corner;

#endif
