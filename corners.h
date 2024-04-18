#ifndef CORNERS_H
#define CORNERS_H

typedef struct {
    int latitude;
    int longitude;
    struct Corner *upper;
    struct Corner *right;
    struct Corner *down;
    struct Corner *left;
} Corner;

#endif
