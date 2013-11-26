#ifndef RGB_H
#define RGB_H

#include <cassert>

struct rgb {
    float r,g,b;

    float & operator [] (int k) {
        switch(k) {
        case 0: return r;
        case 1: return g;
        case 2: return b;
        }
        assert(false);
        return r;
    }
};


#endif
