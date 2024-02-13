# HeartBoids

Ava Hajratwala

## Creative Embedded Systems - Installation 1

_aka, my first ever C++ program!_

## Overview

In this project, I used Craig Reynold's 1986 [Boids](https://en.wikipedia.org/wiki/Boids) algorithm to animate the movement of heart sprites onto my ESP32 TTGO screen.

## Implementation

At the core of my implementation is the `Boid` struct:

```cpp
struct Boid {
pair<float, float> pos;
pair<float, float> vel;
pair<float, float> acc;

TFT_eSprite *boidSpr;

Boid() {
boidSpr = new TFT_eSprite(&tft);
}

~Boid() {
delete boidSpr;
}
};
```

Pointers to each created Boid are stored in a vector to keep track of them.

## Tools

### Hardware

- ESP32 TTGO
- USB-C cable with data pins

### Software

- Arduino IDE with packages

## Demo

![heartboids](https://github.com/avahajr/heartboids/assets/100488972/24999d19-3bc0-4e76-914e-8d190345ec4e)
