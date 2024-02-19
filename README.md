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

Like all Boid systems, you can tweak the relative weights of the forces to make the system act differently. These global parameters are in the `boids.h` header file. 
## Tools

### Hardware

- ESP32 TTGO
- USB-C cable with data pins

### Software

- Arduino IDE with packages

### How to run
1. Install packages. Configure board, port & upload speed.
2. Load the file onto the board using the Arduino IDE.
3. Let the HeartBoids soothe you.

## Demo

![heartboids](https://github.com/avahajr/heartboids/assets/100488972/24999d19-3bc0-4e76-914e-8d190345ec4e)

Yearning for more HeartBoids? Here you go!
[read the blog post](https://avahajr.github.io/ces-portfolio/)
[HeartBoids in video](https://youtu.be/NogfVlpFZs0)
[Whole installation](https://youtu.be/yLaZCF0YMJ8)



