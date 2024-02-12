#include <iostream>
#include <utility>
#include <vector>
#include <cmath>
using namespace std;
#include <stdlib.h>
#include <stdio.h>
#include <TFT_eSPI.h>
#include <Arduino.h>
// Serial.begin(115200);

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite background = TFT_eSprite(&tft);

const int BOID_DIM = 14;
const float MAX_SPEED = 3;
const float MAX_FORCE = 0.1;
const int DESIRED_SEP = 30;

struct Boid {
  pair<float, float> pos;
  pair<float, float> vel;
  pair<float, float> acc;

  TFT_eSprite *boidSpr;

  Boid() {
    boidSpr = new TFT_eSprite(&tft);
  }

  ~Boid() {
    delete boidSpr;  // Deallocate the sprite in the destructor
  }
};

vector<Boid *> allBoids;

float getMag(pair<float, float> p) {
  return sqrt(p.first * p.first + p.second * p.second);
}

pair<float, float> rescalePair(pair<float, float> vect, float desiredMag) {

  float currentMag = getMag(vect);

  vect.first = (vect.first / currentMag) * desiredMag;
  vect.second = (vect.second / currentMag) * desiredMag;
  return vect;
}

// calculates the distance between the center of two boids.
float distanceBetweenBoids(Boid &boid1, Boid &boid2) {
  int x1 = boid1.pos.first + BOID_DIM / 2;
  int y1 = boid1.pos.second + BOID_DIM / 2;
  int x2 = boid2.pos.first + BOID_DIM / 2;
  int y2 = boid2.pos.second + BOID_DIM / 2;

  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

/* 
  Given a boid, return the average seperation force for nearby boids.
  Pointers to all other boids are stored in the allBoids vector, which 
  is a global variable.
*/
pair<float, float> getSeparationForce(Boid &currentBoid) {
  pair<float, float> sum = { 0, 0 };
  int count = 0;

  for (Boid *otherBoidPtr : allBoids) {
    Boid &otherBoid = *otherBoidPtr;
    float d = distanceBetweenBoids(currentBoid, otherBoid);
    if (d <= DESIRED_SEP && otherBoidPtr != &currentBoid) {
      // dist
      int dx = currentBoid.pos.first - otherBoid.pos.first;
      int dy = currentBoid.pos.second - otherBoid.pos.second;

      pair<float, float> diff = rescalePair({ dx, dy }, 1.0 / d);

      sum.first += diff.first;
      sum.second += diff.second;
      count++;
    }
  }

  if (count > 0) {
    sum = rescalePair(sum, MAX_SPEED);

    // Subtract the current velocity to get the steering force
    sum.first -= currentBoid.vel.first;
    sum.second -= currentBoid.vel.second;
  }

  if (getMag(sum) > MAX_FORCE) {
    sum = rescalePair(sum, MAX_FORCE);
  }

  return sum;
}
