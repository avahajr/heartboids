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

const int BOID_DIM = 14;  // size of the sprite
const float MAX_SPEED = 1.5;
const float MAX_FORCE = 0.25;
const int DESIRED_SEP = 20;    // radius within which separation force activates
const int NEIGHBOR_DIST = 25;  // radius within which boids are in a flock.
const bool RANDOM = true;

// float ALIGN_WEIGHT;
// float SEP_WEIGHT;
// float COHES_WEIGHT;

// weighting the forces

float ALIGN_WEIGHT = .7;
float SEP_WEIGHT = 1.3;
float COHES_WEIGHT = 0.35;
// } else {
//   ALIGN_WEIGHT = (float)(random(1, 10) / ((float)random(1, 10)));
//   SEP_WEIGHT = (float)(random(1, 10) / ((float)random(1, 10)));
//   COHES_WEIGHT = (float)(random(1, 10) / ((float)random(1, 10)));
// }
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
  float x1 = boid1.pos.first + BOID_DIM / 2;
  float y1 = boid1.pos.second + BOID_DIM / 2;
  float x2 = boid2.pos.first + BOID_DIM / 2;
  float y2 = boid2.pos.second + BOID_DIM / 2;

  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

pair<float, float> limitMag(pair<float, float> vect, float maxMag) {
  if (getMag(vect) > maxMag) {
    return rescalePair(vect, maxMag);
  }
  return vect;
}
pair<float, float> weightPairBy(pair<float, float> vect, float w) {
  vect.first *= w;
  vect.second *= w;
  return vect;
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
    if (d < DESIRED_SEP && otherBoidPtr != &currentBoid) {
      // dist
      float dx = currentBoid.pos.first - otherBoid.pos.first;
      float dy = currentBoid.pos.second - otherBoid.pos.second;

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
    sum = limitMag(sum, MAX_FORCE);
  }

  sum = weightPairBy(sum, SEP_WEIGHT);
  return sum;
}

// Steer in the direction of your neighbors.
pair<float, float> getAlignmentForce(Boid &currentBoid) {
  int count = 0;
  pair<float, float> sum = { 0, 0 };
  for (Boid *otherBoidPtr : allBoids) {
    Boid &otherBoid = *otherBoidPtr;
    float d = distanceBetweenBoids(currentBoid, otherBoid);
    if (otherBoidPtr != &currentBoid && d < NEIGHBOR_DIST) {
      sum.first += otherBoidPtr->vel.first;
      sum.second += otherBoidPtr->vel.second;
      count++;
    }
  }
  if (count > 0) {
    sum = rescalePair(sum, MAX_SPEED);
    sum.first -= currentBoid.vel.first;
    sum.second -= currentBoid.vel.second;

    sum = limitMag(sum, MAX_FORCE);
  }
  sum = weightPairBy(sum, ALIGN_WEIGHT);

  return sum;
}
pair<float, float> getCohesionForce(Boid &currentBoid) {
  pair<float, float> sum = { 0, 0 };
  int count = 0;
  for (Boid *otherBoidPtr : allBoids) {
    Boid &otherBoid = *otherBoidPtr;
    float d = distanceBetweenBoids(currentBoid, otherBoid);
    if (otherBoidPtr != &currentBoid && d < NEIGHBOR_DIST) {
      sum.first += otherBoidPtr->pos.first;
      sum.second += otherBoidPtr->pos.second;
      count++;
    }
  }
  if (count > 0) {
    sum.first /= count;
    sum.second /= count;

    // now sum represents the average position. We want to seek this position (sum=target).
    sum.first -= currentBoid.pos.first;
    sum.second -= currentBoid.pos.second;

    sum = rescalePair(sum, MAX_SPEED);

    sum.first -= currentBoid.vel.first;
    sum.second -= currentBoid.vel.second;

    sum = limitMag(sum, MAX_FORCE);
  }

  sum = weightPairBy(sum, COHES_WEIGHT);
  return sum;
}