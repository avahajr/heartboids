#include <iostream>
#include <utility>
#include <vector>
using namespace std;
#include <stdlib.h>
#include <stdio.h>
#include <TFT_eSPI.h>
#include <Arduino.h>
#include "heart.h"
#include "boids.h"

const int NUM_BOIDS = 35;
const int REFRESH_DELAY = 75;


void setup() {
  tft.init();
  tft.setRotation(1);
  Serial.begin(115200);
  tft.fillScreen(TFT_BLACK);
  tft.setSwapBytes(true);
  background.createSprite(tft.width(), tft.height());
  background.fillSprite(TFT_BLACK);
  background.pushSprite(0, 0);
  Serial.print("Align: ");
  Serial.println(ALIGN_WEIGHT);
  Serial.print("Sep: ");
  Serial.println(SEP_WEIGHT);
  Serial.print("Cohes: ");
  Serial.println(COHES_WEIGHT);

  for (int i = 0; i < NUM_BOIDS; i++) {
    Boid *newBoid = new Boid();  // Allocate Boid on the heap
    newBoid->pos = make_pair(random(0, tft.width() + 1), random(0, tft.height() + 1));
    newBoid->vel = make_pair(random(-MAX_SPEED, MAX_SPEED + 1), random(-MAX_SPEED, MAX_SPEED + 1));
    newBoid->acc = { 0, 0 };
    newBoid->boidSpr->createSprite(BOID_DIM, BOID_DIM);
    newBoid->boidSpr->setSwapBytes(true);
    newBoid->boidSpr->pushImage(0, 0, BOID_DIM, BOID_DIM, heart14);
    newBoid->boidSpr->pushSprite(static_cast<int>(newBoid->pos.first), static_cast<int>(newBoid->pos.second), TFT_BLACK);
    allBoids.push_back(newBoid);
  }
}

void loop() {
  background.fillSprite(TFT_BLACK);
  for (int i = 0; i < NUM_BOIDS; i++) {
    updateBoid(*allBoids[i]);
  }
  background.pushSprite(0, 0);
  // delay(REFRESH_DELAY);
}


void updateBoid(Boid &currentBoid) {

  currentBoid.boidSpr->pushImage(0, 0, BOID_DIM, BOID_DIM, heart14);
  currentBoid.boidSpr->pushToSprite(&background, currentBoid.pos.first, currentBoid.pos.second, TFT_BLACK);

  // get the seperation force
  pair<float, float> sepForce = getSeparationForce(currentBoid);
  pair<float, float> alignForce = getAlignmentForce(currentBoid);
  pair<float, float> cohesForce = getCohesionForce(currentBoid);


  // Serial.println(sqrt(sepForce.first * sepForce.first + sepForce.second * sepForce.second));
  // and apply it
  currentBoid.acc.first += sepForce.first + alignForce.first + cohesForce.first;
  currentBoid.acc.second += sepForce.second + alignForce.second + cohesForce.second;

  // based on the observed acceleration, update the boid position and velocity
  currentBoid.vel.first += currentBoid.acc.first;
  currentBoid.vel.second += currentBoid.acc.second;

  if (getMag(currentBoid.vel) > MAX_SPEED) {
    currentBoid.vel = rescalePair(currentBoid.vel, MAX_SPEED);
  }
  currentBoid.pos.first += currentBoid.vel.first;
  currentBoid.pos.second += currentBoid.vel.second;


  // boundary checks & wraparound
  if (currentBoid.pos.first > tft.width()) {
    currentBoid.pos.first = -BOID_DIM;  // Wrap around to the left side
  } else if (currentBoid.pos.second > tft.height()) {
    currentBoid.pos.second = -BOID_DIM;  // Wrap around to the top side
  }
  // Check if the boid has gone past the left edge of the screen
  else if (currentBoid.pos.first < 0 - BOID_DIM) {
    currentBoid.pos.first = tft.width();  // Wrap around to the right side
  }
  // Check if the boid has gone past the top edge of the screen
  else if (currentBoid.pos.second < 0 - BOID_DIM) {
    currentBoid.pos.second = tft.height();  // Wrap around to the bottom side
  }
}
