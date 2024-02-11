#include <iostream>
#include <utility>
#include <vector>
using namespace std;
#include <stdlib.h>
#include <stdio.h>
#include <TFT_eSPI.h>
#include <Arduino.h>
#include "esp_random.h"
#include "heart.h"
#include "bootloader_random.h"

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite background = TFT_eSprite(&tft);
int REFRESH_DELAY = 50;
int MAX_SPEED = 4;

struct Boid {
  pair<int, int> pos;
  pair<int, int> vel;
  pair<int, int> acc;

  TFT_eSprite *boidSpr;

  Boid() {
    boidSpr = new TFT_eSprite(&tft);
  }

  ~Boid() {
    delete boidSpr;  // Deallocate the sprite in the destructor
  }
};

int BOID_SCALAR = 1;
int NUM_BOIDS = 20;
vector<Boid *> allBoids;

void setup() {
  tft.init();
  tft.setRotation(1);
  Serial.begin(115200);
  tft.fillScreen(TFT_BLACK);
  tft.setSwapBytes(true);
  background.createSprite(tft.width(), tft.height());
  background.fillSprite(TFT_BLACK);
  background.pushSprite(0, 0);
  bootloader_random_enable();  // Enable the internal entropy source

  for (int i = 0; i < NUM_BOIDS; i++) {
    Boid *newBoid = new Boid();  // Allocate Boid on the heap
    newBoid->pos = make_pair(random(0,tft.width() + 1),random(0, tft.height()+1));
    newBoid->vel = make_pair(random(-MAX_SPEED, MAX_SPEED + 1),random(-MAX_SPEED, MAX_SPEED + 1));
    newBoid->acc = { 0, 0 };
    newBoid->boidSpr->createSprite(14, 14);
    newBoid->boidSpr->setSwapBytes(true);
    newBoid->boidSpr->pushImage(0, 0, 14, 14, heart14);
    newBoid->boidSpr->pushSprite(newBoid->pos.first, newBoid->pos.second, TFT_BLACK);
    allBoids.push_back(newBoid);
  }
  // bootloader_random_disable();
}

void loop() {
  background.fillSprite(TFT_BLACK);
  for (int i = 0; i < NUM_BOIDS; i++) {
    updateBoid(*allBoids[i]);
  }
  background.pushSprite(0, 0);
  delay(REFRESH_DELAY);
}


void updateBoid(Boid &currentBoid) {
  // render the position of the sprite from the last iteration.
  // currentBoid.boidSpr->pushSprite(currentBoid.pos.first - (5 * BOID_SCALAR), currentBoid.pos.second - (4 * BOID_SCALAR), TFT_BLACK);

  currentBoid.boidSpr->pushImage(0, 0, 14, 14, heart14);
  currentBoid.boidSpr->pushToSprite(&background, currentBoid.pos.first, currentBoid.pos.second, TFT_BLACK);

  currentBoid.vel.first += currentBoid.acc.first;
  currentBoid.vel.second += currentBoid.acc.second;
  currentBoid.pos.first += currentBoid.vel.first;
  currentBoid.pos.second += currentBoid.vel.second;


  if (currentBoid.pos.first > tft.width()) {
    currentBoid.pos.first = -14;  // Wrap around to the left side
  } else if (currentBoid.pos.second > tft.height()) {
    currentBoid.pos.second = -14;  // Wrap around to the top side
  }
  // Check if the boid has gone past the left edge of the screen
  else if (currentBoid.pos.first < 0-14) {
    currentBoid.pos.first = tft.width();  // Wrap around to the right side
  }
  // Check if the boid has gone past the top edge of the screen
  else if (currentBoid.pos.second < 0-14) {
    currentBoid.pos.second = tft.height();  // Wrap around to the bottom side
  }

  Serial.println(currentBoid.pos.first);
}
