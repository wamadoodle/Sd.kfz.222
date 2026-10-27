#pragma once

#include <Arduino.h>
#include <pgmspace.h>

struct Note {
  uint16_t frequencyHz;
  uint16_t durationMs;
  uint16_t restMs;
};

constexpr uint16_t Ab3 = 233;
constexpr uint16_t Db4 = 311;
constexpr uint16_t G3 = 196;
constexpr uint16_t Gb3 = 208;
constexpr uint16_t F3 = 175;
constexpr uint16_t Db3 = 156;
constexpr uint16_t G4 = 392;
constexpr uint16_t F4 = 349;
constexpr uint16_t C4 = 262;
constexpr uint16_t D4 = 294;
constexpr uint16_t E4 = 330;
constexpr uint16_t Gb4 = 415;

const Note melody[] PROGMEM = {
    {Ab3, 500, 0},   {Db4, 1000, 0}, {G3, 750, 0},   {Gb3, 250, 0},
    {G3, 1000, 0},   {F3, 500, 0},   {Db3, 500, 0},  {Ab3, 1000, 0},
    {G4, 750, 0},    {F4, 250, 0},   {Db4, 1375, 125}, {Db4, 500, 0},
    {F4, 1000, 0},   {C4, 625, 125}, {C4, 250, 0},   {C4, 1000, 0},
    {Ab3, 500, 0},   {Gb3, 500, 0},  {G3, 500, 0},   {Ab3, 500, 0},
    {Ab3, 750, 0},   {C4, 250, 0},   {Ab3, 1500, 0}, {Ab3, 500, 0},
    {Ab3, 500, 0},   {D4, 500, 0},   {F4, 500, 0},   {F4, 500, 0},
    {F4, 500, 0},    {D4, 500, 0},   {Ab3, 750, 0},  {Ab3, 250, 0},
    {F4, 750, 0},    {E4, 250, 0},   {F4, 500, 0},   {G4, 500, 0},
    {Gb4, 500, 0},   {F4, 500, 0},   {D4, 500, 0},   {C4, 500, 0},
    {Ab3, 500, 375}, {C4, 125, 0},   {Ab3, 250, 0},  {Gb3, 250, 0},
    {G3, 250, 0},    {Ab3, 250, 0},  {G4, 1000, 0},  {Db4, 750, 0},
    {D4, 250, 0},    {C4, 1000, 0},  {F4, 750, 0},   {Db4, 250, 0},
    {D4, 1000, 0},   {G4, 750, 0},   {F4, 250, 0},   {Db4, 500, 0},
};

constexpr size_t MELODY_LENGTH = sizeof(melody) / sizeof(melody[0]);

inline Note readMelodyNote(size_t index) {
  Note note;
  memcpy_P(&note, &melody[index], sizeof(note));
  return note;
}
