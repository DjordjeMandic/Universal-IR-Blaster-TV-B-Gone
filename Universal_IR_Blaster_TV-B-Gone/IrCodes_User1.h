#ifndef _IRCODES_USER1_H_
#define _IRCODES_USER1_H_

#include "main.h"
#include "WORLD_IR_CODES.h"
#include "Custom_Ir_Codes.h"

const IrCode* const User1Codes[] PROGMEM = {
  &code_FoxCode,
  &code_LGCode,
  &code_TeslaCode,
  &code_BlueberryCode,
  &code_VoxCode,
  &code_MtsSTBCode,
  &code_D3MINISTBCode,
  &code_DenverSTBCode,
  &code_AlphaCode
};

uint8_t num_User1Codes = NUM_ELEM(User1Codes);

#endif
