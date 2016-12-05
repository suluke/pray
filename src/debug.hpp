#ifndef PRAY_DEBUG_H
#define PRAY_DEBUG_H
#pragma once

#include <iostream>

#define ASSERT(exp) do { if(!(exp)) std::cerr << "Assertion failed: " << #exp " at(" __FILE__ ":" << __LINE__ << ")\n"; } while(false);

#endif
