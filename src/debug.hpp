#ifndef PRAY_DEBUG_H
#define PRAY_DEBUG_H
#pragma once

#ifndef NDEBUG
#define DEBUG
#endif

#include <iostream>

#define ASSERT(exp) do { if(!(exp)) std::cerr << "Assertion failed: " << #exp " at(" __FILE__ ":" << __LINE__ << ")\n"; } while(false);

#endif
