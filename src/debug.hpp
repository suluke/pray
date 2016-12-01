#pragma once

#include <iostream>

#define ASSERT(exp) do { if(!(exp)) std::cerr << "Assertion failed: " << #exp " at(" __FILE__ ":" << __LINE__ << ")\n"; } while(false);
