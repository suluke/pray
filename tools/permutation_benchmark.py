#!/usr/bin/env python2
import benchmark
import itertools

REQUIRED = {
  'WITH_TIMING': True,
  'WITH_CONFDUMP': True,
  'DISABLE_OUTPUT': True,
  'WITH_DEBUG_TOOL': False,
}
OPTIONS = {
  'WITH_OMP': None,
  'WITH_SSE': None,
  'WITH_CUDA': None,
  'WITH_SUBSAMPLING': None,
  'WITH_BIH': {
    'WITH_BIH_SMART_TRAVERSION': {
      'WITH_BIH_SMART_TRAVERSION_ITERATIVE': None,
    },
  },
}

def permutation_gen(options):
  option_names = list(options.keys())
  for perm in itertools.product([True, False], repeat=len(option_names)):
    subgens = []
    enabled = []
    for i in range(0, len(option_names)):
      if perm[i]:
        opt = option_names[i]
        if options[opt]:
          subgens.append(permutation_gen(options[opt]))
        enabled.append(opt)
    if len(subgens) > 0:
      for subperm in itertools.product(*subgens):
        yield enabled + subperm[0]
    else:
      yield enabled

for perm in permutation_gen(OPTIONS):
  print perm
