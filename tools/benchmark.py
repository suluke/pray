#!/usr/bin/env python2
import argparse
import calendar
from datetime import datetime
import json
import logging
import os
import re
import StringIO
import subprocess
import sys
import tempfile

class Pray(object):
  def __init__(self, pray):
    self.pray = pray

  def render(self, scene, outfile):
    program_out = subprocess.check_output([self.pray, scene, outfile])
    if not hasattr(self, '_InformativeOutput'):
      self._InformativeOutput = program_out
    return program_out

  def getPreprocessTime(self, program_out):
    if self.getConfiguration()['WITH_TIMING']:
      for line in StringIO.StringIO(program_out):
        match = re.search(r'Preprocess Time: ([0-9]+)ms', line)
        if match:
          return int(match.group(1))
      raise ValueError("Did not find preprocess time in program output although it should be there")
    return 0
    
  def getRenderTime(self, program_out):
    if self.getConfiguration()['WITH_TIMING']:
      for line in StringIO.StringIO(program_out):
        match = re.search(r'Render Time: ([0-9]+)ms', line)
        if match:
          return int(match.group(1))
      raise ValueError("Did not find render time in program output although it should be there")
    return 0

  def isDebug(self):
    return 'Warning: This is a Debug build and might be very slow!' in self._getInformativeOutput()

  def hasConfDump(self):
    return '# Configuration: #' in self._getInformativeOutput().split('\n', 1)[0]

  def getConfiguration(self):
    config = dict(DEBUG=self.isDebug())
    if self.hasConfDump():
      for line in StringIO.StringIO(self._getInformativeOutput()):
        if re.match(r'^#*$', line):
          break
        matches = re.search(r'([A-Z_]+):\s+(ON|OFF)', line)
        if not matches:
          continue
        option = matches.group(1)
        state = matches.group(2)
        config[option] = (state == 'ON')
    return config

  def _getInformativeOutput(self):
    if not hasattr(self, '_InformativeOutput'):
      self.render(
        os.path.abspath(os.path.join(os.path.dirname(__file__), '../examples/cube/cube.json')),
        os.devnull)
    return self._InformativeOutput

class Report(object):
  def __init__(self, scenefile, preprocess_times, render_times):
    self.scene = scenefile
    self.preprocess_times = preprocess_times
    self.render_times = render_times
    self.total_times = [a + b for (a, b) in zip(preprocess_times, render_times)]
    self.avg_preprocess_time = sum(preprocess_times) / len(preprocess_times)
    self.avg_render_time = sum(render_times) / len(render_times)
    self.avg_total_time = (self.avg_preprocess_time + self.avg_render_time)
  
  def __json__(self):
    return self.__dict__

  def __str__(self):
    return ("""%s:
  preprocess avg: %d
  render avg:     %d
  total avg:      %d
    """ % (self.scene, self.avg_preprocess_time, self.avg_render_time, self.avg_total_time))

def run_benchmarks(options):
  pray = Pray(options.pray)
  reports = []
  _, outfile = tempfile.mkstemp(suffix='_pray.bmp')
  try:
    for scene in options.scene:
      times = []
      for i in range(0, options.iterations):
        program_out = pray.render(scene, outfile)
        preprocess_time = pray.getPreprocessTime(program_out)
        render_time = pray.getRenderTime(program_out)
        times.append((preprocess_time, render_time))
      report = Report(scene, *zip(*times)) # hardly readable, but zip(*l) UNzips :D
      print(str(report))
      reports.append(report)
  except KeyboardInterrupt:
    logging.info("Received keyboard interrupt. Publish reports and exit.")

  now = datetime.now()
  report = dict(date=str(now), reports=reports, pray=pray.getConfiguration())
  indent = 2
  with open('pray_benchmark-%s.json' % calendar.timegm(now.timetuple()), 'w') as F:
    json.dump(report, F, indent=indent, default=lambda o: o.__json__())

def script_main(args):
  parser = argparse.ArgumentParser('benchmark.py')
  parser.add_argument('scene',
                      nargs='+', type=str,
                      help='JSON scene files to be used for benchmarking')
  parser.add_argument('--iterations',
                      '-n',
                      type=int, default=10,
                      help='Number of times to run each benchmark')
  parser.add_argument('--pray',
                      default=os.path.abspath(os.path.join(os.path.dirname(__file__), '../out/pray')))
  options = parser.parse_args(args)
  options.pray_dir = os.path.dirname(os.path.abspath(options.pray))
  if options.iterations < 1:
    parser.error("Minimum number of iterations is 1")
  run_benchmarks(options);

if __name__ == '__main__':
    exit(script_main(sys.argv[1:]))
