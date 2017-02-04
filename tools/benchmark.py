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
import numpy
from operator import add

class Pray(object):
  def __init__(self, pray):
    self.pray = pray
    self.infos = argparse.Namespace()
    self.infos.complete = False
    self.infos.config = dict()

  def _digester(self):
    while True:
      info = yield
      if re.match(r'^#+ Configuration: #+$', info):
        self.infos.config['WITH_CONFDUMP'] = True
        info = yield
        while True:
          if not re.match(r'^#+$', info):
            matches = re.search(r'([A-Z_]+):\s+(.*)$', info)
            option = matches.group(1)
            state = matches.group(2)
            try:
              self.infos.config[option] = {'ON': True, 'OFF': False}[state]
            except KeyError:
              self.infos.config[option] = state
            info = yield
          else:
            break
      elif 'Warning: This is a Debug build and might be very slow!' in info:
        self.infos.config['DEBUG'] = True
      elif re.search(r'Preprocess Time: ([0-9]+)ms', info):
        self.infos.config['WITH_TIMING'] = True
      else:
        # We have gathered all info we could from the first few lines.
        self.infos.complete = True
        self.checkConfiguration()
        break
    while True:
      info = yield

  def run(self, scene, outfile, stop_on_infos_complete = False):
    proc = subprocess.Popen([self.pray, scene, outfile], stdout=subprocess.PIPE, bufsize=1)
    program_out = ''
    if not self.infos.complete:
      lines = []
      digest = self._digester()
      digest.send(None)
      for line in iter(proc.stdout.readline, b''):
        digest.send(line)
        lines.append(line)
        if self.infos.complete and stop_on_infos_complete:
          proc.terminate()
      proc.stdout.close()
      proc.wait()
      self.infos.complete = True
      program_out = '\n'.join(lines)
    else:
      program_out = proc.communicate()
    return program_out

  def parsePreprocessTime(self, program_out):
    if self.getConfiguration().get('WITH_TIMING'):
      for line in StringIO.StringIO(program_out):
        match = re.search(r'Preprocess Time: ([0-9]+)ms', line)
        if match:
          return int(match.group(1))
      raise ValueError("Did not find preprocess time in program output although it should be there")
    return 0
    
  def parseRenderTime(self, program_out):
    if self.getConfiguration().get('WITH_TIMING'):
      for line in StringIO.StringIO(program_out):
        match = re.search(r'Render Time: ([0-9]+)ms', line)
        if match:
          return int(match.group(1))
      raise ValueError("Did not find render time in program output although it should be there")
    return 0

  def checkConfiguration(self):
    config = self.getConfiguration()
    if config.get('DEBUG'):
      logging.warning('Pray is compiled in debug mode. Benchmark results will not be representative.')
    if not config.get('WITH_CONFDUMP'):
      logging.warning('WITH_CONFDUMP not enabled. Cannot guarantee that all information is available to create a meaningful report.')
    if not config.get('WITH_TIMING'):
      logging.warning('WITH_TIMING not enabled. Benchmark results will not be usable at all.')
    if not config.get('DISABLE_OUTPUT'):
      logging.warning('DISABLE_OUTPUT is recommended for faster execution but not enabled')

  def getConfiguration(self):
    self.assertInfosComplete()
    return self.infos.config

  def assertInfosComplete(self):
    if not self.infos.complete:
      self.run(
        os.path.abspath(os.path.join(os.path.dirname(__file__), '../examples/cube/cube.json')),
        os.devnull, True)

class Report(object):
  def __init__(self, scenefile, preprocess_times, render_times):
    self.scene = scenefile
    self.preprocess_times = preprocess_times
    self.render_times = render_times
    self.total_times = [a + b for (a, b) in zip(preprocess_times, render_times)]
    self.avg_preprocess_time = sum(preprocess_times) / len(preprocess_times)
    self.stddev_preprocess_time = numpy.std(preprocess_times)
    self.avg_render_time = sum(render_times) / len(render_times)
    self.stddev_render_time = numpy.std(render_times)
    self.avg_total_time = (self.avg_preprocess_time + self.avg_render_time)
    self.stddev_total_time = numpy.std(map(add, preprocess_times, render_times))
  
  def __json__(self):
    return self.__dict__

  def __str__(self):
    return ("""%s:
  preprocess avg: %d (%f)
  render avg:     %d (%f)
  total avg:      %d (%f)
    """ % (self.scene, self.avg_preprocess_time, self.stddev_preprocess_time, self.avg_render_time, self.stddev_render_time, self.avg_total_time, self.stddev_total_time))

def run_benchmarks(options):
  pray = Pray(options.pray)
  reports = []
  try:
    for scene in options.scene:
      times = []
      for i in range(0, options.iterations):
        program_out = pray.run(scene, os.devnull)
        preprocess_time = pray.parsePreprocessTime(program_out)
        render_time = pray.parseRenderTime(program_out)
        times.append((preprocess_time, render_time))
      report = Report(scene, *zip(*times)) # hardly readable, but zip(*l) UNzips :D
      print(str(report))
      reports.append(report)
  except KeyboardInterrupt:
    logging.info("Received keyboard interrupt. Publish reports and exit.")

  if options.report:
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
  parser.add_argument('--report', '-R', dest='report', action='store_true')
  parser.add_argument('--no-report',    dest='report', action='store_false')
  parser.set_defaults(report=False)
  parser.add_argument("--verbose",
                      "-v", default=0, action="count",
                      help="Set the logging verbosity")
  options = parser.parse_args(args)
  options.pray_dir = os.path.dirname(os.path.abspath(options.pray))
  if options.iterations < 1:
    parser.error("Minimum number of iterations is 1")

  logging.basicConfig(level=logging.INFO - options.verbose * 10, format='%(levelname)s: %(message)s')
  run_benchmarks(options);

if __name__ == '__main__':
    exit(script_main(sys.argv[1:]))
