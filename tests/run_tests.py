#!/usr/bin/env python3

import os
import subprocess
import git
import argparse
from PIL import Image
import asyncio
import termcolor

remoteRepUrl = 'https://code.ipd.kit.edu/lboehm/pray.git';

class TestSuite:
  path = os.path.dirname(os.path.realpath(__file__));
  refRepPath = os.path.join(path, 'reference')
  refRepOutPath = os.path.join(refRepPath, 'out')
  devRepPath = os.path.join(path, '..')
  devRepOutPath = os.path.join(devRepPath, 'out')
  
  devRep = None
  refRep = None
  
  results = [];
  
  def __init__(self):
    # init shell mode for subprocess.run
    if os.sys.platform.startswith('win'):
      self.shellMode = True
    else:
      self.shellMode = False
      
    # create results directory for tests and delete former files
    if os.path.exists(os.path.join(self.path, 'results')):
      for file in os.listdir(os.path.join(self.path, 'results')):
        if file.endswith(".json"):
          os.remove(os.path.join(self.path, 'results', file))
    else:
      os.makedirs(os.path.join(self.path, 'results'))
      
      
    
  
  def provideReferenceRepository(self):
    print("preparing reference repository...")
    
    # init development repository
    #self.devRep = git.Repo.init(os.path.join(self.path, '..'))
    
    # init reference repository
    if os.path.exists(self.refRepPath):
      self.refRep = git.Repo.init(self.refRepPath)
    else:
      os.makedirs(self.refRepPath)
      self.refRep = git.Repo.clone_from(remoteRepUrl, self.refRepPath, branch='master')
    
    # checkout reference commit
    if not self.args.commit:
      self.refRep.head.reset(commit='HEAD', index=True, working_tree=True)
    else:
      self.refRep.head.reset(commit=self.args.commit, index=True, working_tree=True)
    
    # create out directory for build
    if not os.path.exists(self.refRepOutPath):
      os.makedirs(self.refRepOutPath)
    
    termcolor.cprint("Testing against {}".format(self.refRep.head.commit.name_rev), attrs=['bold'])
    
  
  def build(self, basePath):
    print("building {}...".format(basePath))
    outPath = os.path.join(basePath, 'out')
    
    # build reference version
    subprocess.run(args=['cmake', basePath], 
      shell=self.shellMode, cwd=outPath, check=True)
    subprocess.run(args=['make', '-j', '4', '-C', outPath], 
      shell=self.shellMode, cwd=outPath, check=True)
  

  def parseArgs(self):
    parser = argparse.ArgumentParser(description='Run tests and compare with reference commit. ')
    parser.add_argument('--commit', dest='commit', default=None, 
      help='reference commit (default: master HEAD)')

    self.args = parser.parse_args()
    
    
  def runTests(self, createImages=True, compareImages=True):
    for file in os.listdir(os.path.join(self.path, 'tests')):
      if file.endswith(".json"):
        self.runTestCase(file, createImages, compareImages)


  def evaluateTestCaseResult(self, file, devRes, refRes):
    # open images
    devImg = Image.open(devRes)
    refImg = Image.open(refRes)
    devImg = devImg.convert('RGB')
    refImg = refImg.convert('RGB')
    
    numDiffPixel = 0
    
    if (not devImg.height == refImg.height) or (not devImg.width == refImg.width):
       self.results.append([file, False, "dimensions of dev and reference image are different"])
    
    # iterate pixel and check if equal
    for i in range(devImg.height):
      for j in range(devImg.width):
        if not devImg.getpixel((j, i)) == refImg.getpixel((j, i)):
          numDiffPixel += 1
    
    if numDiffPixel > 0:
      self.results.append([file, False, "{} of {} pixels do not match".format(numDiffPixel, devImg.width * devImg.height)])
    else:
      self.results.append([file, True, ""])
    

  def runTestCase(self, file, createImages=True, compareImages=True):
    if createImages:
      # execute with development binary
      try:
        print("{}: creating image...".format(file))
        subprocess.run(args=[os.path.join(self.devRepOutPath, 'pray'), 
          os.path.join(self.path, 'tests', file), 
          os.path.join(self.path, 'results', file + '.dev.bmp')], 
          shell=self.shellMode, cwd=self.refRepOutPath, check=True)
      except subprocess.CalledProcessError as e:
        self.results.append([file, False, 'development executable failed (return value: {})'.format(e.returncode)])
        return
      
      # execute with reference binary
      try:
        print("{}: creating reference image...".format(file))
        subprocess.run(args=[os.path.join(self.refRepOutPath, 'pray'), 
          os.path.join(self.path, 'tests', file), 
          os.path.join(self.path, 'results', file + '.ref.bmp')], 
          shell=self.shellMode, cwd=self.refRepOutPath, check=True)
      except subprocess.CalledProcessError as e:
        self.results.append([file, False, 'reference executable failed (return value: {})'.format(e.returncode)])
        return
    
    if compareImages:
      # evaluate results
      print("{}: comparing images...".format(file))
      self.evaluateTestCaseResult(file, os.path.join(self.path, 'results', file + '.dev.bmp'), 
        os.path.join(self.path, 'results', file + '.ref.bmp'))
      
  
  def printResults(self):
    termcolor.cprint("--- RESULTS ---", attrs=['bold'])
    for t in self.results:
      if t[1]:
        print("[SUCCESS] {}".format(t[0]))
      else:
        termcolor.cprint("[FAILED] {}: {}".format(t[0], t[2]), 'red', attrs=['bold'])
  
  
  def run(self):
    self.parseArgs();
    self.provideReferenceRepository();
    self.build(self.devRepPath)
    self.build(self.refRepPath)
    self.runTests(createImages=False);
    self.printResults();


# main
testSuite = TestSuite();
testSuite.run();
