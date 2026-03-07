#!/usr/bin/env python


__doc__ = """
Collection of utilities to ease interaction with ROOT.

Unsurprisingly, this module requires ROOT.
"""

__all__ = [
  "splitROOTpath", "createROOTpath", "getROOTclass",
  "activateDirectory",
  "ROOT",
  ]

import logging

Logger = logging.getLogger(__name__)

################################################################################
###
### Try to save the command line arguments from unconsiderate ROOT behaviour
###
def ROOTloader():
  """
  The ROOT interpreter likes to peek at the command line arguments and interpret
  them its own way.
  For example, an option `-b` will be interpreted to set ROOT in batch mode.
  Likewise, if a `--help` argument is present on the command line, the
  interpreter will print the ROOT help message and, even worse, halt the script.
  This is clearly not very friendly to the script.
  
  The initialization of the interpreter happens lazily the first time anything
  is read out of `ROOT` module: `import ROOT` is not enough, but pretty much
  everything after that involving ROOT is (exception: `ROOT.gROOT` will not
  trigger the interpreter, but trying to get anything out of `ROOT.gROOT` will).
  That makes it complicate to control when that happens.
  
  This function triggers the interpreter initialization after having removed all
  command line options, and finally restores them (we use a context manager to
  show that we know Python). The loaded module is returned.
  
  This function is called as soon as this module is imported. It is important
  that this happens as early as possible, possibly as a replacement of ROOT
  import, as:
      
      from ROOTutils import ROOT
      
      from ROOTutils import *
      
      import ROOTutils
      import ROOT
      
  or equivalent.
  """
  import sys
  try:              alreadyLoaded = 'gInterpreter' in dir(ROOT)
  except NameError: alreadyLoaded = False
  if alreadyLoaded:
    Logger.warning(
      "ROOT module was loaded before ROOTutils.py: command line arguments may be garbled"
      )
    return sys.modules['ROOT']
  # if already loaded 
  
  class EmptyArgs:
    def __enter__(self):
      self.args = sys.argv
      Logger.debug("Saving command line: %s", self.args)
      sys.argv = sys.argv[0:1]
      Logger.debug(
       "Replaced command line %s with %s before loading ROOT module",
       self.args, sys.argv)
    def __exit__(self, exc_type, exc_value, traceback):
      sys.argv = self.args
      Logger.debug("Restored command line %s", sys.argv)
  # class EmptyArgs
  
  with EmptyArgs():
    import ROOT
    ROOT.gInterpreter # make sure that an interpreter is initialized
    return ROOT
  # with
  
# ROOTloader()

ROOT = ROOTloader() # import ROOT...
del ROOTloader

################################################################################
### Print vectors easily
def TVector2ToString(v):
  return "( %g, %g )" % (v.X(), v.Y())
def TVector3ToString(v):
  return "( %g, %g, %g )" % (v.X(), v.Y(), v.Z())
def TLorentzVectorToString(v):
  return "( %g, %g, %g; %g )" % (v.X(), v.Y(), v.Z(), v.T())

ROOT.TVector2.__str__ = TVector2ToString
ROOT.TVector3.__str__ = TVector3ToString
ROOT.TLorentzVector.__str__ = TLorentzVectorToString


################################################################################
###  File management
###  
def splitROOTpath(path):
  """
  Returns the specified path split into file path and ROOT directory path.
  
  The `path` is in the form:
  "/UNIX/path/to/file.root:internal/ROOT/directory/and/object".
  The returned value is a pair `(filePath, dirPath)`: in the example, that
  would be `("/UNIX/path/to/file.root", "internal/ROOT/directory/and/object")`.
  
  Note: for compatibility with some ROOT tradition, the separator ':' can be
  replaced by '/'
  """
  
  # this implementation is not robust, but I am in a hurry :-P
  try:
    filePath, ROOTpath = path.rsplit('.root')
  except ValueError:
    raise RuntimeError("Path '{}' does not include a ROOT file.".format(path))
  filePath += '.root'
  ROOTpath.lstrip(':')
  ROOTpath.lstrip('/')
  return filePath, ROOTpath
  
# splitROOTpath()


def createROOTpath(path, fileMode = "UPDATE"):
  """
  Creates a complete ROOT directory path.
  
  The `path` is in the form:
  "/UNIX/path/to/file.root:internal/ROOT/directory/structure".
  The ROOT file `/UNIX/path/to/file.root` will be created with the specified
  `fileMode` (path may be relative), then the `TDirectoryFile` hierarchy
  `internal/ROOT/directory/structure` will be created under it.
  The return value is a pair `(file, dir)`, where `file` is a open `TFile`
  for `/UNIX/path/to/file.root` and `dir` is the `TDirectory` object of
  `structure`.
  
  Remember to keep track of `file`, or else python may close it compromising
  `dir` as well.
  """
  
  filePath, ROOTpath = splitROOTpath(path)
  
  ROOTfile = ROOT.TFile(filePath, fileMode)
  if not ROOTfile.IsOpen():
    raise RuntimeError \
      ("Can't open ROOT file '{}' in '{}' mode".format(filePath, fileMode))
  
  # instead of using `TDirectory.mkdir()`, we do that manually
  ROOTpathElements = ROOTpath.split('/')
  ROOTdir = ROOTfile
  for ROOTdirName in ROOTpathElements:
    if not ROOTdirName: continue # empty name does nothing
    daughterDir = ROOTdir.GetDirectory(ROOTdirName)
    if not daughterDir:
      daughterDir = ROOTdir.CreateDirectory(ROOTdirName)
    if not daughterDir:
      raise RuntimeError("Can't access directory '{}' under '{}'".format
       (ROOTdirName, ROOTdir.GetPath()))
    ROOTdir = daughterDir
  # for
  
  return ROOTfile, ROOTdir
  
# createROOTpath()


class DirectoryChanger:
  """
  Object changing ROOT directory while on scope.
  
  The purpose is to make a ROOT directory current only as long as it is needed.
  The most typical uses of this objects include the automatic restoration of
  the previous directory as the object falls out of scope.
  Two methods are supported:
  1. function scope:
        
        def writeEverythingInto(dir, everything):
          dirChanger = ROOTutils.DirectoryChanger(dir)
          for item in everything: item.Write()
        # writeEverythingInto()
        
  2. local scope (equivalent to using `activateDirectory()`):
        
        with DirectoryChanger(dir):
          for item in everything: item.Write()
        # with
        
  
  """
  def __init__(self, newDir = None, saveDir = None):
    if saveDir: self.saveDir(saveDir)
    else:       self.saveCurrentDir()
    self.newDir = newDir
    self.changeDir()
  # __init__()
  
  def saveCurrentDir(self): self.saveDir(ROOT.gDirectory)
  
  def saveDir(self, ROOTdir): self.oldDir = ROOTdir
  
  def changeDir(self):
    if self.newDir: self.newDir.cd()
  
  def restoreDir(self):
    if self.oldDir: self.oldDir.cd()
  
  def forget(self): self.oldDir = None
  
  def __del__(self):
    self.restoreDir()
  
  def __enter__(self):
    self.changeDir()
  
  def __exit__(self, exc_type, exc_value, traceback):
    self.restoreDir()
    self.forget()
  
# DirectoryChanger()


def activateDirectory(ROOTdir):
  """
  Sets a directory with `DirectoryChanger`.
  
  Example:
        
        dir = outputFile.GetDirectory("plots")
        with activateDirectory(dir):
          for plot in plots: item.Write()
        # with
        
  """
  return DirectoryChanger(ROOTdir)


################################################################################
def getROOTclass(classPath):
  """Returns the object specified by `classPath` within ROOT module.
  
  Throws `AttributeError` if any object in the path is not available.
  
  Example: `getROOTclass('geo::GeometryCore')` returns `ROOT.geo.GeometryCore`.
  """
  classPath = classPath.replace('::', '.').lstrip('.')
  base = ROOT
  for objName in classPath.split('.'):
    base = getattr(base, objName)
  return base
# getROOTclass()


################################################################################
# this is not really specific to ROOT, but we often have ROOT file lists
def expandFileList(
 fileListPaths: "path (or paths) of the file list (or single file)",
 comment: "(default: '#') character used to introduce a comment" = '#',
 fileListSuffixes: "suffix of entries to recursively add file lists" = [],
 fileSuffixes: "suffix of entries never to be treated as file lists" = [ '.root' ],
 ) -> "a list of file names":
  """Returns a list of file names as found in the specified file lists.
  
  The `fileListPaths` paths are read as text files; in them, each line
  represents a full file path.
  Empty lines and lines starting with a comment character are ignored.
  Also if blanks and a comment character are found, the content of the line
  from the first of those blank characters on is ignored as part of a comment.
  If `comment` is `None`, this feature is disabled.
  
  If file list suffixes are specified, a line ending with any of those suffixes
  will be considered a file list itself, and recursively expanded.
  If file suffixes are specified, a line ending with any of those suffixes
  will be considered a file, and not expanded. That takes priority over the file
  list suffix.
  
  If any of the file lists in `fileListPaths` can't be read, an exception is
  raised.
  
  NOTE: because of the internal implementation, a `fileListPaths` that is a
    collection of file names that are all one character long may be mistaken
    for a string. The general solution is: do not name your file not file lists
    with a single-character name. It's most often a bad idea anyway.
  """
  
  expandAgain = lambda path: expandFileList(
    path, comment=comment,
    fileListSuffixes=fileListSuffixes, fileSuffixes=fileSuffixes,
    )
  
  # ----------------------------------------------------------------------------
  # Path or list?
  #
  # This looks very silly, but distinguishing a string and a list is not trivial
  # (even less so in Python); and we need to support both std::vector and list,
  # both str and std::string.
  # The elements of all these types are of Python type str.
  # So it is hereby decided that if they all are of length 1, then it's a string
  # otherwise it is a list. Yes, this fails in an obvious corner case.
  # Just don't call your file lists "a", "b" etc. Nor your files.
  # Nevertheless, some special cases are singled out and specifically addressed.
  
  isClearlyList = isinstance(fileListPaths, (ROOT.std.vector[ROOT.std.string], list, set))
  isClearlyPath = isinstance(fileListPaths, (ROOT.std.string, str))
  # we expand the argument to support single-pass generators; what happens?
  #  str              -> list[str] (one character each element)
  #  std::string      -> [str ->] list[str] (one character each element)
  #  Python iterable  -> list (usually?)
  #  Python generator -> list
  fileListList = list(fileListPaths)
  if isClearlyList or (any(len(p) > 1 for p in fileListList) and not isClearlyPath):
    Logger.debug("Input is a list of %d paths and will be expanded as such.",
      len(fileListList))
    return sum([ expandAgain(path) for path in fileListList ], [])
  # if the argument is a list
  
  # at this point we believe the original argument was a string (not a list,
  # not a generator) and we broke it into pieces into `fileListList`.
  # But `fileListPath` is still whole, so we will use it.
  
  # ----------------------------------------------------------------------------
  # Expand, at last
  #
  # from here on, it's only one file or file list
  fileListPath = fileListPaths
  hasSuffix = lambda s, suffixes: any(s.endswith(sx) for sx in suffixes)
  
  if hasSuffix(fileListPath, fileSuffixes):
    Logger.debug("'%s' was for sure a file, not a file list.", fileListPath)
    return [ fileListPath ]
  
  Logger.debug("Processing file list '%s'", fileListPath)
  with open(fileListPath, 'r') as fileList:
    
    l = []
    for iLine, line in enumerate(fileList):
      line = line.strip()
      if not line: continue
      if comment:
        if line.startswith(comment): continue
        
        words = line.split(comment)
        line = words[0]
        for left, right in zip(words[:-1], words[1:]):
          if left and left[-1].isspace():
            Logger.debug("Comment starting at line %d between '%s' and '%s'", iLine, left, right)
            break
          line += comment + right
        # for
        line = line.rstrip()
      # if comment
      
      if hasSuffix(line, fileListSuffixes):
        # NOTE in case of conflicting suffixes, a line that matches both
        # list and file suffixes is first treated as a list here, and recursive
        # expansion is attempted; but the recursive expansion will consider it
        # a file and then return it unexpanded.
        Logger.debug("Adding content of file list from line %d ('%s')", iLine, line)
        l.extend((extra:= expandAgain(line)))
        Logger.debug("%d entries collected under file list '%s'", len(extra), line)
      else:
        l.append(line)
        Logger.debug("Line %d added to the list", iLine)
      # for suffix... else
    # for line in list
    
  # with
  return l
# expandFileList()


################################################################################
