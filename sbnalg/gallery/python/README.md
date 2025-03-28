% SBN/LArSoft utilities for interface with Python
% Gianluca Petrillo (petrillo@slac.stanford.edu), Marco Del Tutto ()
% June 23, 2019, revised for v10 on March 18, 2025

This document describes the utilities provided in `sbnalg` to facilitate the
use of LArSoft and SBN-specific code developed for _art_/_gallery_ into
Python.

This kind of documents tend to become outdated very quickly, so the reader is
invited, when in doubt, to refer to the code itself. Most of the utilities that
are designed to be part of the interface to users have some terse documentation
in the form of Python documentation strings.


Format of this document
------------------------

This file (`README.md`) is written in a dialect of [Markdown] format that is
compatible with [`pandoc`][pandoc]. The format is designed to be readable as
plain text: in that case, the reader will find all the links to external
resources at the end of the document.
The program `pandoc` allows to render this file in other formats. For example,
to render it into Portable Document Format:
    
    pandoc --toc -o README.pdf README.md
    
and to render it into HyperText Transfer Protocol format:
    
    pandoc --toc -o README.html README.md
    


Introduction to the interface to Python
========================================

LArSoft and SBN code, as well as _gallery_ code, rely on ROOT to interface
with Python (ROOT internally utilises relies on cppyy library).
Despite some relevant limitations, and despite what we are all used to think
about ROOT, its interface with Python is able to expose into Python a
great deal of the C++ code, including some more abstruse constructs that have
no equivalent in Python.

ROOT can import C++ objects, functions, namespaces and variables from C++
compiled code, and it places it under `ROOT` python module. Therefore, the
object `geo::Point_t` of `larcoreobj` is imported as `ROOT.geo.Point_t` and
the whole namespace `geo` is available as `ROOT.geo`. Note that for template
classes cppyy invents a special pattern, where the template class name, e.g.
`ROOT.lar.sparse_vector`, corresponds to `lar::sparse_vector` (which is almost
never used in C++ code), while the actual class name, e.g.
`lar::sparse_vector<int>`, is accessed as `ROOT.lar.sparse_vector[int]`.

ROOT needs information from two sources: the interface (e.g. the definition of
`geo::GeometryCore`) is learned from a C++ header file, while the actual code
(e.g. what to do when `geo::GeometryCore::Iterate` is called) is learned
from a compiled library. The fact that the building system of _art_/_gallery_
(including `cet_build_tools`, UPS, MRB) follows some conventions helps to
automate the discovery of all the needed components, but this is not fool-proof.


Components of the Python support libraries
===========================================

The support libraries provided here are organized according to the software they
interact with:

* `cppUtils.py` facilitates the management of C++ source code interacting with
  ROOT; most noticeably it includes a global `SourceCode` object tracking all
  the loaded code.
* `ROOTutils.py` simplifies some common tasks in handling ROOT objects, like
  analyzing a ROOT object path and printing standard ROOT vectors.
* `galleryUtils.py` simplifies some common tasks like creating a data product
  handle, loading a FHiCL configuration file, keep track of service providers,
  and more.
* `LArSoftUtils.py` simplifies the creation of LArSoft service providers, and
  some special support for the geometry services.

SBN experiment code is expected to build upon these utilities to provide an
experiment-aware convenient infrastructure.

In the following sections, details are provided on the utilities from these
libraries that are expected to be most commonly used, starting from the highest
level ones.


`LArSoftUtils.py`: one-stop shop for LArSoft/_gallery_ scripts
---------------------------------------------------------------

The module `LArSoftUtils.py` provides most of the facilities that are typically
needed for a stand-alone LArSoft script or a _gallery_ script.
Most of the utilities are actually exposed directly from `galleryUtils.py`.

* source code management: exposes `galleryUtils.SourceCode` and
  `galleryUtils.readHeader`
* configuration helpers: exposes `galleryUtils.findFHiCL`,
  `galleryUtils.ConfigurationClass`, `galleryUtils.ConfigurationHelper` and
  `galleryUtils.loadConfiguration`
* data product support: exposes `galleryUtils.make_getValidHandle`
* _art_ event processing support
  exposes `galleryUtils.makeFileList`, `galleryUtils.forEach` and
  `galleryUtils.eventLoop`
* service providers:
  exposes `galleryUtils.ServiceRegistryClass` and
  `galleryUtils.startMessageFacility`. It also provides:
    
  `loadSimpleService`
  
  :   a simple service is a service whose provider that can be initialized
      in a standardised way. LArSoft already uses some protocol to initialize
      services designed for simple unit tests not pulling _art_ in.
      If a service supports this type of setup (by specialization of
      `testing::ProviderSetupClass` template), then 'loadSimpleService' should
      be able to initialize it.
  
  `loadGeometry`, `loadWireReadout`, `loadAuxDetGeometry`
  
  :   these services are not simple, and their initialization involves a number
      of dynamically loaded objects and libraries. Because of this, a special
      initialization function is provided for them. However, these functions
      will hardly work without some additional, experiment-specific support.


`galleryUtils.py`: helpers for data product and event access
-------------------------------------------------------------

The module `galleryUtils.py` providers helper functions to access _art_ data
products, manage FHiCL configuration and service providers. It also exposes
utilities for source code management:

* source code management: exposes `cppUtils.readHeader` and
  `cppUtils.SourceCode`
* data product access:
  
  `make_getValidHandle`
  
  : getting a data product handle via Python requires some obscure preliminary
    steps, involving template member functions. This functions takes the name
    of the data type in the data product and performs those steps. After this,
    it will be possible to call `gallery.Event.getValidHandle` to read the data
    products from the events. Example:
    `galleryUtils.make_getValidHandle('std::vector<raw::RawDigit>')`.
    However, with the most recent cppyy and ROOT this is hardly necessary,
    and usually a `event.getValidHandle[ROOT.std.vector[ROOT.raw.Whatever]]()`
    call is enough.
  
* _gallery_ event loop:

  `makeFileList`
  
  :   `gallery.Event` constructor requires a file list as argument, in the form
      of `std::vector<std::string>`; this function creates such an object
      starting from a sequence of Python strings.
    
  
  `forEach`
  
  :   an event loop helper for _gallery_, allowing the use of a `gallery.Event`
      in a `for` loop. The loop may look like:
      
      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.python}
      for iEvent, event in enumerate(galleryUtils.forEach(event)):
        ...
      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
  
  `eventLoop`
  
  :   a complete event loop, initialized with input file list, a function to
      call on each event, and some options (number of events to skip and to
      process). Useful if you are in a hurry.
      
      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.python}
      import galleryUtils
      from ROOTutils import ROOT
      
      def countChannels(iEvent, event):
        rawDigits = event.getProduct[ROOT.std.vector[ROOT.raw.RawDigit]]('daq')
        print(f"Event {iEvent} has {len(rawDigits)} channels")
      # countChannels()
      
      galleryUtils.eventLoop('detsim.root', countChannels)
      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
  `EventIterator`
  
  :   turns a `gallery.Event` object into an iterable, allowing the use in
      generators and so on. Ultimately similar to `forEach()`, although the
      implementation is very different:
      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.python}
      for iEvent, event in enumerate(galleryUtils.EventIterator(event)):
        ...
      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      
  
* configuration management:
  
  `findFHiCL`
  
  :   returns the full path of a single FHiCL file. The algorithms used is not
      (necessarily) the same as in _art_: the file is searched in the current
      directory and then in all directories specified in `FHICL_FILE_PATH`
      environment variable.
    
  `loadConfiguration`
  
  :   parses a FHiCL configuration file and returns a `fhicl::ParameterSet`
      object.

  `ConfigurationHelper`
  
  :   a simple object wrapping a `fhicl::ParameterSet` object and mimicking its
      `get` and `has` methods. The `get()` function (as well as a direct call to
      the object itself) allows to specify either a class type or a default
      value (in which case the class type is deduced from it), or both.
      Example:
      
      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.python}
      pset = galleryUtils.loadConfiguration('parameters.fcl')
      config = galleryUtils.ConfigurationHelper(pset)
      threshold = config('threshold', 0.0)
      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
  `ConfigurationClass`
  
  :   an object that reads a job configuration FHiCL file and allows for quick
      access to service and module configuration. Example:
      
      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.python}
      config = galleryUtils.ConfigurationClass('standard_g4_sbnd.fcl')
      LArG4Parameters = config.service('LArG4Parameters')
      LArG4 = config.producer('largeant')
      Output = config.paramsFor('outputs.out1')
      ProcessName = config.config.get('std::string')('process_name')
      
      print "Process name: '%s'" % ProcessName
      print "LArG4 configuration:"
      print LArG4.to_indented_string()
      print "LArG4 parameters:"
      print LArG4Parameters.to_indented_string()
      print "Output:"
      print Output.to_indented_string()
      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
* service providers:

  `ServiceRegistryClass`
  
  :   an object keeping track of both configuration and service providers.
      Services are referred to by the name of their configuration
      (e.g. `Geometry`, `LArPropertiesService`).
      It is initialized with a FHiCL configuration file, and provides:
      
      `config`
      
      :   configuration of the specified service
      
      `register`
      
      :   registers (or replaces) an object in the service registry
      
      `has`
      
      :   whether the specified service is registered
      
      `registeredServiceNames`
      
      :   list of names of all registered services
      
      `get`, or direct call
      
      :   returns the object registered with the specified name
      
      `create`
      
      :   constructs and registers an object. The configuration is the one
          returned by `config()`.
          Example:
          
          ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.python}
          from galleryUtils import SourceCode, ServiceRegistryClass
          from ROOTutils import ROOT
          
          SourceCode.loadHeaderFromUPS('lardataalg/DetectorInfo/LArPropertiesStandard.h')
          SourceCode.loadLibrary('lardataalg_DetectorInfo')
          
          ServiceRegistry = ServiceRegistryClass('standard_g4_sbnd.fcl')
          larProp = ServiceRegistry.create \
            ('LArPropertiesService', ROOT.detinfo.LArPropertiesStandard)
          print larProp.RadiationLength()
          ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
          
          Note that this is equivalent to using `SBNDservices.py`:
          
          ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.python}
          from SBNDservices import ServiceManager
          ServiceManager.setConfiguration('standard_g4_sbnd.fcl')
          larProp = ServiceManager('LArProperties')
          print larProp.RadiationLength()
          ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
          
          which also ensures that services required by `LArProperties` are
          loaded.
      
  
  `startMessageFacility`
  
  :   configures the `messages` service, typically used by LArSoft algorithms
      and service providers, as well as by _gallery_, to emit messages on
      screen


> Note: for `galleryUtils.py` to be fully functional, `gallery` UPS product must
> be set up. That is not part of the usual MRB set up.



`cppUtils.py`: helpers for source management
---------------------------------------------

The `cppUtils.py` module provides a few utilities to manage the loading of C++
software into Python via ROOT.
This is normally achieved by sending the ROOT interpreter some commands that it
turns to its C++ compiler (based on Clang). ROOT can then bind the compiled
objects, types, functions etc. to the `ROOT` Python namespace.
These utilities simplify the procedure relying on the assumption that the code
is following some recommended naming practices and that it is set up in a way
compatible with Fermilab UPS.

> Although the examples in this section use a LArSoft data product
> (`raw::RawDigit`), this machinery is usually _not necessary for data products_
> because ROOT typically knows how to find those. For example, all is needed in
> a properly configured environment is: `import ROOT; d = ROOT.raw.RawDigit` to
> get a `raw::RawDigit` object. These utilities are instead useful when looking
> for algorithms and service providers.


`SourceCode`

:   this object manages the loading of C++ code into ROOT. Only the functions
    expected to be commonly used are documented here.
    
    `addIncPath`, `addIncPaths`
    
    :   adds the specified path(s) to the list of directories ROOT looks for C++
        headers into; it understands environment variables (e.g.
        `${LARDATAOBJ_INC}/RawData`; see `os.path.expandvars`). It won't add a
        path if it's already present, unless `force`d to.
        

    `addIncPathEnv`, `addIncPathEnvs`
    
    :   adds the path in the specified environment variable(s) to the list of
        directories ROOT looks for C++ headers into; example:
        `addIncPathEnv('LARDATAOBJ_INC')`.

    `loadHeader`, `loadHeaderFromUPS` 
    
    :   instructs ROOT to read and compile the specified C++ header.
        `loadHeader` looks for the header, which is specified as a relative
         path, in the paths registered with `addIncPath` family of functions.
        In addition, `loadHeaderFromUPS()` also tries to look in the UPS product
        the header belongs to, whose name is deduced from the first element of
        the header path.
        Both return the full path of the specified header.
        Example:
        `SourceCode.loadHeader('lardataobj/RawData/RawDigit.h')` or,
        equivalent,
        `SourceCode.loadHeaderFromUPS('lardataobj/RawData/RawDigit.h')`.
        For the latter, there is no need to explain `SourceCode` where to find
        the header, since it will automatically consider `$LARDATAOBJ_INC`.
    
    `loadLibrary`
    
    :   instructs ROOT to load the specified compiled library into memory.
        Example: `SourceCode.loadLibrary('lardataobj_RawData')` (library name
        is expanded adding `lib` prefix and a proper suffix).
    
`readHeader`

:   makes ROOT compile the specified header; this is a necessary step to be able
    to use C++ code from precompiled libraries. Most often, using this function
    directly is not necessary if the `SourceCode` object is used to manage the
    code.




`ROOTutils.py`: common ROOT operations
---------------------------------------------

The module `ROOTutils.py` includes helpers for some often-needed tasks related
to ROOT. It also is a "replacement" for `ROOT` module: ROOT can be imported by:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.python}
from ROOTutils import ROOT
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The only reason to do that instead of plain `import ROOT` is that `ROOTutils.py`
attempts to load ROOT module in a way that does not interfere with command line
parsing (the most common symptom is ROOT intercepting command line options like
`--help` or `-b`).

Finally, the module also attaches string conversion functions to some commonly
used ROOT objects, so that they can be printed more easily:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.python}
v = ROOT.TVector3(1.5, 0.0, -2.0)
print v
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

will show `Name: TVector3 Title: A 3D physics vector` with plain `ROOT` module,
and `( 1.5, 0, -2 )` if `ROOTutils` is loaded.

Other provided utilities are:

`splitROOTpath`

:   returns the specified path split into file path and ROOT directory path

`createROOTpath`

:   creates a complete ROOT directory path, including a new file if needed,
    and all `TDirectoryFile` objects in the path.

`getROOTclass`

:   returns the Python class for the specified class: for example,
    `ROOTutils.getROOTclass('geo::Point_t')` returns `ROOT.geo.Point_t`.
    The advantage of this function is that it works even if intermediate objects
    haven't been loaded yet. ROOT binds names to Python in a lazy way, only when
    they are explicitly requested. For example, immediately after loading
    `GeometryCore.h`, namespace `geo` is not bound to `ROOT.geo`. Asking members
    of namespace `geo` ends up being trouble in this situation.

`activateDirectory`

:   switches to a ROOT directory, and back when done. This is used as a context
    manager:
    
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.python}
    dir = outputFile.GetDirectory("plots")
    with activateDirectory(dir):
      for plot in plots: item.Write()
    # with
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    will switch to `plots` ROOT directory, write all plots into it and then,
    at the end of the `with` block, switch back to whatever ROOT directory was
    active before `activateDirectory` call. If an exception is thrown in that
    block, the active directory is restored as well.
    Note that the object acting as context manager,
    `ROOTutils.DirectoryChanger`, is also directly available if needed.


Execution environment
======================

This facility includes interactions between many diverse components: _gallery_,
ROOT, LArSoft, SBN code, Python... While it has been proven that this _can_
work, the setup of a working environment might be non-trivial.
The following recipes has proven to work on ICARUS SL7 GPVM servers:

* use a binary distribution, setting it up with `setup sbnalg ...`
* use a MRB area with all of the following:
    1. `sbnalg` checked out in the source directory, or explicitly set up;
      e.g. `( cd "$SOURCE_DIR" && mrb gitCheckout sbnalg ; )`
    2. all checked out code properly compiled _and installed_: `mrb install -j4`
    3. all local "products" set up: `mrbslp`

Both setup procedures should set up all is needed, including important parts
like:

* UPS environment variables, like `LARCOREALG_INC`, used to locate header files
  and compiled libraries
* ROOT machinery (`ROOTSYS` and more)
* `PYTHONPATH` environment variable, used to locate the Python modules described
  in this document

In addition, to use _gallery_, it must be explicitly set up. The easiest way to
get there is to set up `larsoftobj`:

* check which `larsoftobj` version is matching the LArSoft version in MRB
  working area in the [LArSoft release web page][LArRel]
* after the setup procedure described above, set up `larsoftobj` as well

Note, however, that a setup with only `sbnalg` will not have access to most of
the experiment services (it can be set up to use the "LArTPC detector" defaults
of LArSoft, which is similar to MicroBooNE detector).


Examples
=========

Reading data products
----------------------

An example session to read and print the energy of all muons simulated by
GEANT4:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.python}
import galleryUtils, ROOT

sampleEvents = galleryUtils.makeEvent("data.root")
LArG4tag = ROOT.art.InputTag("largeant")

for event in galleryUtils.forEach(sampleEvents):
  particles = event.getProduct[ROOT.std.vector[ROOT.simb.MCParticle]](LArG4tag)
  for particle in particles:
    if abs(particle.PdgCode()) != 13: continue
    print("{} at {} with energy {g} GeV".format(
      ("mu-" if particle.PdgCode() == 13 else "mu+"),
      particle.Position(),
      particle.E(),
      ))
  # for particles in event
# for all events
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Many utilities here are just shortcuts, and intended to be convenient ones:

* `makeEvent()` creates `gallery.Event` object (bypassing the creation of a
  `std::vector` object for the file list)
* `forEach()` makes the syntax of the event loop more Pythonesque;
* `particle.Position()` is a ROOT `TLorentzVector`, which is pretty-printed
  (`(x, y, z; t)`) thanks to the functions overridden in `ROOTutils`

For comparison, this is the same setup in a ROOT (C++) interactive session:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
#include "gallery/Event.h"
#include "nusimdata/SimulationBase/MCParticle.h"

gallery::Event sampleEvents({ "data.root" });
art::InputTag const LArG4tag { "largeant" };
for(; !sampleEvents.atEnd(); sampleEvents.next()) {
  auto const& particles
    = sampleEvents.getProduct<std::vector<simb::MCParticle>>(LArG4tag);
  for (simb::MCParticle const& particle: particles) {
    if (std::abs(particle.PdgCode()) != 13) continue;
    TLorentzVector const& pos = particle.Position();
    std::cout << ((particle.PdgCode() == 13)? "mu-": "mu+")
      << " at (" << pos.X() << ", " << pos.Y() << ", " << pos.Z() << "; "
      << pos.T() << ") with energy " << particle.E() << " GeV"
      << std::endl;
  } // for particles
}
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~




[Markdown]: https://daringfireball.net/projects/markdown
[pandoc]: https://pandoc.org/MANUAL.html
[LArRel]: https://larsoft.github.io/LArSoftWiki/releases/LArSoft_release_list
