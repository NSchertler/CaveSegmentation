# Cave Segmentation

This repository contains the source code for the following research papers:
> **Chamber Recognition in Cave Data Sets** <br/>
> Nico Schertler, Manfred Buchroithner, Stefan Gumhold <br/>
> CGF 36/2 (Proc. Eurographics), May 2017 <br/>
and
> **Deterministically Defining Chambers in 3D-Scans of Caves ** <br/>
> Nico Schertler, Manfred Buchroithner, Donald McFarlane, Guy van Rentergem, Joyce Lundberg, Stefan Gumhold <br/>
> International Congress of Speleology, 2017 <br/>

Software Components
--
This repository does not only contain a single algorithm but an entire system that can be used for automatic cave segmentation, expert feedback gathering, and evaluation.
The following figure shows an overview of the individual components and their inter-dependencies:

[![Software Components][software]][software]

There are five executables (top row):

- **CaveSegmentationCommandLine** can be used to run cave segmentation on a 3D model with a specified set of parameters.
- **Evaluation** can be used to evaluate the algorithm and its parametrization with respect to expert feedback data sets.
- **CaveSegmentationGUI** is an interactive tool for cave segmentation.
- **ManualCaveSegmentationGUI** is a tool that allows experts to give feedback by painting a segmentation on a cave. This tool communicates with the **ManualCaveSegmentationService** that is used to distribute cave data and to gather feedback data sets.
- **PCPlot** is a parallel coordinates renderer for the result data from *Evaluation*.

The three dynamic libraries are (center):

- **MCFSkeleton** - a stand-alone adaptation of [Mean Curvature Flow Skeletons](https://github.com/ataiya/starlab-mcfskel).
- **CaveSegmentationLib** provides the algorithms for cave segmentation.
- **CaveSegmentationGUICore** provides common code for the GUI executables.

All other components are external dependencies, some of which are included in the repository (identifiable by the dotted background).

Compiling the Software
--

Compilation is only supported under Windows / Visual Studio. The project comes as a Visual Studio 2015 solution. Later versions should also work in principle.

Start by checking out the repository, including submodules:

    git clone https://github.com/NSchertler/CaveSegmentation --recursive
	
The project's solution will be in the repository root and is called `CaveSegmentation.sln`. As soon as all dependencies are installed, simply open this solution and compile the subprojects that you are interested in.

Dependencies
--

Consult the software components figure above to see what external dependencies you need for the applications that you are interested in. The following section will give instructions on installing these dependencies (there may be other ways as well).

All dependencies are injected via Visual Studio property sheets that reside in the repository root. In order to specify the location of some dependency, either define a system-wide environment variable (via Windows system settings) or create according user macros in these property sheets.

To add a user macro to a property sheet, open the solution in Visual studio, activate the property manager (via View -> Other Windows -> Property Manager). Then navigate to a project that has a specific dependency in the Property Manager. Expand the nodes under this project and open the property sheet. Then, under Common Properties -> User Macros, click on Add Macro.

There is one special project in the solution: **CopyDependenciesToOutput** copies all necessary binaries of the dependencies to the output directory.

### Boost

This section describes how to fulfill the Boost dependency via the pre-built binaries installer. The latest version that has been tested is *1.65.0*.

Download the x64 binaries of the current version of Boost from https://sourceforge.net/projects/boost/files/boost-binaries/.
The file name is usually `boost_[version]_msvc-14.0-64.exe`.

Run the installer.

Set up the following variable (either as an environment variable or as a user macro in `Boost.props`):
- `BOOST_ROOT` should point to the install directory.

### CGAL

This section describes how to fulfill the CGAL dependency. The latest version that has been tested is *4.10*. This approach requires that Boost and CMake are already installed on the system.

Download the latest installer from https://github.com/CGAL/cgal/releases.
The file name us usually `CGAL-[version]-Setup.exe`.

Run the installer. On the configuration page, choose x64 install.

Use CMake to generate a Visual Studio solution for the installed sources. If you have not set up `BOOST_ROOT` as an environment variable, you need to provide this variable to CMake.
Specify a reasonable `CMAKE_INSTALL_PREFIX` (the location where you want the libraries, binaries, and header files to be installed) as well as `CGAL_INSTALL_DOC_DIR` and `CGAL_INSTALL_MAN_DIR`).

Open the generated Visual Studio solution and build the `INSTALL` project.

Set up the following variables (either as environment variables or as user macros in `CGAL.props`):
- `CGAL_INSTALL_DIR` should point to the specified `CMAKE_INSTALL_PREFIX`.
- `CGAL_VERSION` should define the installed version (e.g. `4.10`).

### SuiteSparse

This section describes how to fulfill the SuiteSparse dependency. The latest version that has been tested is *4.5.1*. This approach requires that CMake is already installed on the system.

Check out the CMake wrapper from  https://github.com/jlblancoc/suitesparse-metis-for-windows

Use CMake to generate a Visual Studio solution for the sources from the repository. Specify a reasonable `SUITESPARSE_INSTALL_DIR` and/or `CMAKE_INSTALL_PREFIX`.

Open the generated solution and build project `INSTALL`.

Set up the following variable (either as an environment variable or as a user macro in `SuiteSparseQHull.props`):
- `SUITESPARSE_INSTALL_DIR` should point to the specified `CMAKE_INSTALL_PREFIX`.

### Qt

This section describes how to fulfill the Qt dependency via the pre-built binaries installer. The latest version that has been tested is `5.9.1`.

Download the installer from https://info.qt.io/download-qt-for-application-development and install.

Set up the following variable (either as an environment variable or as a user macro in `QtGLM.props`):
- `QTDIR` should point to the binary directory of the Qt version you want to use (e.g. `C:\Qt\5.9.1\msvc2015_64`).

  [software]: doc/Dependencies.jpg