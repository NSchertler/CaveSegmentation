# Cave Segmentation

This repository contains the source code for the following research papers:
> **Chamber Recognition in Cave Data Sets** <br/>
> Nico Schertler, Manfred Buchroithner, Stefan Gumhold <br/>
> CGF 36/2 (Proc. Eurographics), May 2017

and

> **Deterministically Defining Chambers in 3D-Scans of Caves** <br/>
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

Usage
--

The following tutorial will walk you through every software component. For this, the subfolder `/Data/SyntheticCave` contains an example cave (`model.off`). All other files in that folder are generated by one of the applications and are uploaded to allow starting the tutorial in the middle.

### CaveSegmentationGUI

[![Cave Segmentation GUI][gui]][gui]

The Cave Segmentation GUI can be used for interactive segmentation and parameter tweaking. Open `CaveSegmentationGUI.exe` to start.

The left pane contains means for data I/O and for steering the segmentation:

Let's start by loading the example cave. Click *Load OFF* and navigate to `/Data/SyntheticCave/model.off`. Opening the file will show you the bare cave model.

To control the view, use the left mouse button (rotation), the right mouse button (panning), and the mouse wheel (zoom).

For the segmentation, we need a curve skeleton. You can either load the pre-calculated skeleton by clicking *Load Skeleton* (this will load the skeleton immediately because there is a file with the name `model.skel` in the directory of the loaded model; otherwise you would be asked for a file). Alternatively, you can calculate the skeleton from scratch by clicking *Calculate Skeleton*. This may take a while depending on the parameters (especially the `w_velocity` parameter). Once calculated, you may want to save the skeleton.

After a skeleton is present, it will be shown in the 3D view as grey dots (the vertices) and green edges. When you hover over a vertex, the corresponding surface vertices will be visualized.

After that, we need the perceptible size, cave scale, etc. on the curve skeleton. For this, either load the pre-calculated distances (*Load Distances* which will load the default file `distances.bin`) or calculate new distances with the given *Exponent*. This exponent is the parameter `e` from the paper.
The distances need only be re-calculated when you change the exponent or the curve skeleton. All other parameters have no influence on the distances.

If you want to see what's happening for a specific skeleton vertex, click the vertex and choose *Calculate Distances for Selected Vertex*. This writes visualizations of the intermediate states during the line flow processes to `/Data/SyntheticCave/Output`.

Once the distances are there, a segmentation will be calculated automatically. You can modify the segmentation parameters with the sliders.

You can also look at the calculated measures along a path segment. For this, click on a start skeleton vertex. Then, hold shift and click on the end vertex to specify a path. You can shift-click more vertices to augment the path. The calculated data on this path will then be shown in the function plot (you can zoom and pan the plot and show/hide specific data sets). When you hover over a position in the plot, the corresponding location on the skeleton will be highlighted. The two probability functions will use the right y-axis. All other functions will use the left y-axis.

Finally, you can save the segmentation or the segmented mesh with the buttons on the very bottom of the settings pane.

### CaveSegmentationCommandLine

The same functionality of the interactive GUI can be used from the command line (more or less). The executable will print its options when you call it without any arguments. Here is an example call (which is called from `/x64/Release`):

    > .\CaveSegmentationCommandLine.exe -d ../../Data/SyntheticCave

    3154 direction samples.
    Model file: ../../Data/SyntheticCave/model.off
    Loading from cache instead of OFF...
    Loading skeleton from ../../Data/SyntheticCave/model.skel
    Calculating adjacency list...
    Finished correspondences.
    Loading distances from ../../Data/SyntheticCave/distances.bin
    Loading distances from file...
    Smoothing distances and deriving additional measures...
    Using cave scale kernel factor 10
    Using cave size kernel factor 0.2
    Using cave size derivative kernel factor 0.2
    Finding chambers...
    Using curvature tipping point 0.3
    Using direction tolerance 0.1
    Solving minimization problem (94 factors, 2 non-submodular, 94 variables) ...
    Generating output...
    Writing segmentation to ../../Data/SyntheticCave/output/segmentation.seg
    Saving skeleton with colored correspondences to ../../Data/SyntheticCave/output/Correspondences.obj
    Saving colored skeleton to ../../Data/SyntheticCave/output/Skeleton.obj
    Saving skeleton with cave size encoded as z-coordinates to ../../Data/SyntheticCave/output/SkeletonWithHeightCodedSize.obj
    Saving segmented model to ../../Data/SyntheticCave/output/segmentedMesh.off

The generated output will be in `/Data/SyntheticCave/output`.

### Manual Cave Segmentation

The manual cave segmentation subsystem is used to gather expert feedback. It consists of a server and a client.

Start the server by running the project `ManualCaveSegmentationService` from within Visual Studio. This should start IIS and install the service on it. When installed successfully, a web browser will open with the service's home page. The service uses a very simplistic GUI:

[![Manual Cave Segmentation Service][service]][service]

The first thing we need to do is upload a cave to the server. Only caves from the server will be available for the clients. To do so, we use the form on the service's home page.
Enter a cave name and the upload password (this is hard-coded in `Controllers/CaveController.cs - PostFormData()` to an empty string; this should be changed for real use).
Then choose a binary cave file and upload the cave. Such binary cave files can be created with the *CaveSegmentationGUI* after loading a cave (use the *Write BIN* button at the top). For the test cave, an according `model.bin` is also located in the test cave folder.
There will be no nice success or failure page because this form sends the data directly to the service. However, in case of failure, you will get a non-200 status code.
With this, the server is set up completely. Remember the address that opened in the browser (in this example, this is `http://localhost:3563`). You can also inspect the REST service's API documentation via the *API* link.

Now, we can open the `ManualCaveSegmentationGUI`. When you start the application for the first time, you will be asked for the server address. Specify the address that you remembered earlier (i.e. `http://localhost:3563`). This information (and all other local data) will be stored in `%USERPROFILE%\CaveSegmentation`.

After that, you can download the provided caves from the server. Click the *Download Caves From Server* button, check all caves that you want to download, and click *Download*.

[![Manual Cave Segmentation - Download Caves][download]][download]

Once you have downloaded the caves, you can open any of them with the *Load Cave* button. Camera control is a bit different than in the *CaveSegmentationGUI*: You need to hold down the Ctrl-key to control the camera.

Now it's time to paint the segmentation on the cave. Use the settings in the *Paint Segmentation* category and simply paint the appropriate regions of the cave. You can use the *Front View Cutoff* to reach complicated regions.

[![Manual Cave Segmentation - Paint Segmentation][paint]][paint]

Finally, upload the segmentation back to the server with the *Upload Current Segmentation* button. You can also save the segmentation to your computer and continue your work later.

The segmentation files can also be loaded into *CaveSegmentationGUI*.

### Evaluation

After experts have uploaded their segmentations, you can start evaluating the algorithm based on the manual segmentations. For this, you first need to download all segmentations for a specific cave. Open a browser and make the following REST request:

    http://localhost:3563/api/Caves/1/Segmentations/zip
This will download all segmentations of the cave with id `1` as a zip archive. Extract the archive to the subfolder `segmentations` of the directory that contains the 3D model. The example cave directory already contains such a directory with a single segmentation.

With this, you can run the *Evaluation.exe*. An example call can be found under `/Data/RunEvaluationOnSyntheticCave.bat`. The program will read the `config.txt` that lies in the calling directory. The `config.txt` specifies what parameters should be tested. 

An example call could look like this (called from the `/Data` folder):

    > "../x64/Release/Evaluation.exe" SyntheticCave

    Loading "SyntheticCave"..
    3,154 direction samples.
    Loading from cache instead of OFF...
    Scheduling algorithm for evaluation: Max
    Scheduling algorithm for evaluation: Smooth
    Evaluating a total of 62,208 samples.
    Status: 99.9 %, estimated time to finish: 15.4ms

    Best parameters for maximum minimal plausibility:
    Mean Plausibility: 100 %
    Min Plausibility: 100 %
    Distance Power: 1
    Scale Algorithm: Max
    Scale Kernel: 6
    Size Kernel: 0.1
    Size Derivative Kernel: 0.34
    Curvature Tipping Point: 0.6
    Direction Tolerance: 0.09

    Best parameters for maximum average plausibility:
    Mean Plausibility: 100 %
    Min Plausibility: 100 %
    Distance Power: 1
    Scale Algorithm: Max
    Scale Kernel: 6
    Size Kernel: 0.1
    Size Derivative Kernel: 0.34
    Curvature Tipping Point: 0.6
    Direction Tolerance: 0.09
	
The generated detailed results are also written to `result.bin` and `result.csv`. Those can be visualized with *PCPlot*.

### PCPlot

To visualize the evaluation result with a Parallel Coordinates rendering, simply start *PCPlot* and open the `result.bin`. The mapping from plausibility to intensity is hard-coded in `lines.vert` (`quality = ...`) and might need to be modified.

[![Parallel Coordinates Plot][pc]][pc]

  [software]: doc/Dependencies.jpg
  [gui]: doc/CaveSegmentationGUI.JPG
  [service]: doc/ManualCaveSegmentationService.JPG
  [download]: doc/ManualSegmentationDownload.JPG
  [paint]: doc/ManualSegmentationPaint.JPG
  [pc]: doc/PCPlot.JPG