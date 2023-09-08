# SoftRobots.Inverse plugin for SOFA

[![Information](https://img.shields.io/badge/info-on_website-purple.svg)](https://project.inria.fr/softrobot/)
[![Documentation](https://img.shields.io/badge/doc-on_website-blue.svg)](https://softrobotscomponents.readthedocs.io/en/latest/index.html)
[![Contact](https://img.shields.io/badge/contact-form-green.svg)](https://project.inria.fr/softrobot/contact/) 

This plugin contains inverse method for soft robotics and have a strong dependence to the [SoftRobots plugin](https://github.com/SofaDefrost/SoftRobots).

![](https://github.com/SofaDefrost/SoftRobots.Inverse/blob/main/docs/images/PluginImage.png)

## Documentation
You can find a complete documentation at this address: [SoftRobots Plugin for SOFA](https://project.inria.fr/softrobot/).   
You can also explore the [*examples/*](https://github.com/SofaDefrost/SoftRobots.Inverse/tree/main/examples) directory of this repository.

## Installation
This plugin is distributed in all [stable SOFA binaries](https://www.sofa-framework.org/download/). To build the plugin from sources, instructions can be found [here](https://project.inria.fr/softrobot/install-get-started-2/). 

## Usage

As any SOFA plugin, to use the SoftRobots.Inverse plugin, you should specify to load both libraries:

``` xml
<RequiredPlugin name="SoftRobots">
<RequiredPlugin name="SoftRobots.Inverse">
```

## Authors
 - Christian Duriez
 - Eulalie Coevoet
 - Yinoussa Adagolodjo

## License

This plugins is available under the AGPL-v3 license, granting a non-exclusive, worldwide, royalty-free license to use and exploit the associated patented method. For more information, see both files:
- LICENSE.AGPLV3.txt (AGPL-v3)
- LICENSE.AGPLV3.Clause11.txt (mention of the patent)

The open source software " SoftRobots.Inverse " implements the patented Inria method for performing the inverse in real time in order to control the robot: Inria patent entitled "Method for controlling of a deformable robot, associated module and computer program", filed in France on 8 February 2013 (priority date) publication number: FR3002047 - 2014-08-15 - WO2014122134A1; US2015360367A1; EP2954429A1; JP2016513021A.
  
Clause 11 of its open source license "AGPLv3" grants you a non-exclusive, worldwide, royalty-free license to use and exploit this patent when you use  " SoftRobots.Inverse  " software or any derivative software work (i.e., developed from the source of that software) to implement this patented method. 
 
