/**
 * Description of the components can be found at:
 * https://partage.inria.fr/share/page/site/defrost-project-documentation/wiki-page
*
			*****************
			***REQUIREMENT***
			*****************

***************************************
**QPInverseProblemSolver dependencies**
***************************************

Dependency on qpOASES library

******************************
**LINUX, WINDOWS or MAC OS X**
******************************

1- The sources are distributed with the plugin in "extlibs/qpOASES_VERSION"
2- Depending on your system, in "extlibs/qpOASES_VERSION/make.mk" enable the right file:
	include ${TOP}/make_linux.mk
	#include ${TOP}/make_cygwin.mk
	#include ${TOP}/make_windows.mk
	#include ${TOP}/make_osx.mk
3- Using your terminal:
	cd extlibs/qpOASES_VERSION
	make




