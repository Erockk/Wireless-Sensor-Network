# invoke SourceDir generated makefile for SeniorProj.pem3
SeniorProj.pem3: .libraries,SeniorProj.pem3
.libraries,SeniorProj.pem3: package/cfg/SeniorProj_pem3.xdl
	$(MAKE) -f C:\Users\eolson\WORKSP~3\SeniorProjCoordinator/src/makefile.libs

clean::
	$(MAKE) -f C:\Users\eolson\WORKSP~3\SeniorProjCoordinator/src/makefile.libs clean

