# invoke SourceDir generated makefile for SeniorProjGPS.pem3
SeniorProjGPS.pem3: .libraries,SeniorProjGPS.pem3
.libraries,SeniorProjGPS.pem3: package/cfg/SeniorProjGPS_pem3.xdl
	$(MAKE) -f C:\Users\eolson\WORKSP~3\SeniorProjRouter/src/makefile.libs

clean::
	$(MAKE) -f C:\Users\eolson\WORKSP~3\SeniorProjRouter/src/makefile.libs clean

