#!/bin/sh
# the next line restarts using wish \
exec wish "$0" "$@"

# CU Neuromechanics Lab (P.I. Alaa Ahmed)

# useful.tcl is a library of procedures that may be helpful when coding a new game

##############################

proc loadparameters {} {
	
	global ob mob main startmenu datamanagemenu loadparammenu graphicsmenu trialfieldmenu trialbasicmenu curlmenu divmenu jumpmenu vrotmenu

	set mob(experimentername) "Gary"
	#set mob(folderpath) "/home/alaa/DATA/Erik/EffortStraight"
	set systemTime [clock seconds]
	set mob(fileid) "test"
	set mob(testdate) [ clock format $systemTime -format %m%d%Y ]
	set mob(soundon) 1
	set mob(soundoff) 0
	set mob(savetclon) 0 
	set mob(savetcloff) 1 

	set mob(startsavestate) home
	set mob(stopsavestate) intertrial

	set mob(totaltrials) 400
	set mob(batchtrials) 5
	set mob(catchtrialsper) 0
	set mob(resttrials) 1000
	set mob(trainingdot) 0

	set mob(inputrobot) 1
	set mob(movingbackforces) none
	set mob(inputcop) 0
	set mob(extrarz) 0
	set mob(inputmouse) 0
	set mob(mvtoutonly) 0
	set mob(mvtoutback) 0
	set mob(switchtarhome) 1

	set mob(requiredtimeinhome) 500
	set mob(requiredtimeintarget) 500
	set mob(minmvttime) 1
	set mob(maxmvttime) 10000
	set mob(trialtimelimit) 1

	set mob(home_rad_cm) 1.1
	set mob(target_rad_cm) 1.4
	set mob(cursor_rad_cm) 0.4
	set mob(cursorfdbk) x_y
	set mob(home_x_cm) 0
	set mob(home_y_cm) 0
	set mob(scaling) 1
	
     set mob(targetdistances) "10 10 10 10"
	set mob(targetangles) "45 135 225 315"

	set mob(viapt_x_cm) -0
	set mob(viapt_y_cm) 0
	set mob(viapt_rad_cm) 0.1

	set mob(distributionmean) 0
	set mob(distributionstd) 0

	set mob(fieldcurl) 1
	set mob(fieldcurlposn) 0
	set mob(fielddiv) 0
	set mob(fieldjump) 0
	set mob(fieldvrot) 0

	set mob(curltrials) "1 300"
	set mob(curlgain) 0
	set mob(curlgainsd) 0
	set mob(dampconst) 0

	set mob(divtrials) ""	
	set mob(divgain) 0
	set mob(safetydist) 7
	set mob(deadzone) 1
	
	set mob(jumptrials) ""
	set mob(jumpdistances) ""
	set mob(jumptime) 0
	set mob(jumpcortime) 0
	set mob(jumpangles) ""

	set mob(vrottrials) ""
	set mob(vrotangles) ""

}
