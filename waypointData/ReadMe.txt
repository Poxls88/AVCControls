Based on data from waypointCapt_9-10.xlsx

After the GPS initialization it will take about 7000 loops of pos=ubl.GPSfetch
and about 7 (pos !=None) before GPS data comes in consistently.

if ( (abs(pos['lat']-latW) <= 0.000001) and (abs(pos['lon']-lonW) <= 0.000001) ):
Approaching to 6 figures should work so long as the current timeout stays at 100loops and hAcc stays at 10m
	actually maybe 150 loops is better

bearWP = bearWPsign%360 #removes the sign so counter clockwise 0<->360
#bearWP is changing by no more than 1.8 degrees after the vehicle has moved 3-ish meters perpendicular to its waypoint
	that being said, bearRel behaves most appropriately.

The question now is why doesn't it steer appropriately?
Maybe its declination?
Boulder's declination is 8.2 degrees more to the east than true north.
San Diego's declination is 11.7 degrees more to the east than true north.

Next time i test, make sure to better separate GPS output data. Also take a screenshot of the exact waypoint location.
Also, good idea to have gps and compass be separate processes.