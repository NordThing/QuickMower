#2020-09-15
#@author Henrik Allberg 
#This is the Boustrophedon-algorithm for the coverage path planning for the automower
#Will write the skeleton as comments to then implement the code. This will be as a function that our main-program
#can call.

'''
decimal  decimal     distance
places   degrees    (in meters)
-------  ---------  -----------
  1      0.1000000  11,057.43      11 km
  2      0.0100000   1,105.74       1 km
  3      0.0010000     110.57
  4      0.0001000      11.06
  5      0.0000100       1.11
  6      0.0000010       0.11      11 cm
  7      0.0000001       0.01       1 cm

So one 33 cm is 
0.0000030 in latitude and longitude

Home is 
59.2714840,17.816909
Total length 11m 
00.0001000
59.2723840,17.816909
Every path should be 33 cm
00.0000030
It should be total
00.0001000/00.0000030=33.3 paths
'''


#Rover is 38 cm an cutting width should be lika 38. But to get some marginal we put at 33.
#Because one of the latitude is 11cm and then it goes fine with that
roboUnit = 33

#Start direction up
#Let one roboUnit = the diameter of the robot
#if collision detected then
  #Move backwards one roboUnit
  #if if count is odd then
    #turn 90 degrees clockwise
    #Move forwards one roboUnit
    #Turn 90 degrees clockwise
    #add 1 to count
  #else
    #turn 90 degrees counter clockwise
    #move forwards one roboUnit
    #turn 90 degrees clockwise
    #add 1 to count
  #end
#else
  #move forward
#end

