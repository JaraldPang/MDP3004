

/*
uses only 1 sensor. use the 1 sensor, prefer middle, to take readingR and readingL
THIS ASSUMES the robot is positioned approximately centered to the grid and rotation is performed on the spot
min_tol is the equidistance from the normal, where the normal is a vertical line cutting THROUGH the sensor

when the robot would to take 2 readings, 1 left of its center and 1 right of its center
 - if the robot is aligned, the left and right reading ~ approximately the same
 - if the robot is slanted left, the left reading > right reading
 - if the robot is slanted right, the right reading > left reading
 - the more severe the slant, the greater the difference in reading

e.g.

if the robot is 0 degrees slanted from the wall and distance from the wall is 13.5 cm, then a 10 degree left and right slant reading should read 13.7cm
if the robot is 10 degrees right slanted from the wall and distance from the wall is 13.5cm, then a 10 degree right slant reading will read 14.6cm, while a 10 degree left slant reading would read 
13.2cm

to afford a degree of tolerance, we put the slant difference tolerance as 0.2; this is equivalent to 5 degree slant on either side
this is because if the robot is already slant e.g. slant 10 degrees
a 5 degree turn for the 1st reading and 5 degree turn for the right reading corresponds to slant 5 and slant 15
will return 13.5 and 13.9 respectively.   

*/

//need 2 readings of distance in cm from robot's initial position
set min_tol = 0.2
moveLeft:5 and take ReadingR or reading1
moveRight:10 and take ReadingL or reading2

if(abs(readingR - readingL) > min_tol)
    //robot is slanted towards right
    //how much to move?
    while(abs(readingR - readingL) > min_tol)
        turnRight:15 //robot facing more left
        take ReadingR
        turnLeft:10 //robot facing more Right
        take ReadingL
    //another idea, take T-2 reading.
else if(abs(readingL - readingR) > min_tol)
    //robot is slanted towards left
    while(abs(readingR - readingL) > min_tol)
        turnLeft:15 //robot facing more right 
        take ReadingR
        moveLeft:10 //robot facing more left
        take ReadingL


//can be converted to a angle checking method by returning true or false, try on tuesday
void calibrate_angle_1_sensor(int tpin)
{
	double readingR;
  	double distR;
  	double max_tol = 0.2; //max tol is min tol in my notes
  	
  	moveRight(5);
  	readingR = final_MedianRead(tpin);
  	moveLeft(10);
  	readingL = final_MedianRead(tpin);

  	//if the robot is aligned then the distance readings from both angles should be roughly the same
  	//otherwise if R > L, robot is slanted right
  	//performance or speed of calibration can be improved by comparing T and T-1 reading 
  	if(abs(readingR - readingL) > max_tol)
  	{
	    while(abs(readingR - readingL) > min_tol)
    	{
    	    moveLeft(15) //robot facing more left
        	readingL = final_MedianRead(tpin)
        	moveRight(10) //robot facing more Right
        	readingR = final_MedianRead(tpin)
        }
  	}
    //robot is slanted towards right
    //how much to move?
    //another idea, take T-2 reading.
	else if(abs(readingL - readingR) > min_tol)
    {
    //robot is slanted towards left
    	while(abs(readingR - readingL) > min_tol)
    	{
    	    moveRight(15) //robot facing more left
        	readingL = final_MedianRead(tpin)
        	moveLeft(10) //robot facing more Right
        	readingR = final_MedianRead(tpin)
        }
    }
}