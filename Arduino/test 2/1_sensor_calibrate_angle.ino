

/*
uses only 1 sensor. use the 1 sensor, prefer middle, to take readingR and readingL
THIS ASSUMES the robot is positioned approximately centered to the grid and rotation is performed on the spot
min_tol is the equidistance from the normal, where the normal is a vertical line cutting THROUGH the sensor
given a right-angled triangle with sides a,o,h where a is the distance of the robot to a wall/obstacle in the first reading or initial orientation
then h is the distance taken by the sensor at the second reading
for a 1 degree min_tol, a = 10cm, then o is 10tan(1) = 0.17cm. 
for a 5 degree min_tol, a = 13.5cm, then o is 13.5tan(5) = 1.18cm.
since readings are a and h respectively, min_tol is the minimum distance of difference between a and h
for a= 13.5cm, 5 deg, h should be 13.7cm meaning min_tol should be 13.7 - 13.5 = 0.2cm
meaning if the robot is 0 degrees slanted from the wall and 1st reading from the wall is 13.5 cm, then a 10 degree slant reading should read 13.7cm
if the robot is 10 degrees slanted from the wall and 1st reading from the wall is 13.5cm, then a 10 degree slant reading will read 14.6cm
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