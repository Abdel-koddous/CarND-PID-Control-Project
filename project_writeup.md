# CarND PID Control Project Writeup

[image1]: ./videos/writeup_gif/Kp0.gif "Kp = 0"
[image2]: ./videos/writeup_gif/Kp1.gif "Kp = 1"
[image3]: ./videos/writeup_gif/Kp0_25.gif "Kp = 0.25"
[image4]: ./videos/writeup_gif/Kp0_25-Kd5.gif "Kp = 0.25 & Kd = 5"
[image5]: ./videos/writeup_gif/full_lap.gif "Kp = 0.16 & Kd = 5"

The goal of this project is to implement and tune a PID controller for the steering angle of a car driving along a track in Udacity term 2 simulator. The work on this project is orgamised as follows:

1. PID controller implemetation.
2. Hyperparameters Manual tuning .
3. Twiddle algorithm for hyperparameters tuning.


## 1. PID controller implemetation

* I Implmented the base PID formula introduced in the lesson under the `TotalError()` method in PID class as follows :

`totalError = - ( Kp * p_error + Kd * d_error + Ki * i_error )` where `Kp`, `Kd` and `Ki` are the controller hyperparameters. And `p_error`, `d_error` and `i_error` are respectively the *proportional, derivative and the integral error* that are updated in each step using the current value of the Cross Track Error (CTE).

## 2. Manual tuning of the hyperparameters.

I started intially with a manual tuning of the controller hyperparameters to see how the various "side effects" discussed during the lesson (overshoot, static error ) translate into the car driving around the track in the simualtor :
* For example, setting all hyperparameters to 0 forces the totalError to 0 and thus the steering angle is set to 0 no matter what the *CTE* value is. This results obviously the in the car driving outside the track on the first turn as shown below.

    ![alt text][image1]

* Then, I started playing only with the proportional hyperparameter setting to various values. for example setting to 1, made the car compensate for CTE when it is offcenter by steering in the opposite direction but then this was resulting in large overshoot and oscillations.


Kp = 1         |  Kp = 0.25
:-------------------------:|:-------------------------:
![alt text][image2]       |  ![alt text][image3]




   
* To compensate for this overshoot I introduced the derivative term by setting its coef to various values.

**Kp = 0.25 - kd = 5**
    
![alt text][image4]

* Since I wasn't able to see a significant positive effect when playing with the integral term, I kept it value at 0.

* I Ended up my manual tuning phase with the following set of hyperparameters `Kp = 0.2, Ki = 0, kd =  5`.


## 3. Twiddle algorithm for hyperparameters tuning:  




To step up from the manual tuning I decided to implement twiddle algorithm to tune the hyperparameters as introduced in the lesson:
* In my implementation I took the score of twiddle as the number of steps/iterations the car takes without getting outside of the track.
* When my twiddle flag is set to ON, the car starts with a set of initial hyperparameters, then drives along track until it is outside the track ( `abs(CTE) > 2.3` ), then it updates a single hyperparameter following the twiddle algorithm and starts a new round of the car using the updated set of hyperparameters. My stop condition is a max value for the number of rounds the car takes part in. 
* A I couldn't see a significant positive effect of changing `Ki` ( regarding the static error ) during manual tuning I decided to skip it during twiddle as well for the algorithm to converge faster.

* Might switch the minimal accumlated cte as the best score
* Ideally you would check the accumulated cte over a lap or so, but no location data are available here, instead I'll go for number of a given number of steps say 1000 or 2000 and starting with the best parameters I already found with manual tuning
    
* **Challenges I faced in twiddle algorithm :**


    * The choice of the appropriate initial rate of change `dp` for eachhyper parameters in the Twiddle algorithm (for Kp -> 0.01, then kd -> 0.5, Ki -> 0.00)
    * Choosing the threshold that decides that the car went outside the track and starts a new round. High tolernce values close to 3 didn't penalise enough the sets of hyperparameters introducing large osilations (provided that the cars CTE stays under the threshold 3), Small tolerence values (closer to 1.5) penalize some decent sets of hyperparameters around the turns as this threshold is more likely to be violated around them. The value I ended up keeping (from manual experiments) was *2.3*.

* Final Result video :

Set of hyperparameters: `Kp = 0.16 - Ki = 0 - Kd = 5` at a **speed of 40 MPH**.

![alt text][image5]

You can check the full final video [here](./videos/writeup_videos/my_PID_demo.mp4).


## Current limitations and Future improvements:


My current implemtation of twiddle ( assuming the score = number of steps without going outside of the track ) is inefficient as soon as we approach a range of hyperparameters allowing the car to perfom full laps witout going outside the track : This score doesn't know to compare between a car that runs the lap perfectly in the center of the lane and a car that osscilates a lot but still inside the track. 
    
One quick fix I tried to tacke this was being more harsh on the outside of track threshold (For a car to be be considred in track I tolerated a CTE up to 2.7 m on both sides).
    
But the better soution to this problem would be to change the score for twiddle to the accumulated CTE along a fixed number of steps. Ideally it would have been along 1 lap but location data of the car were not availble.
 
Last but not least PID controller for the speed as well, to slow down when turning ( applying large values of steering angles) is worth implemeting to gain even more speed especially when in straight lanes.