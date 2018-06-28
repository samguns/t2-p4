# Term2 - PID Controller Project Writeup
Self-Driving Car Engineer Nanodegree Program

---

## Reflection


### 1. Describe your implementation.

Because this project starts under the assumption that the simulator feeds back a CrossTrackError (`cte`) in a constant one unit interval, it simplifies the controller without revealing the essence of how a PID controller works. Then I implemented the PID controller pretty much the same as what in lectures shown. That is, P term is `Kp * cte`, I term is `Ki * accumulated cte` and D term is `Kd * (cte - prev_cte)`. So the output steering angle is `-(P term + I term + D term)`.


### 2. Describe the effect each of the P, I, D components had in your implementation.
The P component `-(Kp * cte)` applies a proportional correction to the steering angle. The larger the CrossTrackError was, the larger it reverses the steering, in order to maintain a smaller CTE. But this component doesn't take the changing CTE rate into account, using it alone only brings the vehicle oscillating around the target position (zero CTE).


The D component `-(Kd * (cte - prev_cte))`, applies a force to slow down the error rate. It penalizes relatively faster changing error rates. However, if the error was biased at some point, it can only bring the vehicle back to the biased position (zero CTE rate). Normally, combining P and D components makes the vehicle converge to target position (zero CTE).


The shortcomings of D component leads us to consider how far we're away from the target position accumulatively, hence introducing the I component (-(`Ki * accumulated cte`)). It kicks in whenever a biased error exists that D component can't detect. This means this component is a compliment to P and D and should be as small as possible, since we're not expecting large biases.


### 3. Describe how the final hyperparameters were chosen.
I tried to use twiddle in the first place, but it's complicated to implement. And the fact that the vehicle has to run at lease two laps to accomplish a single twiddle loop made me seeking out other solutions. I searched the web and started my manual parameter tuning according to [this article](https://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops). Under the circumstance of this project, the simulator always starts at the targeting position. The vehicle has already been on the right track. It doesn't make sense to tune the Kp first. So I chose the method [Joe Baker](https://robotics.stackexchange.com/users/308/joe-baker) proposed. That is:
>1. Set all gains to 0.
2. Increase Kd until the system oscillates.
3. Reduce Kd by a factor of 2-4.
4. Set Kp to about 1% of Kd.
5. Increase Kp until oscillations start.
6. Decrease Kp by a factor of 2-4.
7. Set Ki to about 1% of Kp.
8. Increase Ki until oscillations start.
9. Decrease Ki by a factor of 2-4.

I ended up with the hyperparameters `Kp: 0.23`, `Ki: 0.002` and `Kd: 20`.
